// Copyright 2026 KUKA Hungaria Kft.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <arpa/inet.h>
#include <gtest/gtest.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "kuka/external-control-sdk/kss/mxa/client.h"
#include "kuka/external-control-sdk/kss/mxa/mxa_wrapper.h"
#include "kuka/external-control-sdk/utils/os-core-udp-communication/publisher.h"

using kuka::external::control::EventHandler;
using kuka::external::control::ReturnCode;
using kuka::external::control::kss::IStatusUpdateHandler;
using kuka::external::control::kss::StatusUpdate;
using kuka::external::control::kss::mxa::Client;
using kuka::external::control::kss::mxa::mxAWrapper;
using os::core::udp::communication::Publisher;
using os::core::udp::communication::SocketAddress;

namespace
{

constexpr int kAxisGroup = 1;
constexpr int kTelegramSize = 256;
// Where KRC_READAXISGROUP reads the two flags from the telegram
constexpr int kBrakeTestRequestedByte = 2;
constexpr uint8_t kBrakeTestRequestedMask = 1 << 6;
constexpr int kBrakeTestWarningByte = 3;
constexpr uint8_t kBrakeTestWarningMask = 1 << 1;

// Binds an ephemeral UDP port and hands it back, so parallel test binaries
// cannot collide on a fixed port
int GetEphemeralUdpPort()
{
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = 0;
  bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
  socklen_t len = sizeof(addr);
  getsockname(fd, reinterpret_cast<sockaddr *>(&addr), &len);
  int port = ntohs(addr.sin_port);
  close(fd);
  return port;
}

class QuietEventHandler : public EventHandler
{
public:
  void OnSampling() override {}
  void OnError(const std::string &) override {}
};

// Records every status update with the number of telegrams received before it
class RecordingStatusHandler : public IStatusUpdateHandler
{
public:
  using Record = std::pair<int, StatusUpdate>;

  void OnStatusUpdateReceived(const StatusUpdate & update) override
  {
    std::lock_guard<std::mutex> lk{mtx_};
    updates_.emplace_back(telegrams_, update);
  }

  void OnTelegramReceived() override
  {
    std::lock_guard<std::mutex> lk{mtx_};
    telegrams_++;
  }

  int Telegrams()
  {
    std::lock_guard<std::mutex> lk{mtx_};
    return telegrams_;
  }

  std::vector<Record> Updates()
  {
    std::lock_guard<std::mutex> lk{mtx_};
    return updates_;
  }

private:
  std::mutex mtx_;
  int telegrams_ = 0;
  std::vector<Record> updates_;
};

bool WaitFor(const std::function<bool()> & condition)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!condition())
  {
    if (std::chrono::steady_clock::now() > deadline)
    {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return true;
}

bool BothFlags(const StatusUpdate & update)
{
  return update.brake_test_requested_ && update.brake_test_warning_;
}

bool NoFlag(const StatusUpdate & update)
{
  return !update.brake_test_requested_ && !update.brake_test_warning_;
}

}  // namespace

// The flags are not outputs of the Automatic External block; the wrapper reads
// them from the axis group state the read block fills
TEST(MxaBrakeTestStatus, WrapperReadsTheAxisGroupState)
{
  auto & state = KRC_AXISGROUPREFARR[kAxisGroup].AUTEXTSTATE;
  state.BRAKETEST_REQ_INT = false;
  state.BRAKETEST_WARN = false;
  mxAWrapper wrapper;
  EXPECT_FALSE(wrapper.brakeTestRequested());
  EXPECT_FALSE(wrapper.brakeTestWarning());

  state.BRAKETEST_REQ_INT = true;
  EXPECT_TRUE(wrapper.brakeTestRequested());
  EXPECT_FALSE(wrapper.brakeTestWarning());

  state.BRAKETEST_REQ_INT = false;
  state.BRAKETEST_WARN = true;
  EXPECT_FALSE(wrapper.brakeTestRequested());
  EXPECT_TRUE(wrapper.brakeTestWarning());
  state.BRAKETEST_WARN = false;
}

// Feeds the live keep-alive thread over loopback: the status update that
// carries the flags is dispatched in the cycle of the telegram that set them
TEST(MxaBrakeTestStatus, ReportedWithTheTelegramThatCarriesIt)
{
  const int kClientPort = GetEphemeralUdpPort();

  Client client("127.0.0.1", kClientPort);
  client.RegisterEventHandler(std::make_unique<QuietEventHandler>());
  auto handler = std::make_unique<RecordingStatusHandler>();
  auto * records = handler.get();
  client.RegisterStatusUpdateHandler(std::move(handler));
  ASSERT_EQ(client.Setup().return_code, ReturnCode::OK);

  Publisher sender(SocketAddress("127.0.0.1", kClientPort), SocketAddress(), false);
  ASSERT_EQ(sender.Setup(), Publisher::ErrorCode::kSuccess);
  uint8_t telegram[kTelegramSize] = {};

  // A plain telegram first, so the flags arrive as a change of their own
  ASSERT_EQ(sender.Send(telegram, kTelegramSize), Publisher::ErrorCode::kSuccess);
  ASSERT_TRUE(WaitFor([&] { return records->Telegrams() >= 1 && !records->Updates().empty(); }));
  for (const auto & [telegrams, update] : records->Updates())
  {
    EXPECT_TRUE(NoFlag(update)) << "flag set before the telegram carrying it, at " << telegrams;
  }

  telegram[kBrakeTestRequestedByte] |= kBrakeTestRequestedMask;
  telegram[kBrakeTestWarningByte] |= kBrakeTestWarningMask;
  ASSERT_EQ(sender.Send(telegram, kTelegramSize), Publisher::ErrorCode::kSuccess);
  ASSERT_TRUE(WaitFor(
    [&]
    {
      const auto updates = records->Updates();
      return std::any_of(
        updates.begin(), updates.end(),
        [](const auto & record) { return BothFlags(record.second); });
    }))
    << "no status update carried the brake-test flags";
  auto updates = records->Updates();
  const auto with_flags = std::find_if(
    updates.begin(), updates.end(), [](const auto & record) { return BothFlags(record.second); });
  EXPECT_EQ(with_flags->first, 2) << "reported with telegram " << with_flags->first;
  for (auto before = updates.begin(); before != with_flags; ++before)
  {
    EXPECT_TRUE(NoFlag(before->second)) << "a flag set before the telegram carrying it";
  }

  // Cleared the same way
  telegram[kBrakeTestRequestedByte] = 0;
  telegram[kBrakeTestWarningByte] = 0;
  ASSERT_EQ(sender.Send(telegram, kTelegramSize), Publisher::ErrorCode::kSuccess);
  ASSERT_TRUE(WaitFor(
    [&]
    {
      const auto latest = records->Updates();
      return latest.back().first >= 3 && NoFlag(latest.back().second);
    }))
    << "the flags did not clear";
  EXPECT_EQ(records->Updates().back().first, 3);
}
