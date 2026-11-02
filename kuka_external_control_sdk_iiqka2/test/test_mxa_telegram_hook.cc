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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include "kuka/external-control-sdk/kss/mxa/client.h"
#include "kuka/external-control-sdk/utils/os-core-udp-communication/publisher.h"

using kuka::external::control::ReturnCode;
using kuka::external::control::kss::IStatusUpdateHandler;
using kuka::external::control::kss::StatusUpdate;
using kuka::external::control::kss::mxa::Client;
using os::core::udp::communication::Publisher;
using os::core::udp::communication::SocketAddress;

namespace
{

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

class CountingStatusHandler : public IStatusUpdateHandler
{
public:
  void OnStatusUpdateReceived(const StatusUpdate &) override { status_updates++; }
  void OnTelegramReceived() override { telegrams++; }

  std::atomic<int> status_updates{0};
  std::atomic<int> telegrams{0};
};

// Holds the keep-alive thread in the hook until released, as a local stall would
class HoldingStatusHandler : public IStatusUpdateHandler
{
public:
  void OnStatusUpdateReceived(const StatusUpdate &) override {}
  void OnTelegramReceived() override
  {
    std::unique_lock<std::mutex> lk{mtx};
    telegrams++;
    cv.wait(lk, [this] { return released; });
  }

  void Release()
  {
    {
      std::lock_guard<std::mutex> lk{mtx};
      released = true;
    }
    cv.notify_all();
  }

  int Telegrams()
  {
    std::lock_guard<std::mutex> lk{mtx};
    return telegrams;
  }

private:
  std::mutex mtx;
  std::condition_variable cv;
  int telegrams = 0;
  bool released = false;
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

// Feeds the live keep-alive thread over loopback with identical telegrams, so
// the status never changes after the first one
TEST(MxaTelegramHook, StampsEveryTelegram)
{
  const int kClientPort = GetEphemeralUdpPort();
  constexpr int kTelegrams = 20;
  constexpr int kTelegramSize = 256;

  Client client("127.0.0.1", kClientPort);
  auto handler = std::make_unique<CountingStatusHandler>();
  auto * counts = handler.get();
  client.RegisterStatusUpdateHandler(std::move(handler));
  ASSERT_EQ(client.Setup().return_code, ReturnCode::OK);

  Publisher sender(SocketAddress("127.0.0.1", kClientPort), SocketAddress(), false);
  ASSERT_EQ(sender.Setup(), Publisher::ErrorCode::kSuccess);
  const uint8_t telegram[kTelegramSize] = {};
  for (int i = 0; i < kTelegrams; i++)
  {
    ASSERT_EQ(sender.Send(telegram, kTelegramSize), Publisher::ErrorCode::kSuccess);
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (counts->telegrams < kTelegrams && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  // Let anything still queued arrive, so a surplus call would show up
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  EXPECT_EQ(counts->telegrams, kTelegrams);
  EXPECT_LE(counts->status_updates, 1);
}

// A telegram that arrives while the keep-alive thread cannot run stays visible
// as pending, and stops being pending once the thread has received it
TEST(MxaTelegramHook, ReportsTelegramPendingUntilReceived)
{
  const int kClientPort = GetEphemeralUdpPort();
  constexpr int kTelegramSize = 256;

  Client client("127.0.0.1", kClientPort);
  EXPECT_FALSE(client.IsTelegramPending());
  auto handler = std::make_unique<HoldingStatusHandler>();
  auto * hold = handler.get();
  client.RegisterStatusUpdateHandler(std::move(handler));
  // Released before the client joins its keep-alive thread, even on a failed assertion
  struct ReleaseOnExit
  {
    HoldingStatusHandler * hold;
    ~ReleaseOnExit() { hold->Release(); }
  } release_on_exit{hold};
  ASSERT_EQ(client.Setup().return_code, ReturnCode::OK);
  EXPECT_FALSE(client.IsTelegramPending());

  Publisher sender(SocketAddress("127.0.0.1", kClientPort), SocketAddress(), false);
  ASSERT_EQ(sender.Setup(), Publisher::ErrorCode::kSuccess);
  const uint8_t telegram[kTelegramSize] = {};

  ASSERT_EQ(sender.Send(telegram, kTelegramSize), Publisher::ErrorCode::kSuccess);
  ASSERT_TRUE(WaitFor([hold] { return hold->Telegrams() == 1; }));
  EXPECT_FALSE(client.IsTelegramPending());

  ASSERT_EQ(sender.Send(telegram, kTelegramSize), Publisher::ErrorCode::kSuccess);
  EXPECT_TRUE(WaitFor([&client] { return client.IsTelegramPending(); }));
  EXPECT_EQ(hold->Telegrams(), 1);

  hold->Release();
  EXPECT_TRUE(WaitFor([hold] { return hold->Telegrams() == 2; }));
  EXPECT_TRUE(WaitFor([&client] { return !client.IsTelegramPending(); }));
}

}  // namespace
