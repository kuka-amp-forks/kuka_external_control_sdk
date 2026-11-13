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

#include <cstdint>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

#define private public
#include "kuka/external-control-sdk/kss/mxa/client.h"
#undef private

using kuka::external::control::ControlMode;
using kuka::external::control::EventHandler;
using kuka::external::control::ReturnCode;
using kuka::external::control::kss::CycleTime;
using kuka::external::control::kss::mxa::Client;

namespace
{

constexpr int kAxisGroup = 1;
constexpr int kTelegramSize = 256;
constexpr int kTechFunctionCmdId = 35;
constexpr int kTechFunctionStart = 2;
constexpr int kTechFunctionCancel = 3;
// The read block keeps the link offline for its first 100 cycles
constexpr int kCyclesToOnline = 150;
constexpr int kCyclesToStart = 300;

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

bool Bit(const uint8_t * telegram, int byte, int bit) { return (telegram[byte] >> bit) & 1; }

void SetBit(uint8_t * telegram, int byte, int bit, bool value)
{
  telegram[byte] = value ? (telegram[byte] | (1 << bit)) : (telegram[byte] & ~(1 << bit));
}

void PutDint(uint8_t * telegram, int byte, int32_t value)
{
  std::memcpy(telegram + byte, &value, sizeof(value));
}

struct Command
{
  int order_id;
  int command_id;
  int tech_function;
};

// Models the KRC at telegram level, including the "General motion enable" stop message
class FakeKrc
{
public:
  bool dispatcher_active = false;
  bool drives_powered = true;
  bool motion_enable_message = false;
  std::vector<Command> commands;
  std::vector<std::vector<uint8_t>> sent;

  void FillTelegram(uint8_t * in)
  {
    std::memset(in, 0, kTelegramSize);
    heartbeat_ = heartbeat_ % 250 + 1;
    SetBit(in, 0, 0, true);  // RC_RDY1
    SetBit(in, 0, 1, true);  // ALARM_STOP
    SetBit(in, 0, 2, true);  // USER_SAFE
    SetBit(in, 0, 3, drives_powered);
    SetBit(in, 0, 4, true);  // ROB_CAL
    SetBit(in, 0, 6, motion_enable_message);
    SetBit(in, 1, 1, dispatcher_active);
    SetBit(in, 2, 4, true);  // EXT
    in[4] = heartbeat_;
    in[59] = heartbeat_;

    const auto & command = KRC_AXISGROUPREFARR[kAxisGroup].COMMAND;
    if (command.ORDERID == 0)
    {
      return;
    }
    if (command.ORDERID != last_order_id_)
    {
      last_order_id_ = command.ORDERID;
      const int tech_function = command.COMMANDID == kTechFunctionCmdId ? command.CMDPARINT[41] : 0;
      commands.push_back({command.ORDERID, command.COMMANDID, tech_function});
      if (tech_function == kTechFunctionCancel)
      {
        dispatcher_active = false;
      }
    }
    PutDint(in, 178, command.ORDERID);
    PutDint(in, 182, command.COMMANDID);
    PutDint(in, 186, command.ORDERID ^ command.COMMANDID);
  }

  void ReadTelegram(const uint8_t * out)
  {
    sent.emplace_back(out, out + kTelegramSize);
    const bool move_enable = Bit(out, 1, 3);
    if (!move_enable)
    {
      motion_enable_message = true;
    }
    else if (Bit(out, 1, 4))
    {
      motion_enable_message = false;
    }
    if (Bit(out, 1, 6))
    {
      drives_powered = true;
    }
    if (!Bit(out, 1, 5))
    {
      drives_powered = false;
    }
    if (Bit(out, 4, 0) && !motion_enable_message && drives_powered && move_enable)
    {
      dispatcher_active = true;
    }
  }

  // A link drop stops the robot: the dispatcher is deselected and motion enable is lost
  void DropLink()
  {
    dispatcher_active = false;
    motion_enable_message = true;
  }

private:
  int heartbeat_ = 0;
  int last_order_id_ = 0;
};

// The mxA part of one keep-alive cycle, fed by the fake KRC
class Link
{
public:
  Link(Client & client, FakeKrc & krc, int first_tick)
  : client_(client), krc_(krc), tick_(first_tick)
  {
  }

  void Cycle()
  {
    krc_.FillTelegram(in_);
    auto & wrapper = client_.mxa_wrapper_;
    wrapper.getmxAOutput(in_);
    wrapper.mxACycle();
    if (tick_ <= Client::kInitTimeoutTicks + 1)
    {
      wrapper.resetErrors();
      tick_++;
    }
    client_.HandleRsiStart();
    client_.HandleRsiCancel();
    wrapper.setmxAInput(out_);
    krc_.ReadTelegram(out_);
  }

  void Cycles(int count)
  {
    for (int i = 0; i < count; i++)
    {
      Cycle();
    }
  }

private:
  Client & client_;
  FakeKrc & krc_;
  int tick_;
  uint8_t in_[kTelegramSize] = {};
  uint8_t out_[kTelegramSize] = {};
};

std::unique_ptr<Client> MakeClient()
{
  auto client = std::make_unique<Client>("127.0.0.1", GetEphemeralUdpPort());
  client->RegisterEventHandler(std::make_unique<QuietEventHandler>());
  // The keep-alive thread never receives here, so the test runs every cycle
  EXPECT_EQ(client->Setup().return_code, ReturnCode::OK);
  return client;
}

void StartRsi(Client & client)
{
  client.StartRSI(ControlMode::JOINT_POSITION_CONTROL, CycleTime::RSI_4MS);
}

// As on hardware: RSI running, link drop, cancel once the link is back, a latched error
void RunClientIntoTheStuckState(FakeKrc & krc)
{
  auto client = MakeClient();
  Link link(*client, krc, 1);
  link.Cycles(kCyclesToStart);
  StartRsi(*client);
  link.Cycles(kCyclesToStart);
  ASSERT_TRUE(krc.dispatcher_active);
  ASSERT_EQ(krc.commands.back().tech_function, kTechFunctionStart);

  krc.DropLink();
  client->RequestRsiCancel();
  link.Cycles(Client::kRsiCancelHoldCycles + 10);
  ASSERT_TRUE(client->IsRsiCancelled());
  ASSERT_EQ(krc.commands.back().tech_function, kTechFunctionCancel);

  // As logged on hardware after a failed dispatcher start
  KRC_AXISGROUPREFARR[kAxisGroup].INTFBERRORID = 529;
}

int FirstOrderId(const FakeKrc & krc)
{
  return krc.commands.empty() ? 0 : krc.commands[0].order_id;
}

}  // namespace

TEST(MxaReconnect, NewClientSendsWhatANewProcessSends)
{
  FakeKrc fresh_krc;
  fresh_krc.DropLink();
  {
    auto client = MakeClient();
    Link(*client, fresh_krc, 1).Cycles(kCyclesToStart);
  }

  FakeKrc stuck_krc;
  RunClientIntoTheStuckState(stuck_krc);
  FakeKrc reconnect_krc;
  reconnect_krc.DropLink();
  {
    auto client = MakeClient();
    Link(*client, reconnect_krc, 1).Cycles(kCyclesToStart);
  }

  ASSERT_EQ(reconnect_krc.sent.size(), fresh_krc.sent.size());
  EXPECT_EQ(FirstOrderId(fresh_krc), 1);
  EXPECT_EQ(FirstOrderId(reconnect_krc), 1) << "order ids carried over from the destroyed client";
  for (std::size_t i = 0; i < fresh_krc.sent.size(); i++)
  {
    ASSERT_EQ(reconnect_krc.sent[i], fresh_krc.sent[i]) << "telegram " << i << " differs";
  }
}

// The early message reset is skipped when the first telegram arrives late
TEST(MxaReconnect, LatchedErrorOfTheLastClientDoesNotHoldMoveEnableOff)
{
  FakeKrc krc;
  RunClientIntoTheStuckState(krc);
  krc.sent.clear();

  auto client = MakeClient();
  Link link(*client, krc, Client::kInitTimeoutTicks + 2);
  link.Cycles(kCyclesToStart);

  ASSERT_TRUE(client->mxa_wrapper_.isInitialized());
  for (std::size_t i = kCyclesToOnline; i < krc.sent.size(); i++)
  {
    ASSERT_TRUE(Bit(krc.sent[i].data(), 1, 3)) << "MOVE_ENABLE off in telegram " << i;
  }
}

TEST(MxaReconnect, StartAfterTheStuckClientRunsTheFullAutoStart)
{
  FakeKrc krc;
  RunClientIntoTheStuckState(krc);
  krc.commands.clear();
  krc.sent.clear();

  auto client = MakeClient();
  Link link(*client, krc, 1);
  link.Cycles(kCyclesToStart);
  ASSERT_FALSE(krc.dispatcher_active);
  const std::size_t start = krc.sent.size();
  StartRsi(*client);
  link.Cycles(kCyclesToStart);

  int confirm = -1;
  int reset = -1;
  for (std::size_t i = start; i < krc.sent.size(); i++)
  {
    const auto * telegram = krc.sent[i].data();
    EXPECT_FALSE(Bit(telegram, 1, 2)) << "EXT_START in telegram " << i;
    if (confirm < 0 && Bit(telegram, 1, 4) && Bit(telegram, 1, 3))
    {
      confirm = static_cast<int>(i);
    }
    if (reset < 0 && Bit(telegram, 4, 0))
    {
      reset = static_cast<int>(i);
    }
  }
  ASSERT_GE(confirm, 0) << "the stop message was never confirmed with MOVE_ENABLE on";
  ASSERT_GT(reset, confirm);
  EXPECT_TRUE(krc.drives_powered);
  EXPECT_TRUE(krc.dispatcher_active);
  ASSERT_FALSE(krc.commands.empty());
  EXPECT_EQ(krc.commands.back().tech_function, kTechFunctionStart);
  EXPECT_FALSE(client->start_cmd_dispatcher_);
  client->ResetRSI();
}
