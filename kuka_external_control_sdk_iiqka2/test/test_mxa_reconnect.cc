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

#include <chrono>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>
#include <string>
#include <thread>
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
// The driver's first-read bound, 5000 ms, in 4 ms mxA cycles
constexpr int kFirstReadBoundCycles = 1250;

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
  // The KRC already held a state for this order id, so it did not run the command
  bool ignored;
};

constexpr int kOrderStateActive = 5;
constexpr int kOrderStateDone = 6;
constexpr int kOrderStateAborted = 8;
constexpr int kStateReturnSlots = 10;

// Models the KRC at telegram level, including the "General motion enable" stop message
class FakeKrc
{
public:
  bool dispatcher_active = false;
  bool drives_powered = true;
  bool motion_enable_message = false;
  // A $CORRECTION stop: confirming does not clear it, only a program reset does
  bool correction_stop = false;
  bool rsi_running = false;
  std::vector<Command> commands;
  std::vector<std::vector<uint8_t>> sent;

  struct OrderState
  {
    int order_id;
    int state;
  };
  // The KRC's state return table, newest last; it outlives a link and a PLC process
  std::vector<OrderState> state_return;

  bool HoldsOrder(int order_id) const
  {
    for (const auto & entry : state_return)
    {
      if (entry.order_id == order_id)
      {
        return true;
      }
    }
    return false;
  }

  void ReportOrder(int order_id, int state)
  {
    for (auto & entry : state_return)
    {
      if (entry.order_id == order_id)
      {
        entry.state = state;
        return;
      }
    }
    state_return.push_back({order_id, state});
    if (state_return.size() > kStateReturnSlots)
    {
      state_return.erase(state_return.begin());
    }
  }

  void FillTelegram(uint8_t * in)
  {
    std::memset(in, 0, kTelegramSize);
    heartbeat_ = heartbeat_ % 250 + 1;
    SetBit(in, 0, 0, true);  // RC_RDY1
    SetBit(in, 0, 1, true);  // ALARM_STOP
    SetBit(in, 0, 2, true);  // USER_SAFE
    SetBit(in, 0, 3, drives_powered);
    SetBit(in, 0, 4, true);  // ROB_CAL
    SetBit(in, 0, 6, motion_enable_message || correction_stop);
    SetBit(in, 1, 1, dispatcher_active);
    SetBit(in, 2, 4, true);  // EXT
    in[4] = heartbeat_;
    in[59] = heartbeat_;
    for (std::size_t i = 0; i < state_return.size(); i++)
    {
      PutDint(in, 8 + 4 * static_cast<int>(i), state_return[i].order_id);
      in[48 + i] = static_cast<uint8_t>(state_return[i].state);
    }

    const auto & command = KRC_AXISGROUPREFARR[kAxisGroup].COMMAND;
    if (command.ORDERID == 0)
    {
      return;
    }
    if (command.ORDERID != last_order_id_)
    {
      last_order_id_ = command.ORDERID;
      const int tech_function = command.COMMANDID == kTechFunctionCmdId ? command.CMDPARINT[41] : 0;
      const bool ignored = HoldsOrder(command.ORDERID);
      commands.push_back({command.ORDERID, command.COMMANDID, tech_function, ignored});
      if (!ignored)
      {
        Execute(command.ORDERID, tech_function);
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
    else if (Bit(out, 1, 4) && !confirm_last_)
    {
      motion_enable_message = false;
    }
    // The KRC confirms on the rising edge
    confirm_last_ = Bit(out, 1, 4);
    if (Bit(out, 1, 6))
    {
      drives_powered = true;
    }
    if (!Bit(out, 1, 5))
    {
      drives_powered = false;
    }
    if (
      Bit(out, 4, 0) && !motion_enable_message && !correction_stop && drives_powered && move_enable)
    {
      dispatcher_active = true;
    }
  }

  // A link drop stops the robot: the dispatcher is deselected and motion enable is lost
  void DropLink()
  {
    AbortRsi();
    dispatcher_active = false;
    motion_enable_message = true;
  }

  // As on hardware after a link drop during RSI: the dispatcher is halted, not deselected
  void StopByCorrection()
  {
    DropLink();
    correction_stop = true;
  }

private:
  void Execute(int order_id, int tech_function)
  {
    if (tech_function == kTechFunctionCancel)
    {
      AbortRsi();
      dispatcher_active = false;
      correction_stop = false;
      ReportOrder(order_id, kOrderStateDone);
    }
    else if (tech_function == kTechFunctionStart)
    {
      if (dispatcher_active)
      {
        rsi_running = true;
        rsi_order_ = order_id;
        ReportOrder(order_id, kOrderStateActive);
      }
    }
    else
    {
      ReportOrder(order_id, kOrderStateDone);
    }
  }

  void AbortRsi()
  {
    if (rsi_running)
    {
      ReportOrder(rsi_order_, kOrderStateAborted);
    }
    rsi_running = false;
  }

  int rsi_order_ = 0;
  bool confirm_last_ = false;
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
    client_.ContinueOrderIds();
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

int LastOrderId(const FakeKrc & krc);

// Order ids, the echoed state return ids and the checksum may differ, nothing else
std::vector<uint8_t> WithoutOrderIds(std::vector<uint8_t> telegram)
{
  for (std::size_t byte = 0; byte < telegram.size(); byte++)
  {
    const bool state_return_id = byte >= 12 && byte < 52;
    const bool order_id_or_checksum = byte >= 64 && byte < 74 && byte != 68 && byte != 69;
    if (state_return_id || order_id_or_checksum)
    {
      telegram[byte] = 0;
    }
  }
  return telegram;
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
  EXPECT_GT(FirstOrderId(reconnect_krc), LastOrderId(stuck_krc)) << "an order id was reused";
  for (std::size_t i = 0; i < fresh_krc.sent.size(); i++)
  {
    ASSERT_EQ(WithoutOrderIds(reconnect_krc.sent[i]), WithoutOrderIds(fresh_krc.sent[i]))
      << "telegram " << i << " differs";
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

namespace
{

struct StartResult
{
  int cancel_cycle = -1;
  int start_cycle = -1;
};

// Cycles until the RSI start command reaches the KRC, within the driver's first-read bound
StartResult RunStart(Client & client, Link & link, FakeKrc & krc)
{
  const std::size_t known = krc.commands.size();
  StartResult result;
  StartRsi(client);
  for (int i = 0; i < kFirstReadBoundCycles && result.start_cycle < 0; i++)
  {
    link.Cycle();
    for (std::size_t c = known; c < krc.commands.size(); c++)
    {
      if (krc.commands[c].ignored)
      {
        continue;
      }
      if (krc.commands[c].tech_function == kTechFunctionCancel && result.cancel_cycle < 0)
      {
        result.cancel_cycle = i;
      }
      if (krc.commands[c].tech_function == kTechFunctionStart && krc.rsi_running)
      {
        result.start_cycle = i;
      }
    }
  }
  return result;
}

bool ExtStartSent(const FakeKrc & krc)
{
  for (const auto & telegram : krc.sent)
  {
    if (Bit(telegram.data(), 1, 2))
    {
      return true;
    }
  }
  return false;
}

}  // namespace

TEST(MxaCorrectionStop, FirstStartOfANewLinkResetsTheHaltedProgram)
{
  FakeKrc krc;
  krc.StopByCorrection();
  auto client = MakeClient();
  Link link(*client, krc, 1);
  link.Cycles(kCyclesToStart);

  const auto result = RunStart(*client, link, krc);

  ASSERT_GE(result.start_cycle, 0) << "no start within the bound, KRC_AUTOSTART state "
                                   << client->mxa_wrapper_.mxa_auto_start_.State();
  ASSERT_GE(result.cancel_cycle, 0);
  EXPECT_LT(result.cancel_cycle, result.start_cycle);
  EXPECT_LT(result.start_cycle, Client::kStopMessageConfirmCycles) << "reset only by the fallback";
  EXPECT_TRUE(krc.dispatcher_active);
  EXPECT_FALSE(krc.correction_stop);
  EXPECT_FALSE(ExtStartSent(krc));
  client->ResetRSI();
}

TEST(MxaCorrectionStop, StopMessageThatOutlivesTheConfirmIsResetOnce)
{
  FakeKrc krc;
  krc.DropLink();
  auto client = MakeClient();
  Link link(*client, krc, 1);
  link.Cycles(kCyclesToStart);
  ASSERT_GE(RunStart(*client, link, krc).start_cycle, 0);
  client->RequestRsiCancel();
  link.Cycles(Client::kRsiCancelHoldCycles + 10);
  ASSERT_TRUE(client->IsRsiCancelled());

  // Raised with the link up, so nothing but the stuck confirm reveals it
  krc.StopByCorrection();
  link.Cycles(10);
  const std::size_t known = krc.commands.size();
  const auto result = RunStart(*client, link, krc);

  ASSERT_GE(result.start_cycle, 0) << "no start within the bound, KRC_AUTOSTART state "
                                   << client->mxa_wrapper_.mxa_auto_start_.State();
  ASSERT_GE(result.cancel_cycle, 0);
  int cancels = 0;
  for (std::size_t c = known; c < krc.commands.size(); c++)
  {
    cancels += krc.commands[c].tech_function == kTechFunctionCancel ? 1 : 0;
  }
  EXPECT_EQ(cancels, 1);
  EXPECT_TRUE(krc.dispatcher_active);
  EXPECT_FALSE(ExtStartSent(krc));
  client->ResetRSI();
}

TEST(MxaCorrectionStop, LostLinkArmsAResetBeforeTheNextStart)
{
  const int port = GetEphemeralUdpPort();
  Client client("127.0.0.1", port);
  client.RegisterEventHandler(std::make_unique<QuietEventHandler>());
  ASSERT_EQ(client.Setup().return_code, ReturnCode::OK);
  client.reset_before_start_ = false;

  // One telegram brings the link up; the silence after it is the loss
  const int fd = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = htons(port);
  uint8_t telegram[kTelegramSize] = {};
  sendto(fd, telegram, sizeof(telegram), 0, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
  close(fd);

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!client.reset_before_start_ && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(client.reset_before_start_);
}

namespace
{

int LastOrderId(const FakeKrc & krc)
{
  return krc.commands.empty() ? 0 : krc.commands.back().order_id;
}

int IgnoredCommands(const FakeKrc & krc)
{
  int ignored = 0;
  for (const auto & command : krc.commands)
  {
    ignored += command.ignored ? 1 : 0;
  }
  return ignored;
}

}  // namespace

// As on hardware: the last link's RSI start was aborted by a drop, and a new client starts
TEST(MxaOrderIds, ReconnectInTheSameProcessDoesNotReuseAnAbortedOrder)
{
  FakeKrc krc;
  krc.DropLink();
  {
    auto client = MakeClient();
    Link link(*client, krc, 1);
    link.Cycles(kCyclesToStart);
    ASSERT_GE(RunStart(*client, link, krc).start_cycle, 0);
    krc.StopByCorrection();
    client->ResetRSI();
  }
  const int aborted = LastOrderId(krc);
  ASSERT_FALSE(krc.rsi_running);

  auto client = MakeClient();
  Link link(*client, krc, 1);
  link.Cycles(kCyclesToStart);
  const auto result = RunStart(*client, link, krc);

  ASSERT_GE(result.start_cycle, 0) << "the RSI start never ran";
  EXPECT_EQ(IgnoredCommands(krc), 0) << "a command reused an id the KRC holds";
  EXPECT_GT(LastOrderId(krc), aborted);
  client->ResetRSI();
}

// A new process has no memory of the last one's ids, only the KRC's state return
TEST(MxaOrderIds, NewProcessStartsAboveTheOrdersTheKrcHolds)
{
  FakeKrc krc;
  krc.StopByCorrection();
  for (int order_id = 1; order_id <= 5; order_id++)
  {
    krc.ReportOrder(order_id, order_id == 5 ? kOrderStateAborted : kOrderStateDone);
  }
  Client::last_order_id_ = 0;

  auto client = MakeClient();
  Link link(*client, krc, 1);
  link.Cycles(kCyclesToStart);
  const auto result = RunStart(*client, link, krc);

  ASSERT_GE(result.start_cycle, 0) << "the RSI start never ran";
  EXPECT_EQ(IgnoredCommands(krc), 0) << "a command reused an id the KRC holds";
  EXPECT_GT(krc.commands.front().order_id, 5);
  client->ResetRSI();
}

TEST(MxaOrderIds, IdsWrapPastTheLargestTheKrcReports)
{
  FakeKrc krc;
  krc.DropLink();
  krc.ReportOrder(2147483645, kOrderStateDone);
  Client::last_order_id_ = 0;

  auto client = MakeClient();
  Link link(*client, krc, 1);
  link.Cycles(kCyclesToStart);
  const auto result = RunStart(*client, link, krc);

  ASSERT_GE(result.start_cycle, 0);
  EXPECT_EQ(krc.commands.front().order_id, 2147483646);
  EXPECT_EQ(LastOrderId(krc), static_cast<int>(krc.commands.size()) - 1);
  EXPECT_EQ(IgnoredCommands(krc), 0);
  client->ResetRSI();
}

// Built over dirty memory, as a Client on a reused heap block is
TEST(MxaTelegram, BitsNoBlockWritesAreSentAsZero)
{
  alignas(Client) unsigned char storage[sizeof(Client)];
  std::memset(storage, 0xFF, sizeof(storage));
  auto * client = new (storage) Client("127.0.0.1", GetEphemeralUdpPort());

  for (std::size_t i = 0; i < kTelegramSize; i++)
  {
    ASSERT_EQ(client->write_buffer_[i], 0) << "write byte " << i;
    ASSERT_EQ(client->read_buffer_[i], 0) << "read byte " << i;
  }
  client->mxa_wrapper_.setmxAInput(client->write_buffer_);
  const uint8_t * telegram = client->write_buffer_;
  EXPECT_EQ(telegram[2] & 0x7E, 0) << "byte 2 bits 1-6";
  EXPECT_EQ(telegram[3], 0);
  EXPECT_FALSE(Bit(telegram, 4, 6));
  EXPECT_EQ(telegram[6], 0);
  EXPECT_EQ(telegram[7], 0);
  client->~Client();
}

namespace
{

int64_t fake_now_ms = 0;
int64_t FakeNow() { return fake_now_ms; }

// Runs the PLC timers on fake_now_ms for its lifetime, from 0
class FakeClock
{
public:
  FakeClock()
  {
    fake_now_ms = 0;
    PLCIEC::time_source_ms = FakeNow;
  }
  ~FakeClock()
  {
    PLCIEC::time_source_ms = nullptr;
    fake_now_ms = 0;
  }
};

}  // namespace

// Two confirm pulse trains ORed into CONF_MESS can hold the bit high, so the KRC sees no edge
TEST(MxaConfirm, EveryStartPhaseConfirmsTheStopMessageWithoutTheFallback)
{
  FakeClock clock;
  std::vector<int> failed_offsets;
  // Covers a whole period of the 200 ms on, 100 ms off confirm pulse
  for (int offset = 0; offset < 75; offset += 5)
  {
    FakeKrc krc;
    krc.DropLink();
    auto client = MakeClient();
    Link link(*client, krc, 1);
    auto cycle = [&]
    {
      fake_now_ms += 4;
      link.Cycle();
    };
    for (int i = 0; i < kCyclesToStart + offset; i++)
    {
      cycle();
    }
    StartRsi(*client);
    bool started = false;
    for (int i = 0; i < kFirstReadBoundCycles && !started; i++)
    {
      cycle();
      started = krc.rsi_running;
    }
    if (!started || client->stop_message_reset_used_)
    {
      failed_offsets.push_back(offset);
    }
    client->ResetRSI();
  }

  EXPECT_TRUE(failed_offsets.empty()) << "stop message not confirmed for " << failed_offsets.size()
                                      << " start phases, first at idle cycle offset "
                                      << (failed_offsets.empty() ? -1 : failed_offsets[0]);
}

// As on hardware: CONF_MESS is already high from the message reset when the confirm starts
TEST(MxaConfirm, ConfirmStartsWithAnEdgeWhenTheBitIsAlreadyHigh)
{
  // A frozen clock holds every confirm pulse in its high phase
  FakeClock clock;
  FakeKrc krc;
  krc.DropLink();
  auto client = MakeClient();
  Link link(*client, krc, 1);
  link.Cycles(kCyclesToStart);
  ASSERT_TRUE(Bit(krc.sent.back().data(), 1, 4)) << "CONF_MESS not high before the start";

  StartRsi(*client);
  bool started = false;
  for (int i = 0; i < kFirstReadBoundCycles && !started; i++)
  {
    link.Cycle();
    started = krc.rsi_running;
  }

  EXPECT_TRUE(started) << "KRC_AUTOSTART state " << client->mxa_wrapper_.mxa_auto_start_.State();
  EXPECT_FALSE(client->stop_message_reset_used_) << "confirmed only after the fallback reset";
  client->ResetRSI();
}

TEST(MxaClientInstance, OnlyOneClientIsSetUpAtATime)
{
  auto first = std::make_unique<Client>("127.0.0.1", GetEphemeralUdpPort());
  ASSERT_EQ(first->Setup().return_code, ReturnCode::OK);
  Client second("127.0.0.1", GetEphemeralUdpPort());
  EXPECT_EQ(second.Setup().return_code, ReturnCode::ERROR);

  first.reset();
  EXPECT_EQ(second.Setup().return_code, ReturnCode::OK);
}
