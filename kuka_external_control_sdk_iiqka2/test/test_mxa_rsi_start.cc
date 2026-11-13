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
#include <memory>
#include <string>
#include <thread>
#include <vector>

#define private public
#include "kuka/external-control-sdk/kss/mxa/client.h"
#undef private

using kuka::external::control::ControlMode;
using kuka::external::control::EventHandler;
using kuka::external::control::kss::CycleTime;
using kuka::external::control::kss::mxa::Client;
using kuka::external::control::kss::mxa::mxAWrapper;

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

class RecordingEventHandler : public EventHandler
{
public:
  void OnSampling() override { sampling_count++; }
  void OnError(const std::string & reason) override { errors.push_back(reason); }

  int sampling_count = 0;
  std::vector<std::string> errors;
};

// Drives the mxA function blocks through the shared axis group state, the way
// the controller would, without a KRC
class MxaRsiStart : public ::testing::Test
{
protected:
  static constexpr int kAxisGroup = 1;
  static constexpr int kTechFunctionCmdId = 35;
  static constexpr int kTechFunctionStart = 2;
  static constexpr int kTechFunctionCancel = 3;
  static constexpr int kOrderStateActive = 5;
  static constexpr int kOrderStateDone = 6;
  static constexpr int kOrderStateAborted = 8;
  // Enough cycles for one re-issue after a reported abort
  static constexpr int kBackoff = Client::kRsiStartBackoffCycles + 5;
  // Enough cycles for one re-issue after a silent drop
  static constexpr int kTimeout = Client::kRsiStartAckTimeoutCycles + kBackoff;

  void SetUp() override
  {
    auto & group = KRC_AXISGROUPREFARR[kAxisGroup];
    group = AXIS_GROUP_REF{};
    group.INITIALIZED = true;
    group.ONLINE = true;
    group.READDONE = true;
    group.READAXISGROUPINIT = true;
    group.AUTEXTSTATE.EXT = true;
    group.AUTEXTSTATE.ALARM_STOP = true;
    group.AUTEXTSTATE.USER_SAFE = true;
    group.AUTEXTSTATE.PERI_RDY = true;
    group.AUTEXTSTATE.PRO_ACT = true;

    auto handler = std::make_unique<RecordingEventHandler>();
    handler_ = handler.get();
    client_.RegisterEventHandler(std::move(handler));
  }

  void TearDown() override
  {
    // The destructor would wait for the keep-alive thread to cancel
    client_.ResetRSI();
  }

  void StartRsi() { client_.StartRSI(ControlMode::JOINT_POSITION_CONTROL, CycleTime::RSI_4MS); }

  // One keep-alive cycle, with the KRC echoing every new command unless echo_ is off
  void Cycle()
  {
    client_.mxa_wrapper_.mxa_aut_ext_.OnCycle();
    client_.HandleRsiStart();
    client_.HandleRsiCancel();

    auto & group = KRC_AXISGROUPREFARR[kAxisGroup];
    if (echo_ && group.COMMAND.ORDERID != 0 && group.COMMAND.ORDERID != group.CMDSTATE.ORDERIDRET)
    {
      EXPECT_EQ(group.COMMAND.COMMANDID, kTechFunctionCmdId);
      if (group.COMMAND.CMDPARINT[41] == kTechFunctionCancel)
      {
        cancels_.push_back(group.COMMAND.ORDERID);
      }
      else
      {
        EXPECT_EQ(group.COMMAND.CMDPARINT[41], kTechFunctionStart);
        issued_.push_back(group.COMMAND.ORDERID);
      }
      group.CMDSTATE.ORDERIDRET = group.COMMAND.ORDERID;
      group.CMDSTATE.CMDIDRET = group.COMMAND.COMMANDID;
    }
  }

  void Cycles(int count)
  {
    for (int i = 0; i < count; i++)
    {
      Cycle();
    }
  }

  void ReportOrderState(int order_id, int state)
  {
    auto & state_return = KRC_AXISGROUPREFARR[kAxisGroup].CMDSTATE.STATERETURN[1];
    state_return.SR_ORDERID = order_id;
    state_return.SR_STATE = state;
  }

  Client client_{"127.0.0.1", 1};
  RecordingEventHandler * handler_ = nullptr;
  std::vector<int> issued_;
  std::vector<int> cancels_;
  bool echo_ = true;
};

TEST_F(MxaRsiStart, RunningDispatcherGetsOneStartCommand)
{
  StartRsi();
  Cycles(50);

  EXPECT_EQ(issued_.size(), 1u);
  EXPECT_FALSE(client_.start_cmd_dispatcher_);
  EXPECT_TRUE(client_.start_rsi_);
  EXPECT_EQ(handler_->sampling_count, 1);
  EXPECT_TRUE(handler_->errors.empty());
}

TEST_F(MxaRsiStart, SecondStartAfterCancelGetsANewCommand)
{
  StartRsi();
  Cycles(5);
  ASSERT_EQ(issued_.size(), 1u);
  client_.ResetRSI();

  StartRsi();
  Cycles(5);

  ASSERT_EQ(issued_.size(), 2u);
  EXPECT_GT(issued_[1], issued_[0]);
}

TEST_F(MxaRsiStart, AbortedStartIsReissuedOnce)
{
  StartRsi();
  Cycles(5);
  ASSERT_EQ(issued_.size(), 1u);

  ReportOrderState(issued_[0], kOrderStateAborted);
  Cycles(Client::kRsiStartBackoffCycles - 5);
  EXPECT_EQ(issued_.size(), 1u) << "re-issued inside the backoff";
  Cycles(10);

  ASSERT_EQ(issued_.size(), 2u);
  EXPECT_EQ(issued_[1], issued_[0] + 1);
  EXPECT_TRUE(client_.start_rsi_);
  EXPECT_TRUE(handler_->errors.empty());
}

TEST_F(MxaRsiStart, ConfirmationDuringTheBackoffEndsTheStart)
{
  StartRsi();
  Cycles(5);
  ASSERT_EQ(issued_.size(), 1u);

  ReportOrderState(issued_[0], kOrderStateAborted);
  Cycles(10);
  client_.ConfirmRsiStart();
  Cycles(kBackoff + 10);

  EXPECT_EQ(issued_.size(), 1u) << "re-issued against a running session";
  EXPECT_FALSE(client_.start_rsi_);
  EXPECT_TRUE(handler_->errors.empty());
}

TEST_F(MxaRsiStart, CancelStopsAPendingRetry)
{
  StartRsi();
  Cycles(5);
  ASSERT_EQ(issued_.size(), 1u);
  ReportOrderState(issued_[0], kOrderStateAborted);
  Cycles(10);

  client_.RequestRsiCancel();
  Cycles(kBackoff + 10);

  EXPECT_EQ(issued_.size(), 1u) << "re-issued after the cancel";
  EXPECT_EQ(cancels_.size(), 1u);
  EXPECT_TRUE(client_.IsRsiCancelled());
}

TEST_F(MxaRsiStart, CancelRequestReturnsImmediately)
{
  // No link: the client was never set up, so no keep-alive thread runs
  StartRsi();

  const auto before = std::chrono::steady_clock::now();
  const auto status = client_.RequestRsiCancel();
  const auto elapsed = std::chrono::steady_clock::now() - before;

  EXPECT_EQ(status.return_code, kuka::external::control::ReturnCode::OK);
  EXPECT_LT(elapsed, std::chrono::milliseconds(1));
  EXPECT_FALSE(client_.start_rsi_);
  EXPECT_TRUE(client_.cancel_requested_);
  EXPECT_FALSE(client_.IsRsiCancelled());
}

TEST_F(MxaRsiStart, CancelIsFinishedByTheKeepAliveThread)
{
  StartRsi();
  Cycles(5);
  ASSERT_EQ(issued_.size(), 1u);

  client_.RequestRsiCancel();
  Cycles(Client::kRsiCancelHoldCycles - 1);
  EXPECT_TRUE(cancels_.empty()) << "cancelled inside the hold";
  EXPECT_FALSE(client_.IsRsiCancelled());
  Cycles(5);

  ASSERT_EQ(cancels_.size(), 1u);
  EXPECT_TRUE(client_.IsRsiCancelled());
  EXPECT_FALSE(client_.cancel_requested_);
  EXPECT_FALSE(client_.rsi_started_);
  EXPECT_FALSE(KRC_AXISGROUPREFARR[kAxisGroup].AUTEXTCONTROL.MOVE_ENABLE);

  Cycles(50);
  EXPECT_EQ(cancels_.size(), 1u) << "cancel re-issued after it was done";
}

TEST_F(MxaRsiStart, StartDuringACancelIsRefused)
{
  StartRsi();
  Cycles(5);
  ASSERT_EQ(issued_.size(), 1u);
  client_.RequestRsiCancel();

  const auto refused = client_.StartRSI(ControlMode::JOINT_POSITION_CONTROL, CycleTime::RSI_4MS);
  EXPECT_EQ(refused.return_code, kuka::external::control::ReturnCode::ERROR);
  Cycles(Client::kRsiCancelHoldCycles + 5);
  ASSERT_TRUE(client_.IsRsiCancelled());
  EXPECT_EQ(issued_.size(), 1u);

  const auto accepted = client_.StartRSI(ControlMode::JOINT_POSITION_CONTROL, CycleTime::RSI_4MS);
  EXPECT_EQ(accepted.return_code, kuka::external::control::ReturnCode::OK);
  Cycles(5);
  EXPECT_EQ(issued_.size(), 2u);
}

TEST_F(MxaRsiStart, BlockingCancelWaitsForTheKeepAliveThread)
{
  StartRsi();
  Cycles(5);
  ASSERT_EQ(issued_.size(), 1u);

  // CancelRSI waits for the keep-alive thread, which this test stands in for
  std::thread cancel([this] { client_.CancelRSI(); });
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (!client_.cancel_requested_ && std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const bool cancel_requested = client_.cancel_requested_;
  if (cancel_requested)
  {
    Cycles(Client::kRsiCancelHoldCycles + 5);
  }
  else
  {
    client_.SetToCancelled();
  }
  cancel.join();

  ASSERT_TRUE(cancel_requested);
  EXPECT_EQ(cancels_.size(), 1u);
  EXPECT_TRUE(client_.IsRsiCancelled());
  EXPECT_FALSE(client_.rsi_started_);
}

TEST_F(MxaRsiStart, UnechoedStartIsTimedOut)
{
  echo_ = false;
  StartRsi();
  Cycles(kTimeout * (Client::kMaxRsiStartRetries + 1));

  EXPECT_FALSE(client_.start_rsi_);
  ASSERT_EQ(handler_->errors.size(), 1u);
  EXPECT_NE(handler_->errors[0].find("not acknowledged"), std::string::npos);
}

TEST_F(MxaRsiStart, UnacknowledgedStartIsTimedOutAndReissued)
{
  StartRsi();
  Cycles(5);
  ASSERT_EQ(issued_.size(), 1u);

  // The KRC never reports the order: no ACTIVE, no DONE, no ABORTED
  Cycles(Client::kRsiStartAckTimeoutCycles - 10);
  EXPECT_EQ(issued_.size(), 1u) << "timed out early";
  Cycles(kBackoff + 10);

  ASSERT_EQ(issued_.size(), 2u);
  EXPECT_EQ(issued_[1], issued_[0] + 1);
  EXPECT_TRUE(client_.start_rsi_);
  EXPECT_TRUE(handler_->errors.empty());
}

TEST_F(MxaRsiStart, ConfirmedStartIsNeverTimedOut)
{
  StartRsi();
  Cycles(5);
  ASSERT_EQ(issued_.size(), 1u);

  // RSI packets arrive; the KRC still never reports the order
  client_.ConfirmRsiStart();
  Cycles(kTimeout * (Client::kMaxRsiStartRetries + 2));

  EXPECT_EQ(issued_.size(), 1u);
  EXPECT_TRUE(client_.start_rsi_);
  EXPECT_TRUE(handler_->errors.empty());
}

TEST_F(MxaRsiStart, ConfirmationDoesNotCarryOverToTheNextStart)
{
  StartRsi();
  Cycles(5);
  client_.ConfirmRsiStart();
  Cycles(kTimeout);
  ASSERT_EQ(issued_.size(), 1u);
  client_.ResetRSI();

  StartRsi();
  Cycles(5);
  ASSERT_EQ(issued_.size(), 2u);
  Cycles(kTimeout);

  EXPECT_EQ(issued_.size(), 3u) << "the new start was not timed out";
  EXPECT_TRUE(handler_->errors.empty());
}

TEST_F(MxaRsiStart, AcknowledgedStartIsNotTimedOut)
{
  StartRsi();
  Cycles(5);
  ASSERT_EQ(issued_.size(), 1u);

  ReportOrderState(issued_[0], kOrderStateActive);
  Cycles(kTimeout + 50);

  EXPECT_EQ(issued_.size(), 1u);
  EXPECT_TRUE(client_.start_rsi_);
  EXPECT_EQ(handler_->sampling_count, 1);
  EXPECT_TRUE(handler_->errors.empty());
}

TEST_F(MxaRsiStart, GivesUpAfterSilentDrops)
{
  StartRsi();
  Cycles(kTimeout * (Client::kMaxRsiStartRetries + 1));

  EXPECT_EQ(issued_.size(), static_cast<size_t>(Client::kMaxRsiStartRetries + 1));
  EXPECT_FALSE(client_.start_rsi_);
  ASSERT_EQ(handler_->errors.size(), 1u);
  EXPECT_NE(handler_->errors[0].find("not acknowledged"), std::string::npos);
}

TEST_F(MxaRsiStart, GivesUpAfterTheRetryCap)
{
  StartRsi();
  for (int attempt = 0; attempt <= Client::kMaxRsiStartRetries; attempt++)
  {
    Cycles(kBackoff);
    ASSERT_EQ(issued_.size(), static_cast<size_t>(attempt + 1));
    ReportOrderState(issued_.back(), kOrderStateAborted);
  }
  Cycles(kBackoff);

  EXPECT_EQ(issued_.size(), static_cast<size_t>(Client::kMaxRsiStartRetries + 1));
  EXPECT_FALSE(client_.start_rsi_);
  ASSERT_EQ(handler_->errors.size(), 1u);
  EXPECT_NE(handler_->errors[0].find("aborted"), std::string::npos);
}

TEST_F(MxaRsiStart, DoneEndsTheStart)
{
  StartRsi();
  Cycles(5);
  ASSERT_EQ(issued_.size(), 1u);

  ReportOrderState(issued_[0], kOrderStateDone);
  Cycles(50);

  EXPECT_EQ(issued_.size(), 1u);
  EXPECT_FALSE(client_.start_rsi_);
  EXPECT_TRUE(handler_->errors.empty());
}

// Uses a real Client with the keep-alive thread, pointed at an address that
// never answers, so connected_ never becomes true
TEST(MxaClientDestruction, DestructorDoesNotHangWithTheLinkDown)
{
  // TEST-NET-1 (RFC 5737): routable syntax, guaranteed unreachable here
  auto client = std::make_unique<Client>("203.0.113.1", GetEphemeralUdpPort());
  ASSERT_EQ(client->Setup().return_code, kuka::external::control::ReturnCode::OK);
  client->StartRSI(ControlMode::JOINT_POSITION_CONTROL, CycleTime::RSI_4MS);

  const auto before = std::chrono::steady_clock::now();
  client.reset();
  const auto elapsed = std::chrono::steady_clock::now() - before;

  // Well under the old behavior (hangs forever); well over the destructor's
  // own 1 s cancel-wait bound plus one keep-alive receive timeout
  EXPECT_LT(elapsed, std::chrono::seconds(3));
}

TEST_F(MxaRsiStart, StopMessageIsReadFromTheAutomaticExternalState)
{
  auto & group = KRC_AXISGROUPREFARR[kAxisGroup];
  auto & wrapper = client_.mxa_wrapper_;

  group.AUTEXTSTATE.STOPMESS = true;
  wrapper.mxa_aut_ext_.OnCycle();
  EXPECT_TRUE(wrapper.stopMessageActive());

  group.AUTEXTSTATE.STOPMESS = false;
  wrapper.mxa_aut_ext_.OnCycle();
  EXPECT_FALSE(wrapper.stopMessageActive());
}

TEST_F(MxaRsiStart, SentMoveEnableIsTheTelegramBitNotTheInput)
{
  auto & group = KRC_AXISGROUPREFARR[kAxisGroup];
  auto & wrapper = client_.mxa_wrapper_;
  uint8_t telegram[256] = {};
  EXPECT_FALSE(wrapper.moveEnableSent()) << "reported before any telegram";

  wrapper.mxa_aut_ext_.OnCycle();
  wrapper.setmxAInput(telegram);
  ASSERT_TRUE(group.AUTEXTCONTROL.MOVE_ENABLE);
  EXPECT_TRUE(wrapper.moveEnableSent());
  EXPECT_EQ(telegram[1] & (1 << 3), 1 << 3);

  // A latched error keeps the bit off while the input stays on
  group.INTFBERRORID = 532;
  group.READDONE = true;
  wrapper.mxa_aut_ext_.OnCycle();
  wrapper.setmxAInput(telegram);
  ASSERT_TRUE(group.AUTEXTCONTROL.MOVE_ENABLE);
  EXPECT_FALSE(wrapper.moveEnableSent());
  EXPECT_EQ(telegram[1] & (1 << 3), 0);
}

// As on hardware: a dispatcher start stuck on a stop message, a cancel, a new start
TEST_F(MxaRsiStart, CancelDuringADispatcherStartAbandonsIt)
{
  auto & group = KRC_AXISGROUPREFARR[kAxisGroup];
  group.AUTEXTSTATE.PRO_ACT = false;
  group.AUTEXTSTATE.STOPMESS = true;
  StartRsi();
  Cycles(10);
  ASSERT_TRUE(client_.mxa_wrapper_.mxa_auto_start_.BUSY);
  ASSERT_FALSE(group.KRCCONTROL.RESET);

  client_.RequestRsiCancel();
  group.AUTEXTSTATE.STOPMESS = false;
  for (int i = 0; i < Client::kRsiCancelHoldCycles + 10 && !client_.IsRsiCancelled(); i++)
  {
    Cycle();
    ASSERT_FALSE(group.KRCCONTROL.RESET) << "dispatcher started during the cancel, cycle " << i;
  }
  ASSERT_TRUE(client_.IsRsiCancelled());
  EXPECT_FALSE(client_.mxa_wrapper_.mxa_auto_start_.EXECUTERESET);
  EXPECT_FALSE(client_.mxa_wrapper_.mxa_auto_start_.BUSY);
  ASSERT_FALSE(group.AUTEXTCONTROL.MOVE_ENABLE);

  StartRsi();
  for (int i = 0; i < 20 && !group.KRCCONTROL.RESET; i++)
  {
    Cycle();
    ASSERT_NE(group.INTFBERRORID, 532) << "KRC_AUTOSTART refused MOVE_ENABLE at cycle " << i;
  }
  EXPECT_TRUE(group.KRCCONTROL.RESET);
  EXPECT_TRUE(group.AUTEXTCONTROL.MOVE_ENABLE);
  EXPECT_FALSE(group.AUTEXTCONTROL.EXT_START);
}

TEST_F(MxaRsiStart, StoppedDispatcherIsStartedBeforeTheCommand)
{
  auto & group = KRC_AXISGROUPREFARR[kAxisGroup];
  group.AUTEXTSTATE.PRO_ACT = false;

  StartRsi();
  Cycles(10);
  EXPECT_TRUE(issued_.empty());
  EXPECT_TRUE(client_.start_cmd_dispatcher_);
  EXPECT_TRUE(group.KRCCONTROL.RESET);

  group.AUTEXTSTATE.PRO_ACT = true;
  Cycles(50);

  EXPECT_EQ(issued_.size(), 1u);
  EXPECT_FALSE(client_.start_cmd_dispatcher_);
}

}  // namespace
