// Copyright 2026 KUKA Deutschland GmbH
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

#include <gtest/gtest.h>

#include <functional>
#include <vector>

#include "kuka/external-control-sdk/kss/status_update.h"

using kuka::external::control::ControlMode;
using kuka::external::control::OperationMode;
using kuka::external::control::kss::CycleTime;
using kuka::external::control::kss::StatusUpdate;

namespace
{

StatusUpdate MakeDefaultStatusUpdate()
{
  StatusUpdate status_update;
  status_update.Reset();
  return status_update;
}

}  // namespace

TEST(StatusUpdateEquality, IdenticalValuesCompareEqual)
{
  StatusUpdate lhs = MakeDefaultStatusUpdate();
  StatusUpdate rhs = MakeDefaultStatusUpdate();

  EXPECT_TRUE(lhs == rhs);
  EXPECT_FALSE(lhs != rhs);
}

TEST(StatusUpdateEquality, DifferingOnlyInRobotStoppedCompareUnequal)
{
  StatusUpdate lhs = MakeDefaultStatusUpdate();
  StatusUpdate rhs = MakeDefaultStatusUpdate();
  rhs.robot_stopped_ = !lhs.robot_stopped_;

  EXPECT_FALSE(lhs == rhs);
  EXPECT_TRUE(lhs != rhs);
}

// Mutates one field per setter so a future slip in operator== shows up
// as that field's case failing.
TEST(StatusUpdateEquality, EachFieldAloneMakesValuesUnequal)
{
  std::vector<std::function<void(StatusUpdate &)>> setters = {
    [](StatusUpdate & s) { s.control_mode_ = ControlMode::JOINT_POSITION_CONTROL; },
    [](StatusUpdate & s) { s.cycle_time_ = CycleTime::RSI_4MS; },
    [](StatusUpdate & s) { s.drives_powered_ = true; },
    [](StatusUpdate & s) { s.emergency_stop_ = true; },
    [](StatusUpdate & s) { s.guard_stop_ = true; },
    [](StatusUpdate & s) { s.in_motion_ = true; },
    [](StatusUpdate & s) { s.motion_possible_ = true; },
    [](StatusUpdate & s) { s.operation_mode_ = OperationMode::T1; },
    [](StatusUpdate & s) { s.robot_stopped_ = true; },
    [](StatusUpdate & s) { s.brake_test_requested_ = true; },
    [](StatusUpdate & s) { s.brake_test_warning_ = true; },
    [](StatusUpdate & s) { s.stop_message_active_ = true; },
    [](StatusUpdate & s) { s.move_enable_sent_ = true; },
  };

  for (std::size_t i = 0; i < setters.size(); ++i)
  {
    StatusUpdate lhs = MakeDefaultStatusUpdate();
    StatusUpdate rhs = MakeDefaultStatusUpdate();
    setters[i](rhs);

    EXPECT_NE(lhs, rhs) << "field index " << i << " did not affect operator==";
  }
}
