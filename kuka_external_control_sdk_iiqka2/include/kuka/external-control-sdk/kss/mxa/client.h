// Copyright 2025 KUKA Hungaria Kft.
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

#ifndef KUKA__EXTERNAL_CONTROL_SDK__KSS__MXA__CLIENT_H_
#define KUKA__EXTERNAL_CONTROL_SDK__KSS__MXA__CLIENT_H_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "kuka/external-control-sdk/common/irobot.h"
#include "kuka/external-control-sdk/kss/configuration.h"
#include "kuka/external-control-sdk/kss/initialization_data.h"
#include "kuka/external-control-sdk/kss/mxa/mxa_wrapper.h"
#include "kuka/external-control-sdk/kss/status_update.h"
#include "kuka/external-control-sdk/utils/os-core-udp-communication/publisher.h"
#include "kuka/external-control-sdk/utils/os-core-udp-communication/subscriber.h"

namespace kuka::external::control::kss::mxa
{

// Class for communicating with the KRC via MXA
class Client
{
public:
  Client(const std::string & controller_ip, int client_port);

  ~Client();

  // Initiates MXA connection and starts keep-alive thread
  Status Setup();

  // Starts RSI program
  Status StartRSI(ControlMode control_mode, CycleTime cycle_time);

  // Cancels RSI program and calls ResetRSI, blocking until the cancel is
  // done. Blocks without a bound, so the mxA link must stay up; if the link
  // may drop, use RequestRsiCancel()/IsRsiCancelled() instead
  Status CancelRSI();

  // Starts the cancel on the keep-alive thread and returns at once. Poll
  // IsRsiCancelled before the next StartRSI
  Status RequestRsiCancel();

  // True once the requested cancel is done and the client is reset
  bool IsRsiCancelled() const;

  // True when a telegram has arrived that the keep-alive thread has not received yet
  bool IsTelegramPending() const;

  // Reset class to be able to start RSI again
  void ResetRSI();

  // Marks the start as taken. The first RSI packet is the only report of a
  // successful start, so this is what disarms the start timeout
  void ConfirmRsiStart();

  // Turn on drives
  void TurnOnDrives();

  // Turn off drives
  void TurnOffDrives();

  // Registers an event handler
  Status RegisterEventHandler(std::unique_ptr<EventHandler> && event_handler);

  Status RegisterEventHandlerExtension(std::unique_ptr<IEventHandlerExtension> && extension);

  Status RegisterStatusUpdateHandler(std::unique_ptr<IStatusUpdateHandler> && handler);

private:
  // Start keep-alive thread
  void StartKeepAliveThread();

  // One keep-alive cycle of the dispatcher start and the RSI start command
  void HandleRsiStart();

  // Count a failed start attempt: re-issue after the backoff, or report
  void RetryRsiStart(const std::string & reason);

  // One keep-alive cycle of a requested cancel
  void HandleRsiCancel();

  std::optional<int> CheckRosRuntimeVersion(mxAWrapper::VersionResult & version_result);

  // UDP communication
  std::unique_ptr<os::core::udp::communication::Publisher> udp_publisher_;
  std::unique_ptr<os::core::udp::communication::Subscriber> udp_subscriber_;
  static constexpr int kMXAServerPort =
    1336;  // Port on which the controller listens for MXA messages
  static constexpr int kMXAClientSendPort = 1335;  // Port from which the client sends MXA messages
  static constexpr int kInitTimeoutTicks = 4;
  static constexpr std::chrono::milliseconds kUDPTimeoutMs{100};

  // ROS runtime version compatibility check
  static constexpr int kRosRuntimeVersionMajor = 1;
  static constexpr int kRosRuntimeVersionMinor = 0;
  static constexpr int kRosRuntimeVersionRevision = 0;
  bool ros_runtime_version_checked_ = false;

  // Event handling
  std::mutex event_handler_mutex_;
  std::unique_ptr<EventHandler> event_handler_;
  bool event_handler_set_ = false;

  // Keep-alive thread
  std::thread keep_alive_thread_;
  std::atomic<bool> stop_requested_{false};

  // MXA communication
  //------------------------------

  // Byte arrays for the communication
  static constexpr int kMXABufferSize = 256;
  PLCIEC::BYTE read_buffer_[kMXABufferSize];
  PLCIEC::BYTE write_buffer_[kMXABufferSize];

  mxAWrapper mxa_wrapper_;

  std::atomic<bool> start_cmd_dispatcher_{false};
  std::atomic<bool> start_rsi_{false};
  std::atomic<int> rsi_start_retries_{0};
  static constexpr int kMaxRsiStartRetries = 3;
  // Both counts are keep-alive cycles, one per mxA status packet, about 4 ms
  // Wait after a failed attempt, so the re-issue clears the dispatcher restart
  static constexpr int kRsiStartBackoffCycles = 100;
  // A start neither reported nor confirmed by then counts as a dropped attempt
  static constexpr int kRsiStartAckTimeoutCycles = 500;
  std::atomic<int> rsi_start_hold_cycles_{0};
  std::atomic<int> rsi_start_pending_cycles_{0};
  std::atomic<bool> rsi_start_confirmed_{false};
  std::atomic<bool> rsi_started_{false};
  std::atomic<bool> rsi_started_notification_sent_{false};
  std::atomic<bool> connected_notification_sent_{false};

  // Let the RSI program end before the interpreter is stopped, about 50 ms
  static constexpr int kRsiCancelHoldCycles = 12;
  std::atomic<int> rsi_cancel_hold_cycles_{0};
  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> cancelled_{false};
  std::mutex cancel_finished_mutex_;
  std::condition_variable cancel_finished_cv_;
  // Upper bound on the destructor's cancel wait, so a down mxA link cannot
  // block destruction forever
  static constexpr std::chrono::milliseconds kDestructorCancelTimeout{1000};

  void SetToCancelled();

  std::unique_ptr<IEventHandlerExtension> event_handler_extension_;

  std::unique_ptr<IStatusUpdateHandler> status_update_handler_;

  CycleTime cycle_time_ = CycleTime::UNSPECIFIED;
  ControlMode control_mode_ = ControlMode::UNSPECIFIED;

  int active_error_code_ = 0;

  static constexpr std::size_t kMxaTechFunctionParamSize = 41;
};

}  // namespace kuka::external::control::kss::mxa

#endif  // KUKA__EXTERNAL_CONTROL_SDK__KSS__MXA__CLIENT_H_
