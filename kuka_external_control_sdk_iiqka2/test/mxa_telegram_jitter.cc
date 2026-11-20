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

// Measures how evenly the keep-alive thread receives mxA telegrams. A sender
// thread feeds the client over loopback at a fixed period; the tool prints
// p50/p99/max of the receive inter-arrival time and of the send-to-receive
// latency, which leaves out the sender's own jitter.
//
//   mxa_telegram_jitter [--count N] [--period-us US] [--cpu C] [--fifo PRIO] [--load K]
//
// --cpu pins the keep-alive thread, and --load K busy threads, to core C;
// --fifo gives the keep-alive thread SCHED_FIFO at PRIO. The sender runs on
// another core, at PRIO + 1 when --fifo is given.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <thread>
#include <vector>

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

int64_t SteadyNanoseconds()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

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

class StampingHandler : public IStatusUpdateHandler
{
public:
  explicit StampingHandler(size_t capacity) : stamps(capacity, 0) {}

  void OnStatusUpdateReceived(const StatusUpdate &) override {}
  void OnTelegramReceived() override
  {
    const size_t i = received.load(std::memory_order_relaxed);
    if (i < stamps.size())
    {
      stamps[i] = SteadyNanoseconds();
      received.store(i + 1, std::memory_order_release);
    }
  }

  std::vector<int64_t> stamps;
  std::atomic<size_t> received{0};
};

bool Pin(pthread_t thread, int cpu)
{
  cpu_set_t cpus;
  CPU_ZERO(&cpus);
  CPU_SET(cpu, &cpus);
  const int err = pthread_setaffinity_np(thread, sizeof(cpus), &cpus);
  if (err != 0)
  {
    std::fprintf(stderr, "affinity to core %d not set: %s\n", cpu, std::strerror(err));
  }
  return err == 0;
}

bool Fifo(pthread_t thread, int priority)
{
  sched_param param{};
  param.sched_priority = priority;
  const int err = pthread_setschedparam(thread, SCHED_FIFO, &param);
  if (err != 0)
  {
    std::fprintf(
      stderr, "SCHED_FIFO %d not set: %s, keeping the default\n", priority, std::strerror(err));
  }
  return err == 0;
}

void Report(const char * name, std::vector<int64_t> values_ns)
{
  if (values_ns.empty())
  {
    std::printf("%-12s no samples\n", name);
    return;
  }
  std::sort(values_ns.begin(), values_ns.end());
  const auto at = [&values_ns](double q)
  { return values_ns[static_cast<size_t>(q * static_cast<double>(values_ns.size() - 1))] / 1e3; };
  std::printf(
    "%-12s p50 %8.1f us  p99 %8.1f us  max %8.1f us  (n=%zu)\n", name, at(0.5), at(0.99),
    values_ns.back() / 1e3, values_ns.size());
}

}  // namespace

int main(int argc, char ** argv)
{
  int count = 2500;
  int period_us = 4000;
  int cpu = -1;
  int fifo = 0;
  int load = 0;
  for (int i = 1; i + 1 < argc; i += 2)
  {
    const std::string flag = argv[i];
    const int value = std::atoi(argv[i + 1]);
    if (flag == "--count")
    {
      count = value;
    }
    else if (flag == "--period-us")
    {
      period_us = value;
    }
    else if (flag == "--cpu")
    {
      cpu = value;
    }
    else if (flag == "--fifo")
    {
      fifo = value;
    }
    else if (flag == "--load")
    {
      load = value;
    }
    else
    {
      std::fprintf(stderr, "unknown flag %s\n", flag.c_str());
      return 2;
    }
  }

  const int port = GetEphemeralUdpPort();
  Client client("127.0.0.1", port);
  auto handler = std::make_unique<StampingHandler>(static_cast<size_t>(count));
  auto * stamps = handler.get();
  client.RegisterStatusUpdateHandler(std::move(handler));
  if (client.Setup().return_code != ReturnCode::OK)
  {
    std::fprintf(stderr, "client setup failed\n");
    return 1;
  }
  const auto handle = client.KeepAliveThreadNativeHandle();
  if (!handle)
  {
    std::fprintf(stderr, "no keep-alive thread\n");
    return 1;
  }

  bool pinned = false;
  bool scheduled = false;
  if (cpu >= 0)
  {
    pinned = Pin(*handle, cpu);
  }
  if (fifo > 0)
  {
    scheduled = Fifo(*handle, fifo);
  }

  std::atomic<bool> stop{false};
  std::vector<std::thread> busy;
  for (int i = 0; i < load && cpu >= 0; i++)
  {
    busy.emplace_back(
      [&stop]
      {
        while (!stop.load(std::memory_order_relaxed))
        {
        }
      });
    Pin(busy.back().native_handle(), cpu);
  }

  Publisher sender(SocketAddress("127.0.0.1", port), SocketAddress(), false);
  if (sender.Setup() != Publisher::ErrorCode::kSuccess)
  {
    std::fprintf(stderr, "sender setup failed\n");
    return 1;
  }
  std::vector<int64_t> sent(static_cast<size_t>(count), 0);
  std::thread send_thread(
    [&]
    {
      const uint8_t telegram[256] = {};
      timespec next{};
      clock_gettime(CLOCK_MONOTONIC, &next);
      for (int i = 0; i < count; i++)
      {
        next.tv_nsec += static_cast<int64_t>(period_us) * 1000;
        while (next.tv_nsec >= 1000000000)
        {
          next.tv_nsec -= 1000000000;
          next.tv_sec++;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, nullptr);
        sent[static_cast<size_t>(i)] = SteadyNanoseconds();
        sender.Send(telegram, sizeof(telegram));
      }
    });
  if (cpu >= 0)
  {
    Pin(send_thread.native_handle(), cpu == 0 ? 1 : 0);
  }
  if (fifo > 0)
  {
    Fifo(send_thread.native_handle(), fifo + 1);
  }
  send_thread.join();

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (stamps->received.load(std::memory_order_acquire) < static_cast<size_t>(count) &&
         std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  stop = true;
  for (auto & t : busy)
  {
    t.join();
  }

  const size_t n = stamps->received.load(std::memory_order_acquire);
  std::vector<int64_t> inter_arrival;
  std::vector<int64_t> latency;
  // The first telegrams race the thread's start, so they are left out
  constexpr size_t kWarmup = 10;
  for (size_t i = kWarmup; i < n; i++)
  {
    inter_arrival.push_back(stamps->stamps[i] - stamps->stamps[i - 1]);
    latency.push_back(stamps->stamps[i] - sent[i]);
  }

  std::printf(
    "period %d us, %zu of %d telegrams received, keep-alive thread: core %s, %s, %d busy threads\n",
    period_us, n, count, pinned ? std::to_string(cpu).c_str() : "any",
    scheduled ? ("SCHED_FIFO " + std::to_string(fifo)).c_str() : "default scheduling",
    static_cast<int>(busy.size()));
  Report("inter-arrival", inter_arrival);
  Report("latency", latency);
  return 0;
}
