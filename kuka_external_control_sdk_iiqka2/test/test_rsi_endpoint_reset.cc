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
#include <string>

#include "kuka/external-control-sdk/kss/rsi/endpoint.h"

namespace
{

constexpr std::chrono::milliseconds kReceiveTimeout{50};

int GetEphemeralUdpPort()
{
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  addr.sin_port = 0;
  bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
  socklen_t len = sizeof(addr);
  getsockname(fd, reinterpret_cast<sockaddr *>(&addr), &len);
  int port = ntohs(addr.sin_port);
  close(fd);
  return port;
}

class RsiEndpointReset : public ::testing::Test
{
protected:
  void SetUp() override
  {
    port_ = GetEphemeralUdpPort();
    ASSERT_TRUE(endpoint_.Setup("127.0.0.1", port_));
  }

  bool Receive() { return endpoint_.ReceiveOrTimeout(kReceiveTimeout); }

  // As the KRC does while nobody answers: the packet waits in the socket
  void QueuePacket()
  {
    const int fd = socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons(port_);
    const std::string packet = "<Rob TYPE=\"KUKA\"><IPOC>1</IPOC></Rob>";
    sendto(fd, packet.data(), packet.size(), 0, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
    close(fd);
  }

  int port_ = 0;
  kuka::external::control::kss::rsi::Endpoint endpoint_;
};

}  // namespace

TEST_F(RsiEndpointReset, QueuedPacketIsReceivedWithoutAReset)
{
  QueuePacket();
  EXPECT_TRUE(Receive());
}

TEST_F(RsiEndpointReset, ResetDropsAQueuedPacket)
{
  QueuePacket();
  QueuePacket();
  EXPECT_TRUE(endpoint_.ResetNow());
  EXPECT_FALSE(Receive());
}

// The stop packet's reset waits for 40 ms of silence, which a streaming controller never gives
TEST_F(RsiEndpointReset, ResetDoesNotWaitForSilence)
{
  QueuePacket();
  const auto start = std::chrono::steady_clock::now();
  endpoint_.ResetNow();
  const auto took = std::chrono::steady_clock::now() - start;
  EXPECT_LT(took, std::chrono::milliseconds(20));
}

TEST_F(RsiEndpointReset, ResetDropsAnUnansweredPacket)
{
  QueuePacket();
  ASSERT_TRUE(Receive());
  QueuePacket();
  // An unanswered packet blocks the next receive until it is answered or reset
  EXPECT_FALSE(Receive());

  endpoint_.ResetNow();
  QueuePacket();
  EXPECT_TRUE(Receive());
}
