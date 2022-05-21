/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include <cstdint>

#include "onboard/utils/stream/stream.h"

namespace qcraft {

class UdpStream : public Stream {
  typedef uint16_t be16_t;
  typedef uint32_t be32_t;

 public:
  // if src_port provided, will try to bind to src_port, instead of from
  // arbitrary port.
  UdpStream(const char* address, uint16_t port, uint32_t timeout_usec,
            uint16_t src_port = 0);
  ~UdpStream();

  virtual bool Connect();
  virtual bool Disconnect();
  virtual size_t read(uint8_t* buffer, size_t max_length);
  virtual size_t write(const uint8_t* data, size_t length);

 private:
  UdpStream() = default;
  void open();
  void close();
  be16_t peer_port_ = 0;
  be32_t peer_addr_ = 0;
  be16_t src_port_;
  uint32_t timeout_usec_ = 0;
  int sockfd_ = -1;
  int errno_ = 0;
};

}  // namespace qcraft
