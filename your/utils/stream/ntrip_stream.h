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

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>

#include "onboard/utils/stream/stream.h"
#include "onboard/utils/stream/tcp_stream.h"

namespace qcraft {

// This class is not required when using Novatel PwrPak7D, because it has built
// in support for Ntrip
class NtripStream : public Stream {
 public:
  NtripStream(const std::string& address, uint16_t port,
              const std::string& mountpoint, const std::string& user,
              const std::string& passwd, uint32_t timeout_s);
  virtual ~NtripStream();

  virtual size_t read(uint8_t* buffer, size_t max_length);
  virtual size_t write(const uint8_t* data, size_t length);
  virtual bool Connect();
  virtual bool Disconnect();

 private:
  void Reconnect();
  bool is_login_ = false;
  const std::string mountpoint_;
  const std::string write_data_prefix_;
  const std::string login_data_;
  double timeout_s_ = 60.0;
  double data_active_s_ = 0.0;
  std::unique_ptr<TcpStream> tcp_stream_;
  std::mutex internal_mutex_;
};

}  // namespace qcraft
