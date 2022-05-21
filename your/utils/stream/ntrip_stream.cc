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

#include "onboard/utils/stream/ntrip_stream.h"

#include <unistd.h>

#include <iostream>
#include <mutex>

#include "absl/time/clock.h"
#include "onboard/ap_common/util/string_util.h"
#include "onboard/lite/logging.h"
#include "onboard/utils/stream/tcp_stream.h"
#include "onboard/utils/time_util.h"

namespace {

template <typename T>
constexpr bool is_zero(T value) {
  return value == static_cast<T>(0);
}
}  // namespace

namespace qcraft {

NtripStream::NtripStream(const std::string& address, uint16_t port,
                         const std::string& mountpoint, const std::string& user,
                         const std::string& passwd, uint32_t timeout_s)
    : mountpoint_(mountpoint),
      write_data_prefix_("GET /" + mountpoint +
                         " HTTP/1.0\r\n"
                         "User-Agent: NTRIP gnss_driver/0.0\r\n"
                         "accept: */* \r\n\r\n"),

      login_data_("GET /" + mountpoint +
                  " HTTP/1.0\r\n"
                  "User-Agent: NTRIP gnss_driver/0.0\r\n"
                  "accept: */* \r\n"
                  "Authorization: Basic " +
                  apollo::common::util::EncodeBase64(user + ":" + passwd) +
                  "\r\n\r\n"),
      timeout_s_(timeout_s),
      tcp_stream_(new TcpStream(address.c_str(), port, 0, false)) {}

NtripStream::~NtripStream() { this->Disconnect(); }

bool NtripStream::Connect() {
  if (is_login_) {
    return true;
  }
  if (!tcp_stream_) {
    QLOG(ERROR) << "New tcp stream failed.";
    return true;
  }

  if (!tcp_stream_->Connect()) {
    status_ = Stream::Status::DISCONNECTED;
    QLOG(ERROR) << "Tcp connect failed.";
    return false;
  }

  uint8_t buffer[2048];
  size_t size = 0;
  size_t try_times = 0;

  size = tcp_stream_->write(
      reinterpret_cast<const uint8_t*>(login_data_.data()), login_data_.size());
  if (size != login_data_.size()) {
    tcp_stream_->Disconnect();
    status_ = Stream::Status::ERROR;
    QLOG(ERROR) << "Send ntrip request failed.";
    return false;
  }

  bzero(buffer, sizeof(buffer));
  QLOG(INFO) << "Read ntrip response.";
  size = tcp_stream_->read(buffer, sizeof(buffer) - 1);
  while ((size == 0) && (try_times < 3)) {
    sleep(1);
    size = tcp_stream_->read(buffer, sizeof(buffer) - 1);
    ++try_times;
  }

  if (!size) {
    tcp_stream_->Disconnect();
    status_ = Stream::Status::DISCONNECTED;
    QLOG(ERROR) << "No response from ntripcaster.";
    return false;
  }

  if (std::strstr(reinterpret_cast<char*>(buffer), "ICY 200 OK\r\n")) {
    status_ = Stream::Status::CONNECTED;
    is_login_ = true;
    QLOG(INFO) << "Ntrip login successfully.";
    return true;
  }

  if (std::strstr(reinterpret_cast<char*>(buffer), "SOURCETABLE 200 OK\r\n")) {
    QLOG(ERROR) << "Mountpoint " << mountpoint_ << " not exist.";
  }

  if (std::strstr(reinterpret_cast<char*>(buffer), "HTTP/")) {
    QLOG(ERROR) << "Authentication failed.";
  }

  QLOG(INFO) << "No expect data.";
  QLOG(INFO) << "Recv data length: " << size;
  // QLOG(INFO) << "Data from server: " << reinterpret_cast<char*>(buffer);

  tcp_stream_->Disconnect();
  status_ = Stream::Status::ERROR;
  return false;
}

bool NtripStream::Disconnect() {
  if (is_login_) {
    bool ret = tcp_stream_->Disconnect();
    if (!ret) {
      return false;
    }
    status_ = Stream::Status::DISCONNECTED;
    is_login_ = false;
  }

  return true;
}

void NtripStream::Reconnect() {
  QLOG(INFO) << "Reconnect ntrip caster.";
  std::unique_lock<std::mutex> lock(internal_mutex_);
  Disconnect();
  Connect();
  if (status_ != Stream::Status::CONNECTED) {
    QLOG(INFO) << "Reconnect ntrip caster failed.";
    return;
  }

  data_active_s_ = ToUnixDoubleSeconds(absl::Now());
  QLOG(INFO) << "Reconnect ntrip caster success.";
}

size_t NtripStream::read(uint8_t* buffer, size_t max_length) {
  if (!tcp_stream_) {
    return 0;
  }

  size_t ret = 0;

  if (tcp_stream_->get_status() != Stream::Status::CONNECTED) {
    Reconnect();
    if (status_ != Stream::Status::CONNECTED) {
      return 0;
    }
  }

  if (is_zero(data_active_s_)) {
    data_active_s_ = ToUnixDoubleSeconds(absl::Now());
  }

  ret = tcp_stream_->read(buffer, max_length);
  if (ret) {
    data_active_s_ = ToUnixDoubleSeconds(absl::Now());
  }

  // timeout detect
  if ((ToUnixDoubleSeconds(absl::Now()) - data_active_s_) > timeout_s_) {
    QLOG(INFO) << "Ntrip timeout.";
    Reconnect();
  }

  return ret;
}

size_t NtripStream::write(const uint8_t* buffer, size_t length) {
  if (!tcp_stream_) {
    return 0;
  }
  std::unique_lock<std::mutex> lock(internal_mutex_, std::defer_lock);
  if (!lock.try_lock()) {
    QLOG(INFO) << "Try lock failed.";
    return 0;
  }

  if (tcp_stream_->get_status() != Stream::Status::CONNECTED) {
    return 0;
  }

  std::string data(reinterpret_cast<const char*>(buffer), length);
  data = write_data_prefix_ + data;
  size_t ret = tcp_stream_->write(reinterpret_cast<const uint8_t*>(data.data()),
                                  data.size());
  if (ret != data.size()) {
    QLOG(ERROR) << "Send ntrip data size " << data.size() << ", return " << ret;
    status_ = Stream::Status::ERROR;
    return 0;
  }

  return length;
}

}  // namespace qcraft
