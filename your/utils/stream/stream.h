#ifndef ONBOARD_UTILS_STREAM_STREAM_H_
#define ONBOARD_UTILS_STREAM_STREAM_H_

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

// This defines an stream interface for communication via USB, Ethernet, etc.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "onboard/base/macros.h"
#include "onboard/global/singleton.h"

namespace qcraft {

// An abstract class of Stream.
// One should use the create_xxx() functions to create a Stream object.
class Stream {
 public:
  virtual ~Stream() {}

  // Stream status.
  enum class Status {
    DISCONNECTED,
    CONNECTED,
    ERROR,
  };

  static constexpr size_t NUM_STATUS =
      static_cast<int>(Stream::Status::ERROR) + 1;
  Status get_status() const { return status_; }

  // Returns whether it was successful to connect.
  virtual bool Connect() = 0;

  // Returns whether it was successful to disconnect.
  virtual bool Disconnect() = 0;

  // Reads up to max_length bytes. Returns actually number of bytes read.
  virtual size_t read(uint8_t *buffer, size_t max_length) = 0;

  // Returns how many bytes it was successful to write.
  virtual size_t write(const uint8_t *buffer, size_t length) = 0;

  size_t write(const std::string &buffer) {
    return write(reinterpret_cast<const uint8_t *>(buffer.data()),
                 buffer.size());
  }

 protected:
  Stream() {}

  Status status_ = Status::DISCONNECTED;

 private:
  DISALLOW_COPY_AND_ASSIGN(Stream);
};

}  // namespace qcraft

#endif  // ONBOARD_UTILS_STREAM_STREAM_H_
