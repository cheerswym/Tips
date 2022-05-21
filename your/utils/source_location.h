// Copyright 2019 The MediaPipe Authors.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// NOTE: This file is copied from MediaPipe project and modified by qcraft.ai
// for its own use.

#ifndef ONBOARD_UTILS_SOURCE_LOCATION_H_
#define ONBOARD_UTILS_SOURCE_LOCATION_H_

#include <cstdint>
#include <string>

#include "absl/strings/str_cat.h"

namespace qcraft {

// Class representing a specific location in the source code of a program.
// SourceLocation is copyable.
class SourceLocation {
 public:
  // Avoid this constructor; it populates the object with dummy values.
  constexpr SourceLocation() : line_(0), file_name_(nullptr) {}

  // Wrapper to invoke the private constructor below. This should only be
  // used by the QCRAFT_LOC macro, hence the name.
  static constexpr SourceLocation DoNotInvokeDirectly(std::uint_least32_t line,
                                                      const char* file_name) {
    return SourceLocation(line, file_name);
  }

  // The line number of the captured source location.
  constexpr std::uint_least32_t line() const { return line_; }

  // The file name of the captured source location.
  constexpr const char* file_name() const { return file_name_; }

  std::string ToString() const { return absl::StrCat(file_name_, ":", line_); }

  // column() and function_name() are omitted because we don't have a
  // way to support them.

 private:
  // Do not invoke this constructor directly. Instead, use the
  // QCRAFT_LOC macro below.
  //
  // file_name must outlive all copies of the SourceLocation object, so in
  // practice it should be a std::string literal.
  constexpr SourceLocation(std::uint_least32_t line, const char* file_name)
      : line_(line), file_name_(file_name) {}

  std::uint_least32_t line_;
  const char* file_name_;
};

}  // namespace qcraft

// If a function takes a SourceLocation parameter, pass this as the argument.
#define QCRAFT_LOC                                              \
  qcraft::SourceLocation::DoNotInvokeDirectly(__builtin_LINE(), \
                                              __builtin_FILE())

#endif  // ONBOARD_UTILS_SOURCE_LOCATION_H_
