#ifndef ONBOARD_UTILS_HTTP_CLIENT_H_
#define ONBOARD_UTILS_HTTP_CLIENT_H_

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

#include <list>
#include <string>

#include "absl/status/status.h"
#include "nlohmann/json.hpp"

namespace qcraft {

class HttpClient {
 public:
  /**
   * @brief Get from a target url, get response as string.
   */
  static absl::Status Get(const std::string &url,
                          const std::list<std::string> &headers,
                          std::string *result = nullptr, int32_t timeout = 0,
                          bool ssl_verifyserver = true);

  /**
   * @brief Get from a target url, get response as json.
   */
  static absl::Status Get(const std::string &url,
                          const std::list<std::string> &headers,
                          nlohmann::json *result = nullptr, int32_t timeout = 0,
                          bool ssl_verifyserver = true);

  /**
   * @brief post a string to target url, post response as string.
   */
  static absl::Status Post(const std::string &url,
                           const std::list<std::string> &headers,
                           const std::string &body,
                           std::string *result = nullptr, int32_t timeout = 0,
                           int32_t connection_timeout = 0);

  /**
   * @brief post a string to target url, post response as json.
   */
  static absl::Status Post(const std::string &url,
                           const std::list<std::string> &headers,
                           const std::string &body, nlohmann::json *result,
                           int32_t timeout = 0, int32_t connection_timeout = 0);

  /**
   * @brief post a json to target url, post response as string.
   */
  static absl::Status Post(const std::string &url,
                           const std::list<std::string> &headers,
                           const nlohmann::json &json,
                           std::string *result = nullptr, int32_t timeout = 0,
                           int32_t connection_timeout = 0);

  /**
   * @brief post a json to target url, post response as json.
   */
  static absl::Status Post(const std::string &url,
                           const std::list<std::string> &headers,
                           const nlohmann::json &json, nlohmann::json *result,
                           int32_t timeout = 0, int32_t connection_timeout = 0);

  /**
   * @brief put a json to target url, put response as string.
   */
  static absl::Status Put(const std::string &url,
                          const std::list<std::string> &headers,
                          const nlohmann::json &json,
                          std::string *result = nullptr, int32_t timeout = 0);

  /**
   * @brief put a json to target url, put response as json.
   */
  static absl::Status Put(const std::string &url,
                          const std::list<std::string> &headers,
                          const nlohmann::json &json, nlohmann::json *result,
                          int32_t timeout = 0);

  /**
   * @brief Delete from a target url, delete response as string.
   */
  static absl::Status Delete(const std::string &url,
                             const std::list<std::string> &headers,
                             std::string *result = nullptr,
                             int32_t timeout = 0);
  /**
   * @brief Delete from a target url, delete response as json.
   */
  static absl::Status Delete(const std::string &url,
                             const std::list<std::string> &headers,
                             nlohmann::json *result, int32_t timeout = 0);
};

}  // namespace qcraft

#endif  // ONBOARD_UTILS_HTTP_CLIENT_H_
