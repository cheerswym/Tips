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

#include "onboard/utils/http_client.h"

#include "curlpp/Easy.hpp"
#include "curlpp/Exception.hpp"
#include "curlpp/Options.hpp"
#include "curlpp/cURLpp.hpp"
#include "glog/logging.h"

namespace qcraft {

using Json = nlohmann::json;

absl::Status HttpClient::Get(const std::string &url,
                             const std::list<std::string> &headers,
                             std::string *result, int32_t timeout,
                             bool ssl_verifyserver) {
  try {
    curlpp::options::Url http_url(url);
    curlpp::Easy request;
    request.setOpt(http_url);
    request.setOpt(new curlpp::options::CustomRequest{"GET"});
    std::ostringstream response;
    request.setOpt(new curlpp::options::WriteStream(&response));
    request.setOpt(new curlpp::options::NoSignal(true));
    if (timeout) {
      request.setOpt(new curlpp::options::Timeout(timeout));
    }
    if (!ssl_verifyserver) {
      request.setOpt(new curlpp::options::SslVerifyPeer(false));
      request.setOpt(new curlpp::options::SslVerifyHost(0));
    }
    curlpp::options::HttpHeader http_header(headers);
    request.setOpt(http_header);
    request.perform();

    if (result != nullptr) {
      *result = response.str();
    }
  } catch (curlpp::LogicError &e) {
    LOG(ERROR) << e.what();
    return absl::InternalError(e.what());
  } catch (curlpp::RuntimeError &e) {
    LOG(ERROR) << e.what();
    return absl::InternalError(e.what());
  }

  return absl::OkStatus();
}

absl::Status HttpClient::Get(const std::string &url,
                             const std::list<std::string> &headers,
                             Json *result, int32_t timeout,
                             bool ssl_verifyserver) {
  std::string response;
  const auto status = Get(url, headers, &response, timeout, ssl_verifyserver);
  if (status.ok()) {
    try {
      *result = Json::parse(response.begin(), response.end());
    } catch (...) {
      return absl::InternalError("Cannot parse response as json.");
    }
  }
  return status;
}

absl::Status HttpClient::Post(const std::string &url,
                              const std::list<std::string> &headers,
                              const std::string &body, std::string *result,
                              int32_t timeout, int32_t connection_timeout) {
  try {
    curlpp::Cleanup cleaner;
    curlpp::Easy request;
    std::ostringstream response;
    request.setOpt(new curlpp::options::WriteStream(&response));
    request.setOpt(new curlpp::options::CustomRequest{"POST"});
    request.setOpt(new curlpp::options::Url(url));
    request.setOpt(new curlpp::options::PostFields(body));
    request.setOpt(new curlpp::options::PostFieldSize(body.length()));
    request.setOpt(new curlpp::options::NoSignal(true));
    if (timeout) {
      request.setOpt(new curlpp::options::Timeout(timeout));
    }
    if (connection_timeout) {
      request.setOpt(new curlpp::options::ConnectTimeout(connection_timeout));
    }

    request.setOpt(new curlpp::options::HttpHeader(headers));

    // Perform request and get response string.
    request.perform();
    if (result != nullptr) {
      *result = response.str();
    }
  } catch (curlpp::LogicError &e) {
    LOG(ERROR) << e.what();
    return absl::InternalError(e.what());
  } catch (curlpp::RuntimeError &e) {
    LOG(ERROR) << e.what();
    return absl::InternalError(e.what());
  }

  return absl::OkStatus();
}

absl::Status HttpClient::Post(const std::string &url,
                              const std::list<std::string> &headers,
                              const std::string &body, nlohmann::json *result,
                              int32_t timeout, int32_t connection_timeout) {
  std::string response;
  const auto status =
      Post(url, headers, body, &response, timeout, connection_timeout);
  if (status.ok()) {
    try {
      *result = Json::parse(response.begin(), response.end());
    } catch (...) {
      return absl::InternalError("Cannot parse response as json.");
    }
  }
  return status;
}

absl::Status HttpClient::Post(const std::string &url,
                              const std::list<std::string> &headers,
                              const Json &json, std::string *result,
                              int32_t timeout, int32_t connection_timeout) {
  const std::string body = json.dump();
  return Post(url, headers, body, result, timeout, connection_timeout);
}

absl::Status HttpClient::Post(const std::string &url,
                              const std::list<std::string> &headers,
                              const Json &json, Json *result, int32_t timeout,
                              int32_t connection_timeout) {
  const std::string body = json.dump();
  return Post(url, headers, body, result, timeout, connection_timeout);
}

absl::Status HttpClient::Put(const std::string &url,
                             const std::list<std::string> &headers,
                             const Json &json, std::string *result,
                             int32_t timeout) {
  try {
    curlpp::Cleanup cleaner;
    curlpp::Easy request;
    std::ostringstream response;
    request.setOpt(new curlpp::options::WriteStream(&response));
    request.setOpt(new curlpp::options::CustomRequest{"PUT"});
    request.setOpt(new curlpp::options::Url(url));
    const std::string data = json.dump();
    request.setOpt(new curlpp::options::PostFields(data));
    request.setOpt(new curlpp::options::PostFieldSize(data.length()));
    request.setOpt(new curlpp::options::NoSignal(true));
    if (timeout) {
      request.setOpt(new curlpp::options::Timeout(timeout));
    }

    request.setOpt(new curlpp::options::HttpHeader(headers));

    // Perform request and get response string.
    request.perform();
    if (result != nullptr) {
      *result = response.str();
    }
  } catch (curlpp::LogicError &e) {
    LOG(ERROR) << e.what();
    return absl::InternalError(e.what());
  } catch (curlpp::RuntimeError &e) {
    LOG(ERROR) << e.what();
    return absl::InternalError(e.what());
  }

  return absl::OkStatus();
}

absl::Status HttpClient::Put(const std::string &url,
                             const std::list<std::string> &headers,
                             const Json &json, nlohmann::json *result,
                             int32_t timeout) {
  std::string response;
  const auto status = Put(url, headers, json, &response, timeout);
  if (status.ok()) {
    try {
      *result = Json::parse(response.begin(), response.end());
    } catch (...) {
      return absl::InternalError("Cannot parse response as json.");
    }
  }
  return status;
}

absl::Status HttpClient::Delete(const std::string &url,
                                const std::list<std::string> &headers,
                                std::string *result, int32_t timeout) {
  try {
    curlpp::Cleanup cleaner;
    curlpp::Easy request;
    std::ostringstream response;

    request.setOpt(new curlpp::options::Url(url));
    request.setOpt(new curlpp::options::CustomRequest{"DELETE"});
    request.setOpt(new curlpp::options::WriteStream(&response));
    request.setOpt(new curlpp::options::NoSignal(true));
    if (timeout) {
      request.setOpt(new curlpp::options::Timeout(timeout));
    }

    request.setOpt(new curlpp::options::HttpHeader(headers));

    // Perform request and get response string.
    request.perform();
    if (result != nullptr) {
      *result = response.str();
    }
  } catch (curlpp::LogicError &e) {
    LOG(ERROR) << e.what();
    return absl::InternalError(e.what());
  } catch (curlpp::RuntimeError &e) {
    LOG(ERROR) << e.what();
    return absl::InternalError(e.what());
  }

  return absl::OkStatus();
}

absl::Status HttpClient::Delete(const std::string &url,
                                const std::list<std::string> &headers,
                                Json *result, int32_t timeout) {
  std::string response;
  const auto status = Delete(url, headers, &response, timeout);
  if (status.ok()) {
    try {
      *result = Json::parse(response.begin(), response.end());
    } catch (...) {
      return absl::InternalError(
          absl::StrCat("Cannot parse response as json: ", response));
    }
  }
  return status;
}

}  // namespace qcraft
