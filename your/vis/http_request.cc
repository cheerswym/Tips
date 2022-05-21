#include "onboard/vis/http_request.h"

namespace qcraft::http {

namespace {

class RequestError final : public std::logic_error {
 public:
  explicit RequestError(const char* str) : std::logic_error(str) {}
  explicit RequestError(const std::string& str) : std::logic_error(str) {}
};

class ResponseError final : public std::runtime_error {
 public:
  explicit ResponseError(const char* str) : std::runtime_error(str) {}
  explicit ResponseError(const std::string& str) : std::runtime_error(str) {}
};

inline int getLastError() noexcept { return errno; }

constexpr int getAddressFamily(InternetProtocol internetProtocol) {
  return (internetProtocol == InternetProtocol::V4) ? AF_INET
         : (internetProtocol == InternetProtocol::V6)
             ? AF_INET6
             : throw RequestError("Unsupported protocol");
}

constexpr auto closeSocket = close;

constexpr int noSignal = MSG_NOSIGNAL;

class Socket final {
 public:
  using Type = int;
  static constexpr Type invalid = -1;

  explicit Socket(InternetProtocol internetProtocol)
      : endpoint(socket(getAddressFamily(internetProtocol), SOCK_STREAM,
                        IPPROTO_TCP)) {
    if (endpoint == invalid)
      throw std::system_error(getLastError(), std::system_category(),
                              "Failed to create socket");
  }

  ~Socket() {
    if (endpoint != invalid) closeSocket(endpoint);
  }

  Socket(Socket&& other) noexcept : endpoint(other.endpoint) {
    other.endpoint = invalid;
  }

  Socket& operator=(Socket&& other) noexcept {
    if (&other == this) return *this;
    if (endpoint != invalid) closeSocket(endpoint);
    endpoint = other.endpoint;
    other.endpoint = invalid;
    return *this;
  }

  void connect(const struct sockaddr* address, socklen_t addressSize);

  size_t send(const void* buffer, size_t length, int flags);

  size_t recv(void* buffer, size_t length, int flags);

  operator Type() const noexcept { return endpoint; }

 private:
  Type endpoint = invalid;
};

void Socket::connect(const struct sockaddr* address, socklen_t addressSize) {
  auto result = ::connect(endpoint, address, addressSize);

  while (result == -1 && errno == EINTR)
    result = ::connect(endpoint, address, addressSize);

  if (result == -1)
    throw std::system_error(getLastError(), std::system_category(),
                            "Failed to connect");
}

size_t Socket::send(const void* buffer, size_t length, int flags) {
  auto result =
      ::send(endpoint, reinterpret_cast<const char*>(buffer), length, flags);

  while (result == -1 && errno == EINTR)
    result =
        ::send(endpoint, reinterpret_cast<const char*>(buffer), length, flags);
  if (result == -1)
    throw std::system_error(getLastError(), std::system_category(),
                            "Failed to send data");

  return static_cast<size_t>(result);
}

size_t Socket::recv(void* buffer, size_t length, int flags) {
  auto result =
      ::recv(endpoint, reinterpret_cast<char*>(buffer), length, flags);

  while (result == -1 && errno == EINTR)
    result = ::recv(endpoint, reinterpret_cast<char*>(buffer), length, flags);
  if (result == -1)
    throw std::system_error(getLastError(), std::system_category(),
                            "Failed to read data");

  return static_cast<size_t>(result);
}

}  // namespace

std::string urlEncode(const std::string& str) {
  constexpr char hexChars[16] = {'0', '1', '2', '3', '4', '5', '6', '7',
                                 '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

  std::string result;

  for (auto i = str.begin(); i != str.end(); ++i) {
    const std::uint8_t cp = *i & 0xFF;

    if ((cp >= 0x30 && cp <= 0x39) ||           // 0-9
        (cp >= 0x41 && cp <= 0x5A) ||           // A-Z
        (cp >= 0x61 && cp <= 0x7A) ||           // a-z
        cp == 0x2D || cp == 0x2E || cp == 0x5F  // - . _
    ) {
      result += static_cast<char>(cp);
    } else if (cp <= 0x7F) {
      result +=
          std::string("%") + hexChars[(*i & 0xF0) >> 4] + hexChars[*i & 0x0F];
    } else if ((cp >> 5) == 0x06) {
      result +=
          std::string("%") + hexChars[(*i & 0xF0) >> 4] + hexChars[*i & 0x0F];
      if (++i == str.end()) break;
      result +=
          std::string("%") + hexChars[(*i & 0xF0) >> 4] + hexChars[*i & 0x0F];
    } else if ((cp >> 4) == 0x0E) {
      result +=
          std::string("%") + hexChars[(*i & 0xF0) >> 4] + hexChars[*i & 0x0F];
      if (++i == str.end()) break;
      result +=
          std::string("%") + hexChars[(*i & 0xF0) >> 4] + hexChars[*i & 0x0F];
      if (++i == str.end()) break;
      result +=
          std::string("%") + hexChars[(*i & 0xF0) >> 4] + hexChars[*i & 0x0F];
    } else if ((cp >> 3) == 0x1E) {
      result +=
          std::string("%") + hexChars[(*i & 0xF0) >> 4] + hexChars[*i & 0x0F];
      if (++i == str.end()) break;
      result +=
          std::string("%") + hexChars[(*i & 0xF0) >> 4] + hexChars[*i & 0x0F];
      if (++i == str.end()) break;
      result +=
          std::string("%") + hexChars[(*i & 0xF0) >> 4] + hexChars[*i & 0x0F];
      if (++i == str.end()) break;
      result +=
          std::string("%") + hexChars[(*i & 0xF0) >> 4] + hexChars[*i & 0x0F];
    }
  }

  return result;
}

Request ::Request(const std::string& url, InternetProtocol protocol)
    : internetProtocol(protocol) {
  const auto schemeEndPosition = url.find("://");

  if (schemeEndPosition != std::string::npos) {
    scheme = url.substr(0, schemeEndPosition);
    path = url.substr(schemeEndPosition + 3);
  } else {
    scheme = "http";
    path = url;
  }

  const auto fragmentPosition = path.find('#');

  // remove the fragment part
  if (fragmentPosition != std::string::npos) path.resize(fragmentPosition);

  const auto pathPosition = path.find('/');

  if (pathPosition == std::string::npos) {
    domain = path;
    path = "/";
  } else {
    domain = path.substr(0, pathPosition);
    path = path.substr(pathPosition);
  }

  const auto portPosition = domain.find(':');

  if (portPosition != std::string::npos) {
    port = domain.substr(portPosition + 1);
    domain.resize(portPosition);
  } else {
    port = "80";
  }
}

Response Request::send(const std::string& method,
                       const std::map<std::string, std::string>& parameters,
                       const std::vector<std::string>& headers) {
  std::string body;
  bool first = true;

  for (const auto& parameter : parameters) {
    if (!first) body += "&";
    first = false;

    body += urlEncode(parameter.first) + "=" + urlEncode(parameter.second);
  }

  return send(method, body, headers);
}

Response Request::send(const std::string& method, const std::string& body,
                       const std::vector<std::string>& headers) {
  if (scheme != "http") throw RequestError("Only HTTP scheme is supported");

  addrinfo hints = {};
  hints.ai_family = getAddressFamily(internetProtocol);
  hints.ai_socktype = SOCK_STREAM;

  addrinfo* info;
  if (getaddrinfo(domain.c_str(), port.c_str(), &hints, &info) != 0)
    throw std::system_error(getLastError(), std::system_category(),
                            "Failed to get address info of " + domain);

  std::unique_ptr<addrinfo, decltype(&freeaddrinfo)> addressInfo(info,
                                                                 freeaddrinfo);

  std::string headerData = method + " " + path + " HTTP/1.1\r\n";

  for (const std::string& header : headers) headerData += header + "\r\n";

  headerData += "Host: " + domain +
                "\r\n"
                "Content-Length: " +
                std::to_string(body.size()) +
                "\r\n"
                "\r\n";

  std::vector<uint8_t> requestData(headerData.begin(), headerData.end());
  requestData.insert(requestData.end(), body.begin(), body.end());

  Socket socket(internetProtocol);

  // take the first address from the list
  socket.connect(addressInfo->ai_addr,
                 static_cast<socklen_t>(addressInfo->ai_addrlen));

  auto remaining = requestData.size();
  auto sendData = requestData.data();

  // send the request
  while (remaining > 0) {
    const auto size = socket.send(sendData, remaining, noSignal);
    remaining -= size;
    sendData += size;
  }

  std::uint8_t tempBuffer[4096];
  constexpr std::uint8_t crlf[] = {'\r', '\n'};
  Response response;
  std::vector<std::uint8_t> responseData;
  bool firstLine = true;
  bool parsedHeaders = false;
  bool contentLengthReceived = false;
  uint64_t contentLength = 0;
  bool chunkedResponse = false;
  std::size_t expectedChunkSize = 0;
  bool removeCrlfAfterChunk = false;

  // read the response
  for (;;) {
    const auto size = socket.recv(tempBuffer, sizeof(tempBuffer), noSignal);

    if (size == 0) break;  // disconnected

    responseData.insert(responseData.end(), tempBuffer, tempBuffer + size);

    if (!parsedHeaders) {
      for (;;) {
        const auto i = std::search(responseData.begin(), responseData.end(),
                                   std::begin(crlf), std::end(crlf));

        // didn't find a newline
        if (i == responseData.end()) break;

        const std::string line(responseData.begin(), i);
        responseData.erase(responseData.begin(), i + 2);

        // empty line indicates the end of the header section
        if (line.empty()) {
          parsedHeaders = true;
          break;
        } else if (firstLine) {
          firstLine = false;

          std::string::size_type lastPos = 0;
          const auto length = line.length();
          std::vector<std::string> parts;

          // tokenize first line
          while (lastPos < length + 1) {
            auto pos = line.find(' ', lastPos);
            if (pos == std::string::npos) pos = length;

            if (pos != lastPos)
              parts.emplace_back(
                  line.data() + lastPos,
                  static_cast<std::vector<std::string>::size_type>(pos) -
                      lastPos);

            lastPos = pos + 1;
          }

          if (parts.size() >= 2) response.status = std::stoi(parts[1]);
        } else {
          response.headers.push_back(line);

          const auto pos = line.find(':');

          if (pos != std::string::npos) {
            std::string headerName = line.substr(0, pos);
            std::string headerValue = line.substr(pos + 1);

            // ltrim
            headerValue.erase(
                headerValue.begin(),
                std::find_if(headerValue.begin(), headerValue.end(),
                             [](int c) { return !std::isspace(c); }));

            // rtrim
            headerValue.erase(
                std::find_if(headerValue.rbegin(), headerValue.rend(),
                             [](int c) { return !std::isspace(c); })
                    .base(),
                headerValue.end());

            if (headerName == "Content-Length") {
              contentLength = std::stoul(headerValue);
              contentLengthReceived = true;
              response.body.reserve(contentLength);
            } else if (headerName == "Transfer-Encoding") {
              if (headerValue == "chunked")
                chunkedResponse = true;
              else
                throw ResponseError("Unsupported transfer encoding: " +
                                    headerValue);
            }
          }
        }
      }
    }

    if (parsedHeaders) {
      // Content-Length must be ignored if Transfer-Encoding is received
      if (chunkedResponse) {
        bool dataReceived = false;
        for (;;) {
          if (expectedChunkSize > 0) {
            const auto toWrite =
                std::min(expectedChunkSize, responseData.size());
            response.body.insert(
                response.body.end(), responseData.begin(),
                responseData.begin() + static_cast<ptrdiff_t>(toWrite));
            responseData.erase(
                responseData.begin(),
                responseData.begin() + static_cast<ptrdiff_t>(toWrite));
            expectedChunkSize -= toWrite;

            if (expectedChunkSize == 0) removeCrlfAfterChunk = true;
            if (responseData.empty()) break;
          } else {
            if (removeCrlfAfterChunk) {
              if (responseData.size() >= 2) {
                removeCrlfAfterChunk = false;
                responseData.erase(responseData.begin(),
                                   responseData.begin() + 2);
              } else {
                break;
              }
            }

            const auto i = std::search(responseData.begin(), responseData.end(),
                                       std::begin(crlf), std::end(crlf));

            if (i == responseData.end()) break;

            const std::string line(responseData.begin(), i);
            responseData.erase(responseData.begin(), i + 2);

            expectedChunkSize = std::stoul(line, nullptr, 16);

            if (expectedChunkSize == 0) {
              dataReceived = true;
              break;
            }
          }
        }

        if (dataReceived) break;
      } else {
        response.body.insert(response.body.end(), responseData.begin(),
                             responseData.end());
        responseData.clear();

        // got the whole content
        if (contentLengthReceived && response.body.size() >= contentLength) {
          break;
        }
      }
    }
  }

  return response;
}

}  // namespace qcraft::http
