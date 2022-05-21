#ifndef ONBOARD_VIS_HTTP_REQUEST_H_
#define ONBOARD_VIS_HTTP_REQUEST_H_

#include <errno.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <type_traits>
#include <vector>

namespace qcraft::http {

enum class InternetProtocol : std::uint8_t { V4, V6 };

std::string urlEncode(const std::string& str);

struct Response final {
  enum Status {
    Continue = 100,
    SwitchingProtocol = 101,
    Processing = 102,
    EarlyHints = 103,

    Ok = 200,
    Created = 201,
    Accepted = 202,
    NonAuthoritativeInformation = 203,
    NoContent = 204,
    ResetContent = 205,
    PartialContent = 206,
    MultiStatus = 207,
    AlreadyReported = 208,
    ImUsed = 226,

    MultipleChoice = 300,
    MovedPermanently = 301,
    Found = 302,
    SeeOther = 303,
    NotModified = 304,
    UseProxy = 305,
    TemporaryRedirect = 307,
    PermanentRedirect = 308,

    BadRequest = 400,
    Unauthorized = 401,
    PaymentRequired = 402,
    Forbidden = 403,
    NotFound = 404,
    MethodNotAllowed = 405,
    NotAcceptable = 406,
    ProxyAuthenticationRequired = 407,
    RequestTimeout = 408,
    Conflict = 409,
    Gone = 410,
    LengthRequired = 411,
    PreconditionFailed = 412,
    PayloadTooLarge = 413,
    UriTooLong = 414,
    UnsupportedMediaType = 415,
    RangeNotSatisfiable = 416,
    ExpectationFailed = 417,
    ImaTeapot = 418,
    MisdirectedRequest = 421,
    UnprocessableEntity = 422,
    Locked = 423,
    FailedDependency = 424,
    TooEarly = 425,
    UpgradeRequired = 426,
    PreconditionRequired = 428,
    TooManyRequests = 429,
    RequestHeaderFieldsTooLarge = 431,
    UnavailableForLegalReasons = 451,

    InternalServerError = 500,
    NotImplemented = 501,
    BadGateway = 502,
    ServiceUnavailable = 503,
    GatewayTimeout = 504,
    HttpVersionNotSupported = 505,
    VariantAlsoNegotiates = 506,
    InsufficientStorage = 507,
    LoopDetected = 508,
    NotExtended = 510,
    NetworkAuthenticationRequired = 511
  };

  int status = 0;
  std::vector<std::string> headers;
  std::vector<std::uint8_t> body;
};

class Request final {
 public:
  explicit Request(const std::string& url,
                   InternetProtocol protocol = InternetProtocol::V4);

  Response send(const std::string& method,
                const std::map<std::string, std::string>& parameters,
                const std::vector<std::string>& headers = {});

  Response send(const std::string& method = "GET", const std::string& body = "",
                const std::vector<std::string>& headers = {});

 private:
  InternetProtocol internetProtocol;
  std::string scheme;
  std::string domain;
  std::string port;
  std::string path;
};

}  // namespace qcraft::http

#endif  // ONBOARD_VIS_HTTP_REQUEST_H_
