#include "onboard/utils/stream/socketcan_stream.h"

#include <fcntl.h>

#include <algorithm>
#include <cstring>

#include "onboard/lite/logging.h"

namespace qcraft {

inline bool LogIf(bool failed, const char *what, int fd = -1) {
  if (failed) {
    QLOG(ERROR) << what << " failed, errno: " << errno << ", "
                << strerror(errno);
    if (fd > 0) ::close(fd);
  }
  return failed;
}

SocketCanStream::SocketCanStream(const std::string &if_name, int32_t bitrate,
                                 uint32_t timeout_usec)
    : if_name_(if_name), bitrate_(bitrate), timeout_usec_(timeout_usec) {}

SocketCanStream::~SocketCanStream() { close(); }

void SocketCanStream::open() {
  CHECK(if_name_.size() < IF_NAMESIZE) << "Linux IF_NAMESIZE limit is 16";
  int fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (LogIf(fd < 0, "Create socket")) return;

  // ifr_.ifr_name is a char array, not c_str
  std::snprintf(ifr_.ifr_name, IF_NAMESIZE, "%s", if_name_.c_str());
  if (LogIf(ioctl(fd, SIOCGIFINDEX, &ifr_) != 0, "Query CAN interface index"))
    return;

  int flags = fcntl(fd, F_GETFL, 0);
  if (LogIf(flags == -1, "Get CAN flags", fd)) return;
  if (timeout_usec_ > 0) {
    if (LogIf(fcntl(fd, F_SETFL, flags & ~O_NONBLOCK) == -1,
              "Set blocking mode", fd))
      return;

    struct timeval block_to = {timeout_usec_ / 1000000,
                               timeout_usec_ % 1000000};
    if (LogIf(setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO,
                         reinterpret_cast<char *>(&block_to),
                         sizeof(block_to)) < 0,
              "Set recv timeout", fd))
      return;

    if (LogIf(setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO,
                         reinterpret_cast<char *>(&block_to),
                         sizeof(block_to)) < 0,
              "Set send timeout", fd))
      return;
  } else if (LogIf(fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1,
                   "Set blocking mode", fd)) {
    return;
  }

  memset(&addr_, 0, sizeof(addr_));
  addr_.can_family = AF_CAN;
  addr_.can_ifindex = ifr_.ifr_ifindex;
  if (LogIf(bind(fd, (struct sockaddr *)&addr_, sizeof(addr_)) < 0,
            "Bind to socket CAN"))
    return;

  sockfd_ = fd;
}

void SocketCanStream::close() {
  if (sockfd_ > 0) {
    ::close(sockfd_);
    sockfd_ = -1;
    status_ = Stream::Status::DISCONNECTED;
  }
}

bool SocketCanStream::Connect() {
  if (sockfd_ < 0) {
    this->open();
    if (sockfd_ < 0) {
      return false;
    }
  }

  status_ = Stream::Status::CONNECTED;
  return true;
}

bool SocketCanStream::Disconnect() {
  if (sockfd_ < 0) {
    // not open
    return false;
  }

  this->close();
  return true;
}

size_t SocketCanStream::read(uint8_t *buffer, size_t max_length) {
  return ::read(sockfd_, &buffer, sizeof(struct can_frame));
}

size_t SocketCanStream::read(struct can_frame *frame) {
  return ::read(sockfd_, frame, sizeof(struct can_frame));
}

size_t SocketCanStream::write(const uint8_t *data, size_t length) {
  return ::write(sockfd_, data, sizeof(struct can_frame));
}

size_t SocketCanStream::write(uint32_t id, bool extended, uint8_t dlc,
                              const uint8_t data[8]) {
  struct can_frame frame;
  frame.can_id = id;
  frame.can_dlc = dlc;
  std::copy(data, data + 8, frame.data);
  return ::write(sockfd_, &frame, sizeof(frame));
}

}  // namespace qcraft
