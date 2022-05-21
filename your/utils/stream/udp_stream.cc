#include "onboard/utils/stream/udp_stream.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>

#include "onboard/lite/logging.h"

namespace qcraft {

UdpStream::UdpStream(const char* address, uint16_t port, uint32_t timeout_usec,
                     uint16_t src_port)
    : src_port_(src_port), sockfd_(-1), errno_(0) {
  peer_addr_ = inet_addr(address);
  peer_port_ = htons(port);
  timeout_usec_ = timeout_usec;
  // call open or call open in connect later
}

UdpStream::~UdpStream() { this->close(); }

void UdpStream::open() {
  int fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) {
    // error
    QLOG(ERROR) << "Create socket failed, errno: " << errno << ", "
                << strerror(errno);
    return;
  }

  if (src_port_) {
    struct sockaddr_in srcaddr;
    memset(&srcaddr, 0, sizeof(srcaddr));
    srcaddr.sin_family = AF_INET;
    srcaddr.sin_addr.s_addr = htonl(INADDR_ANY);
    srcaddr.sin_port = htons(src_port_);
    if (bind(fd, (struct sockaddr*)&srcaddr, sizeof(srcaddr)) < 0) {
      QLOG(ERROR) << "Failed to bind to UDP port: " << src_port_;
      return;
    }
  }

  // block or not block
  if (timeout_usec_ != 0) {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
      ::close(fd);
      QLOG(ERROR) << "fcntl get flag failed, errno: " << errno << ", "
                  << strerror(errno);
      return;
    }

    if (fcntl(fd, F_SETFL, flags & ~O_NONBLOCK) == -1) {
      ::close(fd);
      QLOG(ERROR) << "fcntl set block failed, errno: " << errno << ", "
                  << strerror(errno);
      return;
    }

    struct timeval block_to = {timeout_usec_ / 1000000,
                               timeout_usec_ % 1000000};
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO,
                   reinterpret_cast<char*>(&block_to), sizeof(block_to)) < 0) {
      ::close(fd);
      QLOG(ERROR) << "setsockopt set rcv timeout failed, errno: " << errno
                  << ", " << strerror(errno);
      return;
    }

    if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO,
                   reinterpret_cast<char*>(&block_to), sizeof(block_to)) < 0) {
      ::close(fd);
      QLOG(ERROR) << "setsockopt set snd timeout failed, errno: " << errno
                  << ", " << strerror(errno);
      return;
    }
  } else {
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1) {
      ::close(fd);
      QLOG(ERROR) << "fcntl get flag failed, errno: " << errno << ", "
                  << strerror(errno);
      return;
    }

    if (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
      ::close(fd);
      QLOG(ERROR) << "fcntl set non block failed, errno: " << errno << ", "
                  << strerror(errno);
      return;
    }
  }

  sockfd_ = fd;
}

void UdpStream::close() {
  if (sockfd_ > 0) {
    ::close(sockfd_);
    sockfd_ = -1;
    status_ = Stream::Status::DISCONNECTED;
  }
}

bool UdpStream::Connect() {
  if (sockfd_ < 0) {
    this->open();
    if (sockfd_ < 0) {
      return false;
    }
  }

  if (status_ == Stream::Status::CONNECTED) {
    return true;
  }

  // upper layer support ping method ??
  status_ = Stream::Status::CONNECTED;
  return true;
}

bool UdpStream::Disconnect() {
  if (sockfd_ < 0) {
    // not open
    return false;
  }

  this->close();
  return true;
}

size_t UdpStream::read(uint8_t* buffer, size_t max_length) {
  ssize_t ret = 0;
  //// the addresses were not used, can be NULL
  // struct sockaddr_in peer_sockaddr;
  // socklen_t socklenth = sizeof(peer_sockaddr);
  // bzero(&peer_sockaddr, sizeof(peer_sockaddr));
  // peer_sockaddr.sin_family = AF_INET;
  // peer_sockaddr.sin_port = peer_port_;
  // peer_sockaddr.sin_addr.s_addr = peer_addr_;

  while ((ret = ::recvfrom(sockfd_, buffer, max_length, 0, NULL, NULL)) < 0) {
    if (errno == EINTR) {
      continue;
    } else {
      // error
      if (errno != EAGAIN) {
        status_ = Stream::Status::ERROR;
        errno_ = errno;
      }
    }

    return 0;
  }

  return ret;
}

size_t UdpStream::write(const uint8_t* data, size_t length) {
  size_t total_nsent = 0;
  struct sockaddr_in peer_sockaddr;
  bzero(&peer_sockaddr, sizeof(peer_sockaddr));
  peer_sockaddr.sin_family = AF_INET;
  peer_sockaddr.sin_port = peer_port_;
  peer_sockaddr.sin_addr.s_addr = peer_addr_;

  while (length > 0) {
    ssize_t nsent =
        ::sendto(sockfd_, data, length, 0, (struct sockaddr*)&peer_sockaddr,
                 (socklen_t)sizeof(peer_sockaddr));
    if (nsent < 0) {  // error
      if (errno == EINTR) {
        continue;
      } else {
        // error
        if (errno == EPIPE || errno == ECONNRESET) {
          status_ = Stream::Status::DISCONNECTED;
          errno_ = errno;
        } else if (errno != EAGAIN) {
          status_ = Stream::Status::ERROR;
          errno_ = errno;
        }
        return total_nsent;
      }
    }

    total_nsent += nsent;
    length -= nsent;
    data += nsent;
  }

  return total_nsent;
}

}  // namespace qcraft
