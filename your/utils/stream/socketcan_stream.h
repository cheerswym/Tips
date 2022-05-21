#ifndef ONBOARD_UTILS_STREAM_SOCKETCAN_STREAM_H_
#define ONBOARD_UTILS_STREAM_SOCKETCAN_STREAM_H_

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdint>
#include <string>

#include "onboard/utils/stream/stream.h"

namespace qcraft {
class SocketCanStream : public Stream {
 public:
  SocketCanStream(const std::string &if_name, int32_t bitrate,
                  uint32_t timeout_usec);
  ~SocketCanStream();
  bool Connect() override;
  bool Disconnect() override;

  size_t read(uint8_t *buffer, size_t max_length) override;

  size_t write(const uint8_t *buffer, size_t length) override;

  // support socket CAN specific I/O
  size_t read(struct can_frame *frame);
  size_t write(uint32_t id, bool extended, uint8_t dlc, const uint8_t data[8]);

 private:
  const std::string if_name_;
  [[maybe_unused]] const int32_t bitrate_;
  const uint32_t timeout_usec_;
  int sockfd_ = -1;

  struct sockaddr_can addr_;
  struct ifreq ifr_;

  void open();
  void close();
};
}  // namespace qcraft

#endif  // ONBOARD_UTILS_STREAM_SOCKETCAN_STREAM_H_
