#pragma once

#include <cstdint>
#include <string>

#include "onboard/utils/stream/stream.h"

namespace qcraft {
namespace stream {

// Return a pointer to a Stream object. The caller should take ownership.
Stream *create_tcp(const char *address, uint16_t port,
                   uint32_t timeout_usec = 1000000);

// Return a pointer to a Stream object. The caller should take ownership.
// If the optional 'src_port' is provided, will try to bind to that port.
// The default if not bind to any specific port (port is chosen by the OS).
Stream *create_udp(const char *address, uint16_t port,
                   uint32_t timeout_usec = 1000000, uint16_t src_port = 0);

// Currently the following baud rates are supported:
//  9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600.
Stream *create_serial(const char *device_name, uint32_t baud_rate);

Stream *create_ntrip(const std::string &address, uint16_t port,
                     const std::string &mountpoint, const std::string &user,
                     const std::string &passwd, uint32_t timeout_s = 30);
Stream *create_socketcan(const std::string &if_name, int32_t bitrate,
                         uint32_t timeout_usec);
}  // namespace stream
}  // namespace qcraft
