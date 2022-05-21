#include "onboard/utils/stream/stream_factory.h"

#include <cstdint>
#include <string>

#include "onboard/utils/stream/ntrip_stream.h"
#include "onboard/utils/stream/serial_stream.h"
#include "onboard/utils/stream/socketcan_stream.h"
#include "onboard/utils/stream/tcp_stream.h"
#include "onboard/utils/stream/udp_stream.h"

namespace qcraft {

namespace stream {
Stream* create_tcp(const char* address, uint16_t port, uint32_t timeout_usec) {
  return new TcpStream(address, port, timeout_usec);
}

Stream* create_udp(const char* address, uint16_t port, uint32_t timeout_usec,
                   uint16_t src_port) {
  return new UdpStream(address, port, timeout_usec, src_port);
}

Stream* create_serial(const char* device_name, uint32_t baud_rate) {
  speed_t baud = SerialStream::get_serial_baudrate(baud_rate);
  return baud == 0 ? nullptr : new SerialStream(device_name, baud);
}

Stream* create_ntrip(const std::string& address, uint16_t port,
                     const std::string& mountpoint, const std::string& user,
                     const std::string& passwd, uint32_t timeout_s) {
  return new NtripStream(address, port, mountpoint, user, passwd, timeout_s);
}

Stream* create_socketcan(const std::string& if_name, int32_t bitrate,
                         uint32_t timeout_usec) {
  return new SocketCanStream(if_name, bitrate, timeout_usec);
}

}  // namespace stream
}  // namespace qcraft
