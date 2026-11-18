// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#ifndef OMNIBOT_HARDWARE__SERIAL_PORT_HPP_
#define OMNIBOT_HARDWARE__SERIAL_PORT_HPP_

#include <cstdint>
#include <string>
#include <vector>

namespace omnibot_hardware
{

// Non-blocking byte pipe. read() and write() of the hardware plugin call
// write_bytes and read_available only. read_for sleeps and is for configure
// and activate, which are allowed to block.
class SerialPort
{
public:
  virtual ~SerialPort() = default;

  virtual bool open_port(const std::string & path, int baud, std::string & error) = 0;
  virtual void close_port() = 0;
  virtual bool is_open() const = 0;
  virtual bool write_bytes(const std::vector<uint8_t> & data, std::string & error) = 0;
  virtual std::vector<uint8_t> read_available() = 0;

  // Returns whatever arrived before timeout_ms. Empty on timeout.
  std::vector<uint8_t> read_for(int timeout_ms);
};

class LinuxSerialPort : public SerialPort
{
public:
  LinuxSerialPort() = default;
  ~LinuxSerialPort() override;

  bool open_port(const std::string & path, int baud, std::string & error) override;
  void close_port() override;
  bool is_open() const override;
  bool write_bytes(const std::vector<uint8_t> & data, std::string & error) override;
  std::vector<uint8_t> read_available() override;

private:
  int fd_{-1};
};

}  // namespace omnibot_hardware

#endif  // OMNIBOT_HARDWARE__SERIAL_PORT_HPP_
