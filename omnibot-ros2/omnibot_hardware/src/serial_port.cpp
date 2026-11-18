// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#include "omnibot_hardware/serial_port.hpp"

#include <chrono>
#include <thread>

#if defined(__linux__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#endif

namespace omnibot_hardware
{

std::vector<uint8_t> SerialPort::read_for(int timeout_ms)
{
  std::vector<uint8_t> out;
  const auto start = std::chrono::steady_clock::now();
  while (true) {
    auto chunk = read_available();
    out.insert(out.end(), chunk.begin(), chunk.end());
    if (!out.empty()) {
      return out;
    }
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start).count();
    if (elapsed >= timeout_ms) {
      return out;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}

LinuxSerialPort::~LinuxSerialPort()
{
  close_port();
}

bool LinuxSerialPort::open_port(const std::string & path, int baud, std::string & error)
{
  close_port();
#if defined(__linux__)
  speed_t speed = B115200;
  if (baud == 115200) {
    speed = B115200;
  } else if (baud == 1000000) {
    speed = B1000000;
  } else {
    error = "unsupported baud " + std::to_string(baud) + " (use 115200 or 1000000)";
    return false;
  }
  fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    error = "open " + path + " failed: " + std::strerror(errno);
    return false;
  }
  termios tio{};
  if (tcgetattr(fd_, &tio) != 0) {
    error = "tcgetattr failed: " + std::string(std::strerror(errno));
    close_port();
    return false;
  }
  cfmakeraw(&tio);
  cfsetispeed(&tio, speed);
  cfsetospeed(&tio, speed);
  tio.c_cflag |= (CLOCAL | CREAD);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
    error = "tcsetattr failed: " + std::string(std::strerror(errno));
    close_port();
    return false;
  }
  return true;
#else
  (void)path;
  (void)baud;
  error = "serial ports are only opened on Linux";
  return false;
#endif
}

void LinuxSerialPort::close_port()
{
#if defined(__linux__)
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
#else
  fd_ = -1;
#endif
}

bool LinuxSerialPort::is_open() const
{
  return fd_ >= 0;
}

bool LinuxSerialPort::write_bytes(const std::vector<uint8_t> & data, std::string & error)
{
#if defined(__linux__)
  if (fd_ < 0) {
    error = "serial port is not open";
    return false;
  }
  size_t sent = 0;
  while (sent < data.size()) {
    const ssize_t n = ::write(fd_, data.data() + sent, data.size() - sent);
    if (n < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK) {
        error = "serial write would block";
      } else {
        error = "serial write failed: " + std::string(std::strerror(errno));
      }
      return false;
    }
    sent += static_cast<size_t>(n);
  }
  return true;
#else
  (void)data;
  error = "serial ports are only opened on Linux";
  return false;
#endif
}

std::vector<uint8_t> LinuxSerialPort::read_available()
{
  std::vector<uint8_t> out;
#if defined(__linux__)
  if (fd_ < 0) {
    return out;
  }
  uint8_t buf[256];
  while (out.size() < 4096) {
    const ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n > 0) {
      out.insert(out.end(), buf, buf + n);
      if (static_cast<size_t>(n) < sizeof(buf)) {
        break;
      }
      continue;
    }
    break;
  }
#endif
  return out;
}

}  // namespace omnibot_hardware
