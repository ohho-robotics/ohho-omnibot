// Copyright 2026 OhhO Robotics
// SPDX-License-Identifier: Apache-2.0

#include "omnibot_hardware/calibration.hpp"

#include <cctype>
#include <fstream>
#include <sstream>

namespace omnibot_hardware
{
namespace calibration
{
namespace
{

std::string strip_comment(const std::string & line)
{
  bool quoted = false;
  for (size_t i = 0; i < line.size(); ++i) {
    if (line[i] == '"') {
      quoted = !quoted;
    }
    if (line[i] == '#' && !quoted) {
      return line.substr(0, i);
    }
  }
  return line;
}

std::string trim(const std::string & text)
{
  size_t begin = 0;
  size_t end = text.size();
  while (begin < end && std::isspace(static_cast<unsigned char>(text[begin]))) {
    ++begin;
  }
  while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1]))) {
    --end;
  }
  return text.substr(begin, end - begin);
}

int indent_of(const std::string & line)
{
  int indent = 0;
  while (indent < static_cast<int>(line.size()) && line[static_cast<size_t>(indent)] == ' ') {
    ++indent;
  }
  return indent;
}

bool parse_int(const std::string & text, int & value)
{
  try {
    size_t used = 0;
    value = std::stoi(trim(text), &used);
    return used == trim(text).size();
  } catch (const std::exception &) {
    return false;
  }
}

bool in_range(int ticks)
{
  return ticks >= 0 && ticks <= 4095;
}

bool parse_inline_list(const std::string & value, std::array<int, 6> & home_ticks, std::string & error)
{
  const auto open = value.find('[');
  const auto close = value.find(']');
  if (open == std::string::npos || close == std::string::npos || close < open) {
    error = "home_ticks must be a list of six integers";
    return false;
  }
  std::stringstream body(value.substr(open + 1, close - open - 1));
  std::string item;
  int count = 0;
  while (std::getline(body, item, ',')) {
    if (trim(item).empty()) {
      continue;
    }
    if (count >= 6) {
      error = "home_ticks must contain exactly six integers";
      return false;
    }
    int ticks = 0;
    if (!parse_int(item, ticks) || !in_range(ticks)) {
      error = "home_ticks values must be integers from 0 through 4095";
      return false;
    }
    home_ticks[static_cast<size_t>(count)] = ticks;
    ++count;
  }
  if (count != 6) {
    error = "home_ticks must contain exactly six integers";
    return false;
  }
  return true;
}

}  // namespace

bool parse_home_ticks_yaml(
  const std::string & text, std::array<int, 6> & home_ticks, std::string & error)
{
  bool saw_home = false;
  bool in_home_block = false;
  bool saw_joints = false;
  bool in_joints = false;
  int block_count = 0;
  int joint_index = -1;
  std::array<int, 6> joint_ticks{};
  std::array<bool, 6> joint_seen{};

  std::stringstream lines(text);
  std::string raw;
  while (std::getline(lines, raw)) {
    if (!raw.empty() && raw.back() == '\r') {
      raw.pop_back();
    }
    const std::string line = strip_comment(raw);
    if (trim(line).empty()) {
      continue;
    }
    const int indent = indent_of(line);
    const std::string body = trim(line);

    if (indent == 0) {
      if (in_home_block && block_count != 6) {
        error = "home_ticks must contain exactly six integers";
        return false;
      }
      in_home_block = false;
      in_joints = false;
      const auto colon = body.find(':');
      if (colon == std::string::npos) {
        continue;
      }
      const std::string key = trim(body.substr(0, colon));
      const std::string value = trim(body.substr(colon + 1));
      if (key == "home_ticks") {
        saw_home = true;
        if (value.empty()) {
          in_home_block = true;
          block_count = 0;
        } else if (!parse_inline_list(value, home_ticks, error)) {
          return false;
        }
      } else if (key == "joints") {
        saw_joints = true;
        in_joints = true;
        joint_index = -1;
      }
      continue;
    }

    if (in_home_block) {
      if (body.rfind("-", 0) != 0) {
        error = "home_ticks block entries must be a list";
        return false;
      }
      if (block_count >= 6) {
        error = "home_ticks must contain exactly six integers";
        return false;
      }
      int ticks = 0;
      if (!parse_int(body.substr(1), ticks) || !in_range(ticks)) {
        error = "home_ticks values must be integers from 0 through 4095";
        return false;
      }
      home_ticks[static_cast<size_t>(block_count)] = ticks;
      ++block_count;
      continue;
    }

    if (!in_joints) {
      continue;
    }
    std::string item = body;
    if (item.rfind("-", 0) == 0) {
      ++joint_index;
      if (joint_index >= 6) {
        error = "joints must contain exactly six entries";
        return false;
      }
      item = trim(item.substr(1));
      if (item.empty()) {
        continue;
      }
    }
    if (joint_index < 0) {
      error = "joints entries must be a list";
      return false;
    }
    const auto colon = item.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string key = trim(item.substr(0, colon));
    const std::string value = trim(item.substr(colon + 1));
    // Exact key. home_ticks and home_tick_note are not home_tick.
    if (key != "home_tick") {
      continue;
    }
    int ticks = 0;
    if (!parse_int(value, ticks) || !in_range(ticks)) {
      error = "joints home_tick must be an integer from 0 through 4095";
      return false;
    }
    joint_ticks[static_cast<size_t>(joint_index)] = ticks;
    joint_seen[static_cast<size_t>(joint_index)] = true;
  }

  if (in_home_block && block_count != 6) {
    error = "home_ticks must contain exactly six integers";
    return false;
  }
  if (!saw_home) {
    error = "calibration yaml is missing home_ticks";
    return false;
  }
  if (saw_joints) {
    if (joint_index != 5) {
      error = "joints must contain exactly six entries";
      return false;
    }
    for (int i = 0; i < 6; ++i) {
      if (!joint_seen[static_cast<size_t>(i)]) {
        error = "joints entry is missing home_tick";
        return false;
      }
      if (joint_ticks[static_cast<size_t>(i)] != home_ticks[static_cast<size_t>(i)]) {
        error = "home_ticks does not match joints home_tick";
        return false;
      }
    }
  }
  return true;
}

bool load_home_ticks_file(
  const std::string & path, std::array<int, 6> & home_ticks, std::string & error)
{
  std::ifstream input(path);
  if (!input) {
    error = "Calibration file not found: " + path;
    return false;
  }
  std::stringstream buffer;
  buffer << input.rdbuf();
  if (!parse_home_ticks_yaml(buffer.str(), home_ticks, error)) {
    error = path + ": " + error;
    return false;
  }
  return true;
}

}  // namespace calibration
}  // namespace omnibot_hardware
