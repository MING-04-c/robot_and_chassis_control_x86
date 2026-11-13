#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace sensor_timing {
// 仅统计电脑正长度read和解码/发布，不冒充传感器内部采样时间。
struct ReceiveWindow {
  bool started{false}, have_read{false};
  double begin{0.0}, last_read{0.0}, gap_sum{0.0}, max_gap{0.0};
  std::uint64_t reads{0}, bytes{0}, gaps{0}, decoded{0}, published{0};
  std::size_t max_bytes{0}, current_decoded{0}, max_decoded_per_read{0};

  void on_read(double now, std::size_t size) {
    if (!started) { started = true; begin = now; }
    if (have_read) {
      const auto gap = std::max(0.0, now-last_read);
      gap_sum += gap;
      max_gap = std::max(max_gap, gap);
      ++gaps;
    }
    have_read = true;
    last_read = now;
    ++reads;
    bytes += size;
    max_bytes = std::max(max_bytes, size);
    current_decoded = 0;
  }
  void on_decoded() {
    ++decoded;
    ++current_decoded;
    max_decoded_per_read = std::max(max_decoded_per_read, current_decoded);
  }
  double elapsed(double now) const { return started ? std::max(0.0, now-begin) : 0.0; }
  double silence(double now) const { return have_read ? std::max(0.0, now-last_read) : 0.0; }
  void next_window(double now) {
    // 跨窗口间隙仍需计入，不能让一次长空档在清零时消失。
    const bool old_have_read = have_read;
    const double old_last_read = last_read;
    *this = ReceiveWindow{};
    started = true;
    begin = now;
    have_read = old_have_read;
    last_read = old_last_read;
  }
};
}  // namespace sensor_timing
