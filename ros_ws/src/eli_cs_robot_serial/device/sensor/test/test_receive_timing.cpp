// Pure offline test: no ROS, sockets, devices or robot motion.
#include "../src/receive_timing.hpp"
#include <cassert>
#include <cmath>

int main() {
  sensor_timing::ReceiveWindow window;
  assert(window.elapsed(10) == 0);
  window.on_read(10., 29);
  window.on_decoded();
  ++window.published;
  window.on_read(10.07, 145);  // 70ms空档后同一次read解出5帧。
  for (int i = 0; i < 5; ++i) { window.on_decoded(); ++window.published; }
  assert(window.reads == 2 && window.bytes == 174);
  assert(window.decoded == 6 && window.published == 6);
  assert(window.max_decoded_per_read == 5 && window.max_bytes == 145);
  assert(std::abs(window.max_gap-.07) < 1e-10);
  assert(std::abs(window.silence(10.1)-.03) < 1e-10);
  window.next_window(10.1);
  assert(window.reads == 0 && window.decoded == 0 && window.max_decoded_per_read == 0);
  window.on_read(10.14, 58);
  assert(std::abs(window.max_gap-.07) < 1e-10);  // 跨窗口也不丢失间隔。
  window.on_decoded();
  window.on_decoded();
  assert(window.max_decoded_per_read == 2);
  window = sensor_timing::ReceiveWindow{};  // 重连不会把断线时间混入新连接收包统计。
  window.on_read(20., 29);
  assert(window.gaps == 0 && window.max_gap == 0);
}
