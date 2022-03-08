#include <atomic>
#include <cassert>
#include <thread>
#include <vector>

std::atomic<bool> x = {false};
std::atomic<bool> y = {false};
std::atomic<int> z = {0};

void write_x() { x.store(true, std::memory_order_seq_cst); }

void write_y() { y.store(true, std::memory_order_seq_cst); }

void read_x_then_y() {
  while (!x.load(std::memory_order_seq_cst))
    ;
  if (y.load(std::memory_order_seq_cst)) {
    ++z;
  }
}

void read_y_then_x() {
  while (!y.load(std::memory_order_seq_cst))
    ;
  if (x.load(std::memory_order_seq_cst)) {
    ++z;
  }
}

int main() {
  std::vector<std::thread> threads;

  threads.push_back(std::thread(write_x));
  threads.push_back(std::thread(write_y));
  threads.push_back(std::thread(read_x_then_y));
  threads.push_back(std::thread(read_y_then_x));

  for (std::thread &t : threads) {
    if (t.joinable()) {
      t.join();
    }
  }

  assert(z.load() != 0);  // will never happen
}