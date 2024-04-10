/**
 * @file timer.hpp
 * @author Lorenzo Feng (lorenzo.feng@njust.edu.cn)
 * @brief 定时器
 * @version 0.1
 * @date 2024-04-10
 *
 * @copyright Copyright (c) 2024
 *
 */

#include <chrono>

class Timer final {
public:
    Timer()
        : running_(false) {}

    void start() {
        running_ = true;
        start_   = std::chrono::high_resolution_clock::now();
    }

    void stop() {
        if (running_) {
            end_     = std::chrono::high_resolution_clock::now();
            running_ = false;
        }
    }

    double elapsed() {
        if (running_) {
            return std::chrono::duration<double>(std::chrono::high_resolution_clock::now() - start_)
                       .count()
                 * 1000;
        } else {
            return std::chrono::duration<double>(end_ - start_).count() * 1000;
        }
    }

    void reset() { running_ = false; }

    bool running() { return running_; }

private:
    bool running_;
    std::chrono::time_point<std::chrono::high_resolution_clock> start_, end_;
};