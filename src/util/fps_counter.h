#pragma once

#include <chrono>

class FpsCounter {
public:
    bool count() {
        if (count_ == 0) {
            count_       = 1;
            _timingStart = std::chrono::steady_clock::now();
        } else {
            ++count_;
            if (std::chrono::steady_clock::now() - _timingStart >= std::chrono::seconds(1)) {
                last_  = count_;
                count_ = 0;
                return true;
            }
        }
        return false;
    }

    int get_fps() { return last_; }

private:
    int count_ = 0, last_ = 0;
    std::chrono::steady_clock::time_point _timingStart;
};