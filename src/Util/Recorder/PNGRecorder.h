#pragma once 
/*
Creation Date: 2023/06/10
Latest Update: 2023/06/10
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
*/

#include <chrono>
#include <thread>
#include <chrono>
#include <optional>

#include <eigen3/Eigen/Dense>

#include "Util/Serial/SerialUtil.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Core/Transformer/IMUTransformer.h"


#include <thread>
#include <condition_variable>
#include <mutex>
#include <list>
#include <iostream>


class PNGRecorder {
public:
    explicit PNGRecorder(const char* dir, double maxFps) : _folderPath(dir), _thread(&PNGRecorder::serialMain, this) {
        constexpr double ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds(1)).count();
        double interval = ns / maxFps;
        if (isfinite(interval)) {
            _minFrameInterval = std::chrono::nanoseconds(static_cast<long long>(interval));
            _compressionParams.push_back(cv::IMWRITE_PNG_COMPRESSION);
            _compressionParams.push_back(0);    // 压缩等级
            _enabled = true;
        }
        else _enabled = false;
    }

    ~PNGRecorder() {
        _destructed = true;
        _thread.join();
    }

    void Record(const cv::Mat& img, const std::chrono::steady_clock::time_point& timeStamp) {
        if (_enabled && timeStamp - _lastRecord > _minFrameInterval) {
            _lastRecord = timeStamp;
            std::lock_guard<std::mutex> locker(_mutex);
            if (_queue.size() <= 5) {
                _queue.push_back(img.clone());
                _waitNotEmpty.notify_one();
            }
            else LOG(ERROR) << "Queue size exceeded!";
        }
    }

private:
    void serialMain() {
        try {
            while (!_destructed) {
                cv::Mat img;
                bool gotImage = false;
                do {
                    std::unique_lock<std::mutex> locker(_mutex);
                    gotImage = _waitNotEmpty.wait_for(locker, std::chrono::milliseconds(200), [this] {
                        return !_queue.empty();
                    });
                    if (gotImage) {
                        img = _queue.front();
                        _queue.pop_front();
                    }
                } while (false);
                if (gotImage) {
                    cv::imwrite(_folderPath + std::to_string(_lastRecord.time_since_epoch().count()) + std::string(".png"), img, _compressionParams);
                }
            }
            return;
        }
        catch (std::exception& e) {
            LOG(ERROR) << "PNGRecorder: Uncaught " << typeid(e).name() << ": " << e.what();
        }
        catch (...) {
            LOG(ERROR) << "PNGRecorder: Uncaught unknown error";
        }
        LOG(ERROR) << "PNGRecorder crashed... bye";
    }

    bool _enabled;

    const std::string _folderPath;

    std::atomic<bool> _destructed = false;
    std::thread _thread;

    std::vector<int> _compressionParams;

    std::mutex _mutex;
    std::list<cv::Mat> _queue;
    std::condition_variable _waitNotEmpty;

    std::chrono::nanoseconds _minFrameInterval;
    std::chrono::steady_clock::time_point _lastRecord;
};
