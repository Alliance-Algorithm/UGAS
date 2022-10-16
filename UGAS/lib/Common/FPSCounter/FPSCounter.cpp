#include "FPSCounter.h"
#include "../Color.h"

void FPSCounter::Count() {
    _timeStamps.push(TimeStampCounter::GetTimeStamp());
}

int FPSCounter::GetFPS() {
    TimeStamp _presentTime = TimeStampCounter::GetTimeStamp();
    while (!_timeStamps.empty() && 
            _presentTime - _timeStamps.front() > 1000)
        _timeStamps.pop();
    return _timeStamps.size();
}

void FPSCounter::PrintFPS(cv::Mat& img) {
    cv::putText(img, std::to_string(GetFPS()),
        cv::Point(), 0, 1, COLOR_YELLOW);
}
