#include "FPSCounter.h"
#include "Common/Color.h"

void FPSCounter::Count() {
    _timeStamps.push_back(TimeStampCounter::GetTimeStamp());
}

int FPSCounter::GetFPS() {
    TimeStamp _presentTime = TimeStampCounter::GetTimeStamp();
    while (!_timeStamps.empty() && 
            _presentTime - _timeStamps.first() > 1000)
        _timeStamps.pop();
    return _timeStamps.size();
}

void FPSCounter::PrintFPS(cv::Mat& img) {
    cv::putText(img, std::to_string(GetFPS()),
        cv::Point(), 0, 1, COLOR_YELLOW);
}
