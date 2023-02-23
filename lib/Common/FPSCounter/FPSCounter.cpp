#include "FPSCounter.h"
#include <Common/Color.h>
#include <Parameters.h>

int FPSCounter::Count() {
    if (!_timeStamps.empty()) {
        int _millisecDif = 1000. / MAX_FPS - (TimeStampCounter::GetTimeStamp() - _timeStamps.last());
        if (_millisecDif > 0) cv::waitKey(_millisecDif);
    }

    TimeStamp _presentTime = TimeStampCounter::GetTimeStamp();
    _timeStamps.push_back(_presentTime);
    while (!_timeStamps.empty() &&
            _presentTime - _timeStamps.first() > 1000)
        _timeStamps.pop();
    return _timeStamps.size();
}

void FPSCounter::PrintFPS(cv::Mat& img) {
    cv::putText(img, "Fps : " + std::to_string(_timeStamps.size()),
        cv::Point(0, img.rows - 10), 0, 1, COLOR_LIME);
}
