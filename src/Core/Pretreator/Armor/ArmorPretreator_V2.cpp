#include "ArmorPretreator_V2.h"

ArmorPretreator_V2::ArmorPretreator_V2() : _colorIdentifier(228.0f) { }


void ArmorPretreator_V2::LoopPixel(const uchar* src, uchar* dst, int n) const {
    for (int i = 0; i < n; i += 1, src += 3) {
        cv::Vec3b color(src[0], src[1], src[2]);
        dst[i] = 0;//static_cast<uchar>(_colorIdentifier.Identify(color));
    }
}

void ArmorPretreator_V2::Threshold(const cv::Mat& src, cv::Mat& dst) const {
    cv::Size sz = src.size();
    const uchar* srcBuf = src.data;
    uchar* dstBuf = dst.data;
    size_t srcstep = src.step, dststep = dst.step;

    if (src.isContinuous() && dst.isContinuous()) {
        sz.width *= sz.height;
        sz.height = 1;
    }

    for (; sz.height--; srcBuf += srcstep, dstBuf += dststep)
        LoopPixel(srcBuf, dstBuf, sz.width);
}

std::tuple<cv::Mat, cv::Mat> ArmorPretreator_V2::GetPretreated(const cv::Mat& img) const {
    std::tuple<cv::Mat, cv::Mat> result;
    auto& [imgThre, imgGray] = result;
	if (imgThre.empty())
        imgThre.create(img.size(), CV_8UC1);
	Threshold(img, imgThre);
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
    return result;
}
