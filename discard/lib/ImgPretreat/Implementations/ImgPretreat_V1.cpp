#include "ImgPretreat_V1.h"
using namespace cv;

std::tuple<cv::Mat, cv::Mat> ImgPretreat_V1::GetPretreated(const cv::Mat& img) {
	if (img.empty())
		throw_with_trace(std::runtime_error, "Get empty img!");

	std::tuple<cv::Mat, cv::Mat> result;
	auto& [imgThre, imgGray] = result;
	cvtColor(img, imgGray, COLOR_BGR2GRAY);
	cvtColor(img, imgThre, COLOR_BGR2HSV);
	// 大符颜色参数还没搞，和灯条混在一起弄了，再列几个参数就行
	if (com.Get().team == ((int)Red ^ IN_STATE(com.Get().flag, STATE_BUFF)))
		inRange(imgThre, Scalar(BHmin, BSmin, BVmin), Scalar(BHmax, BSmax, BVmax), imgThre);
	else { // 红色HSV的H色度刚好分布在色度条两头，得分开inRange然后合并（好麻烦）
		Mat tmp;
		inRange(imgThre, Scalar(0     , RSmin, RVmin), Scalar(RHmaxL, RSmax, RVmax), tmp);
		inRange(imgThre, Scalar(RHminR, RSmin, RVmin), Scalar(180   , RSmax, RVmax), imgThre);
		imgThre |= tmp;
	}
	threshold(imgThre, imgThre, 0, 255, THRESH_BINARY);

	//*///
#if DEBUG_PARA == 0
	static // 非调试模式设置静态内核
#endif
		Mat closeCore = getStructuringElement(MORPH_RECT, Size(closeCoreSize | 1, closeCoreSize | 1));
	morphologyEx(imgThre, imgThre, MORPH_CLOSE, closeCore);
	//*///

	return result;
}
