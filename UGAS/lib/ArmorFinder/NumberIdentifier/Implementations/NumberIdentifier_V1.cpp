#include "NumberIdentifier_V1.h"
#include "Parameters.h"

void NumberIdentifier_V1::init(void*) {}

short NumberIdentifier_V1::Identify(const Img& imgGray, const ArmorPlate& region) {
	static const std::vector<cv::Point2f> dst = { {-4, 9}, {-4, 23}, {36, 23}, {36, 9} };
	cv::Mat M = cv::getPerspectiveTransform(region.points, dst);
	cv::Mat imgWarped, imgNumber;
	cv::warpPerspective(imgGray, imgWarped, M, cv::Size(32, 32));
	cv::threshold(imgWarped, imgNumber, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
	cv::cvtColor(imgNumber, imgNumber, cv::COLOR_GRAY2BGR);
	cv::Point drawPos = static_cast<cv::Point>(region.center());
	cv::Rect drawRect = cv::Rect(drawPos.x - 16, drawPos.y - 16, 32, 32);
	imgNumber.copyTo(debugImg(drawRect));

	return static_cast<short>(NUM_DEFAULT);
}
