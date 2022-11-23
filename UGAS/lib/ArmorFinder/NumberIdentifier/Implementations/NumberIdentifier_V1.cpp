#include "NumberIdentifier_V1.h"
#include "Common/DebugTools/DebugHeader.h"
using namespace std;
using namespace cv;
using namespace dnn;

void NumberIdentifier_V1::init(void* model) {
	//加载模型，输入为32*32的二值化图像转灰度图
	_net = cv::dnn::readNetFromTensorflow(
		string("lib/ArmorFinder/NumberIdentifier/Implementations/") + 
		*static_cast<const char**>(model)
	);
	if (_net.empty())
		throw_with_trace(std::runtime_error, "Cannot open model file!");
}

short NumberIdentifier_V1::Identify(const Img& imgGray, const ArmorPlate& region) {
	static const std::vector<cv::Point2f> dst = { {-4, 9}, {-4, 23}, {36, 23}, {36, 9} };
	cv::Mat M = cv::getPerspectiveTransform(region.points, dst);
	cv::Mat imgWarped, imgNumber;
	cv::warpPerspective(imgGray, imgWarped, M, cv::Size(32, 32));
	cv::threshold(imgWarped, imgNumber, 0, 255, cv::THRESH_BINARY + cv::THRESH_OTSU);
	//cv::resize(imgNumber, imgNumber, cv::Size(36, 36));
	Mat blobImage = dnn::blobFromImage(imgNumber, 1.0, Size(32, 32), false, false);
	_net.setInput(blobImage);
	Mat pred = _net.forward();
	std::cout << pred << std::endl;
	Point maxLoc;
	double maxVal;
	minMaxLoc(pred, &maxVal, NULL, NULL, &maxLoc);

#if true
	cv::cvtColor(imgNumber, imgNumber, cv::COLOR_GRAY2BGR);
	cv::Point drawPos = static_cast<cv::Point>(region.center());
	cv::Rect drawRect = cv::Rect(drawPos.x - 16, drawPos.y - 16, 32, 32);
	imgNumber.copyTo(debugImg(drawRect));
	cv::putText(debugImg, std::to_string(maxVal), region.points[0], 1, 2, COLOR_GREEN);
	cv::putText(debugImg, std::to_string(maxLoc.x), region.points[2], 1, 5, COLOR_GREEN);
#endif

	return maxLoc.x;
}
