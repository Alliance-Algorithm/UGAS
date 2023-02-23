#include "NumberIdentifier_V1.h"
#include <Common/DebugTools/DebugHeader.h>
#include <Parameters.h>
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
	// 仿射变换与预处理
	static const std::vector<Point2f> dst = { {-4, 9}, {-4, 23}, {36, 23}, {36, 9} };
	Mat imgWarped, imgNumber, M = getPerspectiveTransform(region.OffsetPoints(-ROIoffset), dst);
	warpPerspective(imgGray, imgWarped, M, Size(32, 32));
	threshold(imgWarped, imgNumber, 0, 255, THRESH_BINARY | THRESH_OTSU);
	// 神经网络预测
	normalize(imgNumber, imgNumber, 1, 0, NORM_MINMAX);
	Mat blobImage = blobFromImage(imgNumber, 1.0, Size(32, 32), false, false);
	_net.setInput(blobImage);
	Mat pred = _net.forward();
	// 获取预测结果
	double maxVal; Point maxLoc;
	minMaxLoc(pred, NULL, &maxVal, NULL, &maxLoc);
	if (maxVal < 1.) maxLoc.x = 0;

#if DEBUG_IMG == 1 && DEBUG_ARMOR_NUM == 1
	// 绘制二值化图像
	cv::cvtColor(imgNumber, imgNumber, cv::COLOR_GRAY2BGR);
	cv::Point drawPos = static_cast<cv::Point>(region.center());
	cv::Rect drawRect = cv::Rect(drawPos.x - 16, drawPos.y - 16, 32, 32);
	imgNumber.copyTo(debugImg(drawRect));
	// 绘制识别数字
	cv::putText(debugImg, std::to_string(maxLoc.x), Point(drawPos.x - 12, drawPos.y + 13), 1, 2.5, COLOR_GREEN, 3);
	// 绘制识别置信度
	auto maxValStr = std::to_string(maxVal); maxValStr.resize(6);
	cv::putText(debugImg, maxValStr, region.points[0], 1, 1.5, COLOR_GREEN, 1.5);
#endif

	return maxLoc.x;
}
