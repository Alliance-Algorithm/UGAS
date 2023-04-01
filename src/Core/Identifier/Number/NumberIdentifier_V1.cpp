#include "NumberIdentifier_V1.h"
#include "Util/Debug/DebugCanvas.h"

using namespace std;
using namespace cv;
using namespace dnn;

NumberIdentifier_V1::NumberIdentifier_V1(const char* model) {
	//加载模型，输入为32*32的二值化图像转灰度图
	_net = cv::dnn::readNetFromTensorflow(model);
	if (_net.empty())
		throw_with_trace(std::runtime_error, "Cannot open model file!");
}

short NumberIdentifier_V1::Identify(const cv::Mat& imgGray, const ArmorPlate& region) {
	// 仿射变换与预处理
	// static const std::vector<Point2f> dst = { {-4, 9}, {-4, 23}, {36, 23}, {36, 9} };
	static const std::vector<Point2f> dst = { {-4.5, 10.125}, {-4.5, 25.875}, {40.5, 25.875}, {40.5, 10.125} };
	Mat imgWarped, imgNumber, imgInput, M = getPerspectiveTransform(region.points, dst);
	//warpPerspective(imgGray, imgWarped, M, Size(32, 32));
	warpPerspective(imgGray, imgWarped, M, Size(36, 36));
	threshold(imgWarped, imgNumber, 0, 255, THRESH_BINARY | THRESH_OTSU);
	// 神经网络预测
	// Mat blobImage = blobFromImage(imgNumber, 1.0, Size(32, 32), false, false);
	Mat blobImage = blobFromImage(imgNumber, 1.0, Size(36, 36), false, false);
	_net.setInput(blobImage);
	Mat pred = _net.forward();
	// 获取预测结果
	double maxVal; Point maxLoc;
	minMaxLoc(pred, NULL, &maxVal, NULL, &maxLoc);
	// std::cout << pred << std::endl;
	if (maxVal < 1.) maxLoc.x = 0;

	if constexpr (debugCanvas.armorNum) {
		// 绘制二值化图像
		cv::cvtColor(imgNumber, imgNumber, cv::COLOR_GRAY2BGR);
		cv::Point drawPos = static_cast<cv::Point>(region.center());
		// cv::Rect drawRect = cv::Rect(drawPos.x - 16, drawPos.y - 48, 32, 32);
		cv::Rect drawRect = cv::Rect(drawPos.x - 18, drawPos.y - 54, 36, 36);
		const auto& tl = drawRect.tl();
		if (tl.x >= 0 && tl.y >= 0) imgNumber.copyTo(debugCanvas.armorNum.GetMat()(drawRect));
		// 绘制识别数字
		cv::putText(debugCanvas.armorNum.GetMat(), std::to_string(maxLoc.x), Point(drawPos.x - 12, drawPos.y + 13), 1, 2.5, COLOR_GREEN, 3);
		// 绘制识别置信度
		// auto maxValStr = std::to_string(maxVal); maxValStr.resize(6);
		// cv::putText(debugCanvas.armorNum.GetMat(), maxValStr, region.points[2], 1, 1.5, COLOR_GREEN, 1.5);
	}

	return maxLoc.x;
}
