#include "NumberIdentifier_V1.h"
#include "Util/Debug/DebugCanvas.h"

using namespace std;
using namespace cv;
using namespace dnn;

NumberIdentifier_V1::NumberIdentifier_V1(const char* model) {
    //加载模型，输入为36*36的二值化图像转灰度图
    _net = cv::dnn::readNetFromTensorflow(model);
    if (_net.empty())
        throw_with_trace(std::runtime_error, "Cannot open model file!");
    for(int i=1;i<11;i++)
    {
        std::string path="./templates/"+ std::to_string(i) + ".png";
        cv::Mat img=imread(path);
        cvtColor(img,img,COLOR_BGR2GRAY);
        if(!img.empty())
        {
            cv::resize(img,img,Size(36,36));
            threshold(img,img, 0, 255, THRESH_BINARY | THRESH_OTSU);
            _templateFigures.push_back(img);
        }
        else
        {
            throw_with_trace(std::runtime_error, "Template figure does not exist");
        }
    }
}

bool NumberIdentifier_V1::Identify(const cv::Mat& imgGray, ArmorPlate& armor) {
    // 仿射变换与预处理
    // static const std::vector<Point2f> dst = { {-4, 9}, {-4, 23}, {36, 23}, {36, 9} };
    static const std::vector<Point2f> dst = { {-4.5, 10.125}, {-4.5, 25.875}, {40.5, 25.875}, {40.5, 10.125} };
    Mat imgWarped, imgNumber, imgInput, M = getPerspectiveTransform(armor.points, dst);
    //warpPerspective(imgGray, imgWarped, M, Size(32, 32));
    warpPerspective(imgGray, imgWarped, M, Size(36, 36));
    threshold(imgWarped, imgNumber, 0, 255, THRESH_BINARY | THRESH_OTSU);
    // 神经网络预测
    Mat blobImage = blobFromImage(imgNumber, 1.0, Size(36, 36), false, false);
    _net.setInput(blobImage);
    Mat pred = _net.forward();
    // 获取预测结果
    double maxVal; Point maxLoc;
    minMaxLoc(pred, NULL, &maxVal, NULL, &maxLoc);
    // std::cout << pred << std::endl;
    // if (maxVal < 1.) maxLoc.x = 0;

    //进行正负样本筛查
    cv::Mat resultMatrix;
    cv::matchTemplate(imgNumber, _templateFigures[maxLoc.x-1], resultMatrix, TM_CCOEFF_NORMED);
    double score = 0;
	minMaxLoc(resultMatrix, NULL, &score, NULL, NULL);
    if(score>0.3)
    {
        maxLoc.x=maxLoc.x;
    }
    else
    {
        maxLoc.x=0;
    }
    
    switch (maxLoc.x) {
    case 0:
        return false;
    case 1:
        armor.id = ArmorID::Hero;
        armor.is_large_armor = true;
        break;
    case 2:
        armor.id = ArmorID::Engineer;
        armor.is_large_armor = false;
        break;
    case 3:
        armor.id = ArmorID::InfantryIII;
        armor.is_large_armor = false;
        break;
    case 4:
        armor.id = ArmorID::InfantryIV;
        armor.is_large_armor = false;
        break;
    case 5:
        armor.id = ArmorID::InfantryV;
        armor.is_large_armor = false;
        break;
    case 6:
        armor.id = ArmorID::Sentry;
        armor.is_large_armor = false;
        break;
    case 7:
        armor.id = ArmorID::InfantryIII;
        armor.is_large_armor = true;
    case 8:
        armor.id = ArmorID::InfantryIV;
        armor.is_large_armor = true;
        break;
    case 9:
        armor.id = ArmorID::InfantryV;
        armor.is_large_armor = true;
        break;
    case 10:
        armor.id=ArmorID::Outpost;
        armor.is_large_armor=false;
        break;
    }

    if constexpr (debugCanvas.armorNum) {
        // 绘制二值化图像
        cv::cvtColor(imgNumber, imgNumber, cv::COLOR_GRAY2BGR);
        cv::Point drawPos = static_cast<cv::Point>(armor.center());
        // cv::Rect drawRect = cv::Rect(drawPos.x - 16, drawPos.y - 48, 32, 32);
        cv::Rect drawRect = cv::Rect(drawPos.x - 18, drawPos.y - 54, 36, 36);
        const auto& tl = drawRect.tl();
        if (tl.x >= 0 && tl.y >= 0) imgNumber.copyTo(debugCanvas.armorNum.GetMat()(drawRect));
        // 绘制识别数字
        cv::putText(debugCanvas.armorNum.GetMat(), std::to_string(maxLoc.x), Point(drawPos.x - 12, drawPos.y + 13), 1, 2.5, COLOR_GREEN, 3);
        // 绘制识别置信度
        // auto maxValStr = std::to_string(maxVal); maxValStr.resize(6);
        // cv::putText(debugCanvas.armorNum.GetMat(), maxValStr, armor.points[2], 1, 1.5, COLOR_GREEN, 1.5);
    }

    return true;
}
