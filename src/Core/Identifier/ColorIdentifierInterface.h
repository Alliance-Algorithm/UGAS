#pragma once
/*
Creation Date: 2023/03/29
Latest Update: 2022/03/29
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 对单个像素点进行颜色识别的接口
- 计算出颜色置信度（不可信 / 可信-过曝0个通道 / 可信-过曝1个通道 / 可信-过曝2个通道 / 可信-过曝3个通道）
*/


enum class ColorConfidence : uchar {
	NotCredible = 0,
	CredibleZeroChannelOverexposure = 255,
	CredibleOneChannelOverexposure = 191,
	CredibleTwoChannelOverexposure = 127,
	CredibleThreeChannelOverexposure = 63
};


// 由于性能要求，ColorIdentifier::Identify必须是内联的，该接口仅作声明，不需继承
class ColorIdentifierInterface final {
public:
	virtual ~ColorIdentifierInterface() = default;

	virtual ColorConfidence Identify(cv::Vec3b color) const = 0;
};
