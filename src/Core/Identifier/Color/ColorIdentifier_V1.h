#pragma once
/*
Creation Date: 2023/03/29
Latest Update: 2022/03/29
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- ͨ�����������ɫ���Ŷ�
- ��������Ч
- ����Ҫ�ĵ�
*/

#include <opencv2/opencv.hpp>

#include "Core/Identifier/ColorIdentifierInterface.h"
#include "Util/Debug/Log.h"
class ColorIdentifier_V1 {
public:
	ColorIdentifier_V1(float hue360) {
		_minHue = hue360 - 5;
		_maxHue = hue360 + 5;
		_minHueUChar = _minHue / 360 * 255;
		_maxHueUChar = _maxHue / 360 * 255;
		_confidenceMap = GenerateMap();
	}

	ColorConfidence Identify(const uchar* color) const {
		// ���ִ������� https://github.com/egonSchiele/OpenCV/blob/master/modules/imgproc/src/color.cpp
        constexpr int bidx = 0, scn = 3;
		constexpr int hsv_shift = 12;

        static constexpr int div_table[] = {
            0, 1044480, 522240, 348160, 261120, 208896, 174080, 149211,
            130560, 116053, 104448, 94953, 87040, 80345, 74606, 69632,
            65280, 61440, 58027, 54973, 52224, 49737, 47476, 45412,
            43520, 41779, 40172, 38684, 37303, 36017, 34816, 33693,
            32640, 31651, 30720, 29842, 29013, 28229, 27486, 26782,
            26112, 25475, 24869, 24290, 23738, 23211, 22706, 22223,
            21760, 21316, 20890, 20480, 20086, 19707, 19342, 18991,
            18651, 18324, 18008, 17703, 17408, 17123, 16846, 16579,
            16320, 16069, 15825, 15589, 15360, 15137, 14921, 14711,
            14507, 14308, 14115, 13926, 13743, 13565, 13391, 13221,
            13056, 12895, 12738, 12584, 12434, 12288, 12145, 12006,
            11869, 11736, 11605, 11478, 11353, 11231, 11111, 10995,
            10880, 10768, 10658, 10550, 10445, 10341, 10240, 10141,
            10043, 9947, 9854, 9761, 9671, 9582, 9495, 9410,
            9326, 9243, 9162, 9082, 9004, 8927, 8852, 8777,
            8704, 8632, 8561, 8492, 8423, 8356, 8290, 8224,
            8160, 8097, 8034, 7973, 7913, 7853, 7795, 7737,
            7680, 7624, 7569, 7514, 7461, 7408, 7355, 7304,
            7253, 7203, 7154, 7105, 7057, 7010, 6963, 6917,
            6872, 6827, 6782, 6739, 6695, 6653, 6611, 6569,
            6528, 6487, 6447, 6408, 6369, 6330, 6292, 6254,
            6217, 6180, 6144, 6108, 6073, 6037, 6003, 5968,
            5935, 5901, 5868, 5835, 5803, 5771, 5739, 5708,
            5677, 5646, 5615, 5585, 5556, 5526, 5497, 5468,
            5440, 5412, 5384, 5356, 5329, 5302, 5275, 5249,
            5222, 5196, 5171, 5145, 5120, 5095, 5070, 5046,
            5022, 4998, 4974, 4950, 4927, 4904, 4881, 4858,
            4836, 4813, 4791, 4769, 4748, 4726, 4705, 4684,
            4663, 4642, 4622, 4601, 4581, 4561, 4541, 4522,
            4502, 4483, 4464, 4445, 4426, 4407, 4389, 4370,
            4352, 4334, 4316, 4298, 4281, 4263, 4246, 4229,
            4212, 4195, 4178, 4161, 4145, 4128, 4112, 4096
        };
		constexpr int hr = true ? 255 : 180, hscale = hr == 180 ? 15 : 21;

        int b = color[bidx], g = color[1], r = color[bidx ^ 2];
        int h, s, v = b;

		v = v > g ? v : g; v = v > r ? v : r;
		if (v > _minValueUchar) {
			if (color[_cc1] >= 250) {
				return _confidenceMap.at<ColorConfidence>(color[_cc3], color[_cc2]);
			}
			else {
				int vmin = b;
				vmin = vmin < g ? vmin : g; vmin = vmin < r ? vmin : r;
				int diff = v - vmin;

				int vr = v == r ? -1 : 0;
				int vg = v == g ? -1 : 0;

				s = diff * div_table[v] >> hsv_shift;
				if (s > _minSaturationUChar) {
					h = (vr & (g - b)) +
						(~vr & ((vg & (b - r + 2 * diff)) + ((~vg) & (r - g + 4 * diff))));
					h = (h * div_table[diff] * hscale + (1 << (hsv_shift + 6))) >> (7 + hsv_shift);
					h += h < 0 ? hr : 0;
					if (_minHueUChar < h && h < _maxHueUChar)
						return ColorConfidence::CredibleZeroChannelOverexposure;
				}
			}
		}

		return ColorConfidence::NotCredible;		
	}

private:
	// ����˳��
	static constexpr int _cc1 = 0, _cc2 = 1, _cc3 = 2;

    static constexpr float _minSaturation = 0.8, _minValue = 0.5;
	static constexpr uchar _minSaturationUChar = _minSaturation * 255, _minValueUchar = _minValue * 255;
	float _minHue, _maxHue;
	uchar _minHueUChar, _maxHueUChar;

	cv::Mat _confidenceMap;

	cv::Mat GenerateMap() {
		// ��ʱֻ��������ɫ
		constexpr int renderSize = 256;

		cv::Mat imgBGR, imgHSV, imgMap;
		imgBGR.create(renderSize, renderSize, CV_32FC3);
		imgMap.create(renderSize, renderSize, CV_8UC1);

		for (int i = 0; i < renderSize; ++i) {
			for (int j = 0; j < renderSize; ++j) {
				cv::Vec3f color;
				color[_cc1] = 1.0f;
				color[_cc2] = static_cast<float>(i) / renderSize;
				color[_cc3] = static_cast<float>(j) / renderSize;
				imgBGR.at<cv::Vec3f>(j, i) = color;
				imgMap.at<uchar>(j, i) = static_cast<uchar>(ColorConfidence::NotCredible);
			}
		}
		cv::cvtColor(imgBGR, imgHSV, cv::COLOR_BGR2HSV);
		int cornerPointX = 0;
		float maxSlope = 0;
		for (int i = 0; i < renderSize; ++i) {      // x
			for (int j = 0; j < renderSize; ++j) {  // y
				auto& hsv = imgHSV.at<cv::Vec3f>(j, i);
				if (_minHue < hsv[0] && hsv[0] < _maxHue && hsv[1] > _minSaturation && hsv[2] > _minValue) {
					imgMap.at<uchar>(j, i) = static_cast<uchar>(ColorConfidence::CredibleZeroChannelOverexposure);
					if (i > 0) {
						float slope = static_cast<float>(j) / i;
						if (slope > maxSlope) {
							maxSlope = slope;
							cornerPointX = i;
						}
					}
				}
				else imgBGR.at<cv::Vec3f>(j, i) = { 1.0f, 1.0f, 1.0f };
			}
		}
		for (int i = cornerPointX; i < renderSize; ++i) {
			int maxY = maxSlope * i + 0.5;
			for (int j = 0; j <= maxY; ++j) {
				auto& pixel = imgMap.at<uchar>(j, i);
				if (pixel != static_cast<uchar>(ColorConfidence::CredibleZeroChannelOverexposure))
					imgMap.at<uchar>(j, i) = static_cast<uchar>(ColorConfidence::CredibleOneChannelOverexposure);
			}
		}
		for (int j = 0; j < renderSize; ++j) {
			if (j == renderSize - 1)
				imgMap.at<uchar>(j, renderSize - 1) = static_cast<uchar>(ColorConfidence::CredibleThreeChannelOverexposure);
			else
				imgMap.at<uchar>(j, renderSize - 1) = static_cast<uchar>(ColorConfidence::CredibleTwoChannelOverexposure);
		}

		if constexpr (renderSize == 256) {
			return imgMap;
		}
		else {
			auto& imgDisplay = false ? imgBGR : imgMap;
			cv::flip(imgDisplay, imgDisplay, 0);
			cv::imshow("debug", imgDisplay);
			cv::waitKey(0);
			throw_with_trace(std::runtime_error, "render size != 256, enable debugging...");
		}
	}
};