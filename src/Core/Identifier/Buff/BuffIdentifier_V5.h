#pragma once
/*
Creation Date: 2023/7/2
Latest Update: 2023/7/2
Developer(s): IrumaMegumi 21-WZY 21-WTR
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- power rune identifier with onnx model from yolo
- no longer distinguish between red or blue
*/

#include <optional>
#include <fstream>

#include <openvino/openvino.hpp>

#include "Core/Identifier/BuffIdentifierInterface.h"
#include "Util/Debug/DebugCanvas.h"
#include "Util/UtilFunctions.h"


class BuffIdentifier_V5 {
private:
	BuffAngle _last_angle;
	BuffAngularSpeed _last_angularSpeed;

	TimeStamp _latest_time;
	cv::Point2f _buffPlate;
	cv::Point2f _rCenter;
	double _radius;

    cv::Point2f _buffRPoint;
    cv::Point2f _buffPlate5Points[5];

	ov::Core _core;
    float _size_model;
    ov::CompiledModel _compiled_model;
    ov::InferRequest _infer_request;
    ov::Output<const ov::Node> _input_port;
	std::vector<cv::Scalar> _colors =
        { cv::Scalar(255, 0, 0), cv::Scalar(255, 0, 255), cv::Scalar(170, 0, 255),
          cv::Scalar(255, 0, 85), cv::Scalar(255, 0, 170) };
	//std::ofstream _pdata;
	//std::ofstream _vdata;

	cv::Mat letterbox(const cv::Mat& source) {
		int col = source.cols;
		int row = source.rows;
		int _max = MAX(col, row);
		cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
		source.copyTo(result(cv::Rect(0, 0, col, row)));
		return result;
    }

	void normalizeArray(float arr[], int size) {
	// 按和归一化函数，不到万不得已不要使用
	
		// 计算数组元素的总和
		float sum = 0.0;
		for (int i = 0; i < size; i++) {
			sum += arr[i];
		}
		// 归一化数组元素
		for (int i = 0; i < size; i++)  {
			arr[i] /= sum;
		}
	}

	int findMaxIndex(float arr[], int size) {
	// 寻找最大值索引函数
	
		float maxVal = arr[0];
		int maxIndex = 0;
		for (int i = 1; i < size; i++) {
			if (arr[i] > maxVal) {
				maxVal = arr[i];
				maxIndex = i;
			}
		}
		return maxIndex;
	}

public:
	BuffIdentifier_V5() {
		// read model from file
        _size_model = 512;
        _compiled_model = _core.compile_model("../models/buff_nocolor_v2.onnx", "AUTO");
        _infer_request = _compiled_model.create_infer_request();
        _input_port = _compiled_model.input();

		_latest_time = -1;
		_last_angle.timeStamp = -1;
		_last_angularSpeed.timeStamp = -1;

		//_pdata.open("/home/alliance/Desktop/buff_points.txt");
		//_vdata.open("/home/alliance/Desktop/buff_v.txt");
	}

	BuffIdentifier_V5(const BuffIdentifier_V3&) = delete;
    BuffIdentifier_V5(BuffIdentifier_V3&&) = delete;

	std::optional<Buff5PointIdentifyData> Identify(const cv::Mat& img, const TimeStamp& timeStamp, const ArmorColor enemyColor) {
        // return nullopt if onnx has no result
        _buffPlate5Points[0].x = 0;
		_buffRPoint.x = 0;

		// image pretreat
        cv::Mat letterbox_img = letterbox(img);
        float scale = letterbox_img.size[0] / _size_model;

        cv::Mat blob;
        blob = cv::dnn::blobFromImage(letterbox_img, 1.0 / 255.0, cv::Size(_size_model, _size_model), cv::Scalar(), true);

		// image input
		ov::Tensor input_tensor(_input_port.get_element_type(), _input_port.get_shape(), blob.ptr(0));

        // set input tensor for model with one input
		_infer_request.set_input_tensor(input_tensor);

		// start inference
		_infer_request.infer();

		// get the inference result
		auto output = _infer_request.get_output_tensor(0);
		auto output_shape = output.get_shape();
		// [8400, 25] 8400 anchors in total, 25 nums in one anchor (x, y, w, h, confidence, data of points (x, y, confidence))
		//std::cout << "The shape of output tensor: " << output_shape << std::endl;

		// temp start
		float* data = output.data<float>();
		cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F, data);
		transpose(output_buffer, output_buffer);
		//std::cout << output_buffer.size() << output_buffer.rows;

		float* target[6] = { 0 };
		float confidence[3] = {.0f, .0f, .0f};
		int confidence_index[3]; // 得分最大的R, ToShoot, Shot三个类别所在的行数

		for (int i = 0; i < output_buffer.rows; ++i)
		{
			float conf[3];
			for (int j = 4; j < 7; ++j)
			{
				conf[j - 4] = output_buffer.at<float>(i, j);
			}
            //不到万不得已别用按和归一化函数
			/*int num = findMaxIndex(conf, 3);
			if (num == 0)
			{
				normalizeArray(conf, 3);
			}*/
			for (int ind = 4; ind < 7; ind++)
			{
				output_buffer.at<float>(i, ind) = conf[ind - 4];
			}
			int category = findMaxIndex(conf, 3);
			if(conf[category] > confidence[category])
			{
				confidence[category] = conf[category];
				confidence_index[category] = i;
            }
		}

		cv::Mat R = output_buffer.row(confidence_index[0]);
		cv::Mat ToShoot = output_buffer.row(confidence_index[1]);
        std::cout << "R conf: " << confidence[0] << std::endl;
        std::cout << "ToShoot conf: " << confidence[1] << std::endl;

		//获取R的中心并框出
		float cxR = R.at<float>(0, 0) * scale;
		float cyR = R.at<float>(0, 1) * scale;
		float wR = R.at<float>(0, 2) * scale;
		float hR = R.at<float>(0, 3) * scale;
		float leftR = cxR - 0.5f * wR;
		float topR = cyR - 0.5f * hR;
		float widthR = wR;
		float heightR = hR;
		_buffRPoint = cv::Point2f(cxR, cyR);
		cv::rectangle(img, cv::Rect(leftR,topR,widthR,heightR), cv::Scalar(0, 0, 255), 1);

        if (confidence[1] > 0.25)
        {
            //获取击打点的二维坐标，把能量机关轮廓框出
            float cx = ToShoot.at<float>(0, 0);
            float cy = ToShoot.at<float>(0, 1);
            float w = ToShoot.at<float>(0, 2);
            float h = ToShoot.at<float>(0, 3);
            float left = (cx - 0.5 * w) * scale;
            float top = (cy - 0.5 * h) * scale;
            float width = w * scale;
            float height = h * scale;
            cv::rectangle(img, cv::Rect(left, top, width, height), cv::Scalar(0, 0, 255), 1);
            for (int j = 0; j < 5; j++) {
                float x = ToShoot.at<float>(0, 7 + j * 3) * scale;
                float y = ToShoot.at<float>(0, 7 + j * 3 + 1) * scale;
                circle(img, cv::Point(x, y), 3, _colors[j], -1);
                _buffPlate5Points[j] = cv::Point2f(x, y);
            }
        }
//		cv::namedWindow("YOLOv8-Pose OpenVINO Inference C++ Demo", cv::WINDOW_AUTOSIZE);
//		cv::imshow("YOLOv8-Pose OpenVINO Inference C++ Demo", img);
//		cv::waitKey(1);

		if (_buffPlate5Points[0].x == 0) {
			return std::nullopt; 
		}
		if (_buffRPoint.x == 0 || _buffRPoint.y == 0) {
			return std::nullopt;
		}

		// output buff points data
//		_pdata << "time:" << timeStamp << std::endl;
//		_pdata << "0: " << _buffPlate5Points[0].x << " " << _buffPlate5Points[0].y << std::endl;
//		_pdata << "1: " << _buffPlate5Points[1].x << " " << _buffPlate5Points[1].y << std::endl;
//		_pdata << "2: " << _buffPlate5Points[2].x << " " << _buffPlate5Points[2].y << std::endl;
//		_pdata << "3: " << _buffPlate5Points[3].x << " " << _buffPlate5Points[3].y << std::endl;
//		_pdata << "4: " << _buffPlate5Points[4].x << " " << _buffPlate5Points[4].y << std::endl;
//		_pdata << "R: " << _buffRPoint.x << " " << _buffRPoint.y << std::endl;
//		_pdata << std::endl << std::endl << std::endl;

		_latest_time = timeStamp;
		double v = 0, angle = 0;
		_rCenter = _buffRPoint;
		angle = atan2(_buffPlate5Points[0].y + _buffPlate5Points[1].y - 2 * _rCenter.y,
					  _buffPlate5Points[0].x + _buffPlate5Points[1].x - 2 * _rCenter.x);
		_buffPlate = (	_buffPlate5Points[0] + _buffPlate5Points[1] +
						_buffPlate5Points[2] + _buffPlate5Points[4]) / 4;
		_radius = P2PDis(_buffPlate, _rCenter);
		//std::cout << angle << std::endl;
		//std::cout << _radius << std::endl;
		//std::cout << _last_angle.timeStamp << std::endl;

		if (_last_angle.timeStamp != -1) {
			double delta_angle = angle - _last_angle.angle;
			if (delta_angle > 6.28) delta_angle -= 2 * MathConsts::Pi;
			else if (delta_angle < -6.28) delta_angle += 2 * MathConsts::Pi;
			//std::cout << _latest_time << "  " << delta_angle << std::endl;

			TimeStamp delta_time = _latest_time - _last_angle.timeStamp;
			if (delta_time == 0) return std::nullopt;

			v = 1000 * delta_angle / delta_time;
			//std::cout << "time: " << delta_time << "\t" << "angular speed: " << delta_angle << std::endl;

			_last_angle = BuffAngle(_latest_time, angle);

			_last_angularSpeed = BuffAngularSpeed(_latest_time, v);

			//_vdata << _latest_time << "   " << valid_v << std::endl;

            if constexpr (debugCanvas.buff) {
                cv::circle(debugCanvas.buff.GetMat(), _rCenter, 3, COLOR_RED, 2, 8);
                cv::circle(debugCanvas.buff.GetMat(), _buffPlate5Points[0], 3, COLOR_RED, 2, 8);
                cv::circle(debugCanvas.buff.GetMat(), _buffPlate5Points[1], 3, COLOR_RED, 2, 8);
                cv::circle(debugCanvas.buff.GetMat(), _buffPlate5Points[2], 3, COLOR_RED, 2, 8);
                cv::circle(debugCanvas.buff.GetMat(), _buffPlate5Points[3], 3, COLOR_RED, 2, 8);
                cv::circle(debugCanvas.buff.GetMat(), _buffPlate5Points[4], 3, COLOR_RED, 2, 8);
            }

			return Buff5PointIdentifyData(_buffPlate5Points[0], _buffPlate5Points[1],
				_buffPlate5Points[2], _buffPlate5Points[4],
				_rCenter, _radius, _last_angle, _last_angularSpeed);
		}
		else {
            _last_angularSpeed.timeStamp = _last_angle.timeStamp = _latest_time;
            _last_angle.angle = angle;
            _last_angularSpeed.speed = 0;

            return std::nullopt;
		}
	}
};
