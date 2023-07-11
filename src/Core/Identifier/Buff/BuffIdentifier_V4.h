#pragma once
/*
Creation Date: 2023/6/5
Latest Update: 2023/6/5
Developer(s): IrumaMegumi 21-WZY 21-WTR
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- power rune identifier with onnx model from yolo
- add ROI selection from V3
*/

#include <optional>
#include <fstream>

#include <openvino/openvino.hpp>

#include "Core/Identifier/BuffIdentifierInterface.h"
#include "Util/Debug/DebugCanvas.h"
#include "Util/UtilFunctions.h"


class BuffIdentifier_V4 {
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
    float _size_model, _size_model_ROI;
    ov::CompiledModel _compiled_model, _compiled_model_ROI;
    ov::InferRequest _infer_request, _infer_request_ROI;
    ov::Output<const ov::Node> _input_port, _input_port_ROI;
	std::vector<cv::Scalar> _colors = 
		{ cv::Scalar(255, 0, 0), cv::Scalar(255, 0, 255), cv::Scalar(170, 0, 255),
		  cv::Scalar(255, 0, 85), cv::Scalar(255, 0, 170) };
	//std::ofstream _pdata;
	//std::ofstream _vdata;

	const float _input = 720.0;
    cv::Size2d inputSize = { _input,_input };
	cv::Mat ROIimg = cv::Mat::zeros(_input, _input, CV_8UC3);
	cv::Point2i _lastROIcenter;
	cv::Point2i _LeftUP; // left up point of ROI
	bool _found;

public:
	BuffIdentifier_V4() {
		// read model from file
        _size_model = 640;
        _compiled_model = _core.compile_model("../models/buffsj_4000.onnx", "AUTO");
        _infer_request = _compiled_model.create_infer_request();
    	_input_port = _compiled_model.input();

        _size_model_ROI = 640;
		_compiled_model_ROI = _core.compile_model("../models/buffsj_4000.onnx", "AUTO");
        _infer_request_ROI = _compiled_model_ROI.create_infer_request();
    	_input_port_ROI = _compiled_model_ROI.input();

		_latest_time = -1;
		_last_angle.timeStamp = -1;
		_last_angularSpeed.timeStamp = -1;

		_found = false;

		//_pdata.open("/home/alliance/Desktop/buff_points.txt");
		//_vdata.open("/home/alliance/Desktop/buff_v.txt");
	}

	BuffIdentifier_V4(const BuffIdentifier_V3&) = delete;
	BuffIdentifier_V4(BuffIdentifier_V3&&) = delete;

	std::optional<Buff5PointIdentifyData> Identify(const cv::Mat& img, const TimeStamp& timeStamp, const ArmorColor enemyColor) {
		// return nullopt if onnx has no result
		_buffPlate5Points[4].x = 0;
		_buffRPoint.x = 0;

		// image pretreat
		_LeftUP = getROI(img,ROIimg);

        double scale;
        cv::Mat letterbox_img, blob;
        ov::Tensor input_tensor, output_tensor;
        //std::cout << "_found = " << _found << std::endl;
		if (!_found /*_LeftUP == cv::Point(0, 0)*/) { // no ROI
			letterbox_img = letterbox(img);
            scale = letterbox_img.size[0] / _size_model;
        	blob = cv::dnn::blobFromImage(letterbox_img, 1.0 / 255.0, cv::Size(_size_model, _size_model), cv::Scalar(), true);
			
			// image input
			input_tensor = ov::Tensor(_input_port.get_element_type(), _input_port.get_shape(), blob.ptr(0));
            //std::cout << "Preparing Tensor-640, The shape of input tensor: " << input_tensor.get_shape() << std::endl;

            // set input tensor for model with one input
            _infer_request.set_input_tensor(input_tensor);

            // start inference
            _infer_request.infer();

            // get the inference result
            output_tensor = _infer_request.get_output_tensor(0);
		}
		else { // find ROI
            /*/
            letterbox_img = letterbox(ROIimg); // needn't if ROI has already been a rectangle
            scale = letterbox_img.size[0] / _size_model_ROI;
            blob = cv::dnn::blobFromImage(letterbox_img, 1.0 / 255.0, cv::Size(320, 320), cv::Scalar(), true);
            /*/
            scale = ROIimg.size[0] / _size_model_ROI;
            blob = cv::dnn::blobFromImage(ROIimg, 1.0 / 255.0, cv::Size(_size_model_ROI, _size_model_ROI), cv::Scalar(), true);
			//*/
			// image input
            input_tensor = ov::Tensor(_input_port_ROI.get_element_type(), _input_port_ROI.get_shape(), blob.ptr(0));
            //std::cout << "Preparing Tensor-320, The shape of input tensor: " << input_tensor.get_shape() << std::endl;

            // set input tensor for model with one input
            _infer_request_ROI.set_input_tensor(input_tensor);
            //std::cout << "Tensor inputted." << std::endl;

            // start inference
            _infer_request_ROI.infer();
            //std::cout << "Inferred." << std::endl;

            // get the inference result
            output_tensor = _infer_request_ROI.get_output_tensor(0);
            //std::cout << "Got output tensor." << std::endl;
		}

		auto output_shape = output_tensor.get_shape();
		// [8400, 25] 8400 anchors in total, 25 nums in one anchor (x, y, w, h, confidence, data of points (x, y, confidence))
		//std::cout << "The shape of output tensor: " << output_shape << std::endl;

		// temp start
		float* data = output_tensor.data<float>();
		cv::Mat output_buffer(output_shape[1], output_shape[2], CV_32F, data);
		transpose(output_buffer, output_buffer);
		//std::cout << output_buffer.size() << output_buffer.rows;

		float* target[6] = { 0 };
		float confidence[6];

		for (int i = 0; i < output_buffer.rows; ++i) {
			int type, valid = 0;
			float conf;
            float conf_dest = (enemyColor == ArmorColor::Blue ? 0.6 : 1e-3f);
			for (int j = 4; j < 4 + 6; ++j) {
				conf = output_buffer.at<float>(i, j);
				if (conf > conf_dest) {
					type = j - 4;
					++valid;
				}
			}
			if (valid == 1) {
				if (target[type] == nullptr || conf > confidence[type]) {
					target[type] = &output_buffer.at<float>(i, 0);
					confidence[type] = conf;
				}
			}
		}
		for (int ri = 0; ri < 2; ri++) {
			if (target[ri] != nullptr) {
				float rx = target[ri][0] * scale;
				float ry = target[ri][1] * scale;
				_buffRPoint = cv::Point2f(rx, ry);
                //circle(img, _buffRPoint, 3, _colors[1], -1);
			}
		}
		for (int i = 2; i < 6; ++i) {
			if (target[i] != nullptr) {
				//std::cout << "Type " << i << ": ";
				for (int j = 4; j < 4 + 6; ++j) {
					//std::cout << target[i][j] << ' ';
				}
				//std::cout << '\n';
				if (i == 2 || i == 3) {
					float cx = target[i][0];
					float cy = target[i][1];
					float w = target[i][2];
					float h = target[i][3];
					int left = int((cx - 0.5 * w) * scale);
					int top = int((cy - 0.5 * h) * scale);
					int width = int(w * scale);
					int height = int(h * scale);
					//cv::rectangle(img, cv::Rect(left, top, width, height), cv::Scalar(0, 0, 255), 1);
					//cv::circle(img, cv::Point(int(cx*scale), int(cy*scale)), 3, _colors[i], -1);
					for (int j = 0; j < 5; j++) {
						float x = target[i][10 + j * 3 + 0] * scale;
						float y = target[i][10 + j * 3 + 1] * scale;
						circle(img, cv::Point(x, y), 3, _colors[j], -1);
						_buffPlate5Points[j] = cv::Point2f(x, y);
					}
				}
			}
			else {
				//std::cout << "Type " << i << ": Not found.\n";
			}
		}
//		cv::namedWindow("YOLOv8-Pose OpenVINO Inference C++ Demo", cv::WINDOW_AUTOSIZE);
//		cv::imshow("YOLOv8-Pose OpenVINO Inference C++ Demo", img);
//		cv::waitKey(1);

		if (_buffPlate5Points[4].x == 0) {
			_found = false;
			return std::nullopt; 
		}
		if (_buffRPoint.x == 0 || _buffRPoint.y == 0) {
			_found = false;
			return std::nullopt;
		}

        if constexpr (debugCanvas.buff) {
            cv::circle(ROIimg, _buffRPoint, 3, COLOR_RED, 2, 8);
            cv::circle(ROIimg, _buffPlate5Points[0], 3, COLOR_RED, 2, 8);
            cv::circle(ROIimg, _buffPlate5Points[1], 3, COLOR_RED, 2, 8);
            cv::circle(ROIimg, _buffPlate5Points[2], 3, COLOR_RED, 2, 8);
            cv::circle(ROIimg, _buffPlate5Points[3], 3, COLOR_RED, 2, 8);
            cv::circle(ROIimg, _buffPlate5Points[4], 3, COLOR_RED, 2, 8);
            cv::imshow("ROI", ROIimg);
            cv::waitKey(1);
        }

		_buffRPoint.x += _LeftUP.x;
		_buffRPoint.y += _LeftUP.y;
		for (int i = 0; i < 5; i++) {
			_buffPlate5Points[i].x += _LeftUP.x;
			_buffPlate5Points[i].y += _LeftUP.y;
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
		_found = true;
		_lastROIcenter = _buffPlate;
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
            _last_angularSpeed.timeStamp = _last_angle.timeStamp=_latest_time;
            _last_angle.angle = angle;
            _last_angularSpeed.speed = 0;

            return std::nullopt;
		}
	}

	cv::Mat letterbox(const cv::Mat& source) {
		int col = source.cols;
		int row = source.rows;
		int _max = MAX(col, row);
		cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
		source.copyTo(result(cv::Rect(0, 0, col, row)));
		return result;
	}

	cv::Point2i getROI(const cv::Mat& source, cv::Mat& ROIimg) {
		if (!_found)
		{
			return cv::Point2i(0,0);
		}

		//处理X越界
		if (_lastROIcenter.x <= inputSize.width / 2)
			_lastROIcenter.x = inputSize.width / 2;
		else if (_lastROIcenter.x > (source.size().width - inputSize.width / 2))
			_lastROIcenter.x = source.size().width - inputSize.width / 2;
		//处理Y越界
		if (_lastROIcenter.y <= inputSize.height / 2)
			_lastROIcenter.y = inputSize.height / 2;
		else if (_lastROIcenter.y > (source.size().height - inputSize.height / 2))
			_lastROIcenter.y = source.size().height - inputSize.height / 2;

		//左上角顶点
		auto offset = _lastROIcenter - cv::Point2i(inputSize.width / 2, inputSize.height / 2);
		cv::Rect roi_rect = cv::Rect(offset, inputSize);
		source(roi_rect).copyTo(ROIimg);

		return offset;
	}

};

