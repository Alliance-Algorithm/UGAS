#pragma once
/*
Creation Date: 2023/7/7
Latest Update: 2023/8/4
Developer(s): IrumaMegumi 21-WZY 21-WTR 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- power rune identifier with onnx model from yolo
- running in a new thread
*/

#include <optional>
#include <fstream>
#include <thread>
#include <condition_variable>

#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>

#include "Core/Identifier/Buff/BuffStruct.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Util/Debug/DebugCanvas.h"
#include "Util/UtilFunctions.h"

class BuffIdentifier_V1 {
public:
    explicit BuffIdentifier_V1(const std::string& model_path) {
        ov::Core core;
        compiled_model_ = core.compile_model(model_path, "AUTO");
        if (!compiled_model_)
            throw_with_trace(std::runtime_error, "cannot get compiled model.");
        infer_request_ = compiled_model_.create_infer_request();
    }

    std::optional<BuffPlate> Identify(const cv::Mat& img) {
        cv::Mat letterbox_img = letterbox(img);
        constexpr int letterbox_size = 384; // 352
        double scale = letterbox_img.size[0] / (double)letterbox_size;
        cv::Mat blob = cv::dnn::blobFromImage(letterbox_img, 1.0 / 255.0, cv::Size(letterbox_size, letterbox_size), cv::Scalar(), true);
        auto input_port = compiled_model_.input();
        //std::cout << "step 3 completed" << std::endl;
        // step5:图像输入模型

        ov::Tensor input_tensor(input_port.get_element_type(), input_port.get_shape(), blob.ptr(0));
        // Set input tensor for model with one input
        infer_request_.set_input_tensor(input_tensor);

        // -------- Step 6. Start inference --------
        infer_request_.infer();
        //std::cout << "step 6 completed" << std::endl;
        // -------- Step 7. Get the inference result --------
        auto output = infer_request_.get_output_tensor(0);
        auto output_shape = output.get_shape();
        //std::cout << "The shape of output tensor:" << output_shape << std::endl;
        //[8400,25]，共计8400个anchor，一个anchor里面25个数，分别是x,y,w,h,六个类别的置信度，五组点坐标数据，每组分别为x,y,点置信度

        auto* data = output.data<float>();
        cv::Mat output_buffer((int)output_shape[1], (int)output_shape[2], CV_32F, data);
        transpose(output_buffer, output_buffer);

        float confidence[3] = { .0f,.0f,.0f };//归一化后的置信度
        int confidence_index[3] = { 0,0,0 };//符合要求的行索引位置

        //按和归一化，遍历并找出最大索引对应的anchor
        for (int i = 0; i < output_buffer.rows; ++i) {
            float conf[3];
            for (int j = 4; j < 7; ++j) {
                conf[j - 4] = output_buffer.at<float>(i, j);
            }
            int category = findMaxIndex(conf, 3);
            if (conf[category] > confidence[category]) {
                confidence[category] = conf[category];
                confidence_index[category] = i;
            }
        }

        if (confidence[1] > 0.25) {
            cv::Mat ToShoot = output_buffer.row(confidence_index[1]);
            BuffPlate result;
            auto get_result_point = [&ToShoot, &scale](int i){
                auto x = static_cast<float>(ToShoot.at<float>(0, 7 + i * 3) * scale);
                auto y = static_cast<float>(ToShoot.at<float>(0, 7 + i * 3 + 1) * scale);
                return cv::Point2f(x, y);
            };
            if constexpr (debugCanvas.master) {
                {
                    auto [x, y] = get_result_point(2);
                    cv::circle(debugCanvas.master.GetMat(), cv::Point((int)std::lround(x), (int)std::lround(y)), 8, cv::Scalar{0, 0, 255}, 1);
                }
                {
                    auto [x, y] = get_result_point(1);
                    cv::circle(debugCanvas.master.GetMat(), cv::Point((int)std::lround(x), (int)std::lround(y)), 8, cv::Scalar{0, 255, 255}, 1);
                }
                {
                    auto [x, y] = get_result_point(0);
                    cv::circle(debugCanvas.master.GetMat(), cv::Point((int)std::lround(x), (int)std::lround(y)), 8, cv::Scalar{0, 255, 0}, 1);
                }
                {
                    auto [x, y] = get_result_point(4);
                    cv::circle(debugCanvas.master.GetMat(), cv::Point((int)std::lround(x), (int)std::lround(y)), 8, cv::Scalar{255, 255, 0}, 1);
                }
            }
            result.points.emplace_back(get_result_point(2));
            result.points.emplace_back(get_result_point(1));
            result.points.emplace_back(get_result_point(0));
            result.points.emplace_back(get_result_point(4));
            return result;
        }
        else return {};
    }

private:
    ov::CompiledModel compiled_model_;
    ov::InferRequest infer_request_;

    std::vector<cv::Scalar> colors = { cv::Scalar(255, 0, 0), cv::Scalar(255, 0, 255), cv::Scalar(170, 0, 255), cv::Scalar(255, 0, 85),
                                       cv::Scalar(255, 0, 170) };

    [[nodiscard]] static cv::Mat letterbox(const cv::Mat& source)
    {
        int col = source.cols;
        int row = source.rows;
        int _max = MAX(col, row);
        cv::Mat result = cv::Mat::zeros(_max, _max, CV_8UC3);
        source.copyTo(result(cv::Rect(0, 0, col, row)));
        return result;
    }

    [[nodiscard]] static int findMaxIndex(const float arr[], int size) {
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

    [[nodiscard]] static float findMaxValue(const float arr[], int size) {
        float maxVal = arr[0];

        for (int i = 1; i < size; i++) {
            if (arr[i] > maxVal) {
                maxVal = arr[i];
            }
        }

        return maxVal;
    }
};
