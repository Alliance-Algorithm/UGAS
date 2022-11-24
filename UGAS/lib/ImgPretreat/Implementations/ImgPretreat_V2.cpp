#include "ImgPretreat_V2.h"

inline void ImgPretreat_V2::LoopPixel(const uchar* src, uchar* dst, int n) const {
    static const int div_table[] = {
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
    static const int hueRange = 180;
    static const int hueScale = (hueRange == 180 ? 15 : 21);
    static const int hsv_shift = 12;

    for (int i = 0; i < n; i += 1, src += 3) {
        int b = src[0], g = src[1], r = src[2];
        int hue, vmax = b, vmin = b, diff, vr, vg;

        vmax = vmax > g ? vmax : g; vmax = vmax > r ? vmax : r;
        vmin = vmin < g ? vmin : g; vmin = vmin < r ? vmin : r;

        diff = vmax - vmin;
        int light = (vmin + vmax) >> 1;

        if (diff > 10 && light > 110) {
            vr = vmax == r ? -1 : 0;
            vg = vmax == g ? -1 : 0;

            hue = (vr & (g - b)) +
                (~vr & ((vg & (b - r + 2 * diff)) + ((~vg) & (r - g + 4 * diff))));
            hue = (hue * div_table[diff] * hueScale + (1 << (hsv_shift + 6))) >> (7 + hsv_shift);
            hue += hue < 0 ? hueRange : 0;

            if (com.Get().team == ((int)Red ^ IN_STATE(com.Get().flag, STATE_BUFF)))
                dst[i] = (hue > BHmin && hue < BHmax) ? (uchar)255 : (uchar)0;
            else dst[i] = (hue > RHminR || hue < RHmaxL) ? (uchar)255 : (uchar)0;
        }
        else dst[i] = 0;
    }
}

inline void ImgPretreat_V2::Threshold(const cv::Mat& src, cv::Mat& dst) const {
    cv::Size sz = src.size();
    const uchar* srcBuf = src.data;
    uchar* dstBuf = dst.data;
    size_t srcstep = src.step, dststep = dst.step;

    if (src.isContinuous() && dst.isContinuous()) {
        sz.width *= sz.height;
        sz.height = 1;
    }

    for (; sz.height--; srcBuf += srcstep, dstBuf += dststep)
        LoopPixel(srcBuf, dstBuf, sz.width);
}

void ImgPretreat_V2::GetPretreated(const cv::Mat& img, cv::Mat& imgThre, cv::Mat& imgGray) {
	if (imgThre.empty())
        imgThre.create(img.size(), CV_8UC1);
	Threshold(img, imgThre);
    cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

#if DEBUG_PARA == 0
    static // 非调试模式设置静态内核
#endif
        cv::Mat closeCore = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(closeCoreSize | 1, closeCoreSize | 1));
    morphologyEx(imgThre, imgThre, cv::MORPH_CLOSE, closeCore);
}
