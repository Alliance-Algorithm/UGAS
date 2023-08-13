#include "Parameters.h"
//using namespace cv;

namespace parameters {
    /******** Universal Parameter Area ********/

    const ArmorColor DefaultEnemyColor = ArmorColor::Blue;

    const double maxArmorLightRatio = 1.5, maxdAngle = 9.5, maxMalposition = 0.7, maxLightDy = 0.9, bigArmorDis = 5.0;

    // pnp parameters
    namespace camera_calibration_data {
        // hik 6mm
        double CameraMatrixData6[3][3] = {
            {1.722231837421459e+03, 0, 7.013056440882832e+02},
            {0, 1.724876404292754e+03, 5.645821718351237e+02},
            {0, 0, 1}
        };
        double CameraDistCoeffsData6[5] = {-0.064232403853946, -0.087667493884102, 0, 0, 0.792381808294582};

        // hik 8mm
        double CameraMatrixData8[3][3] = {
            {2411.67435752361, 0, 706.115296931680},
            {0, 2413.93326236200, 606.089691712260},
            {0, 0, 1}
        };
        double CameraDistCoeffsData8[5] = {-0.0299345399861973, 0.127101646352983, 0, 0, 0.155503759356591};

        // hik 12mm
        double CameraMatrixData12[3][3] = {
            {3612.09388868374, 0, 713.350996606441},
            {0, 3615.26346146138, 590.928598586120},
            {0, 0, 1}
        };
        double CameraDistCoeffsData12[5] = { 0.00948033127678667, -0.900070016384203, 0, 0, 12.9708921407335 };

        // dji image transmitter camera
        double TransmitterCameraMatrixData[3][3] = {
                {870.536594077599, 0, 959.879173875982},
                {0, 871.00281111889, 554.055610210946},
                {0, 0, 1}
        };
        double TransmitterCameraDistCoeffsData[5] = {-0.285400532140372, 0.106341621768377, 0, 0, -0.0203255154424868};
    }
    constexpr double NormalArmorWidth = 134, NormalArmorHeight = 56, LargerArmorWidth = 230, LargerArmorHeight = 56;
    const std::vector<cv::Point3d> NormalArmorObjectPoints = {
            cv::Point3d(-0.5 * NormalArmorWidth,  0.5 * NormalArmorHeight, 0.0f),
            cv::Point3d(-0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight, 0.0f),
            cv::Point3d( 0.5 * NormalArmorWidth, -0.5 * NormalArmorHeight, 0.0f),
            cv::Point3d( 0.5 * NormalArmorWidth,  0.5 * NormalArmorHeight, 0.0f)
    };
    const std::vector<cv::Point3d> LargeArmorObjectPoints = {
            cv::Point3d(-0.5 * LargerArmorWidth,  0.5 * LargerArmorHeight, 0.0f),
            cv::Point3d(-0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight, 0.0f),
            cv::Point3d( 0.5 * LargerArmorWidth, -0.5 * LargerArmorHeight, 0.0f),
            cv::Point3d( 0.5 * LargerArmorWidth,  0.5 * LargerArmorHeight, 0.0f)
    };
    constexpr double BuffPlateHeightL = 372, BuffPlateHeightR = 320, BuffPlateWidth = 317;
    const std::vector<cv::Point3d> BuffObjectPoints = {
            cv::Point3d{0, -0.5 * BuffPlateHeightL, -0.5 * BuffPlateWidth},
            cv::Point3d{0, -0.5 * BuffPlateHeightR,  0.5 * BuffPlateWidth},
            cv::Point3d{0,  0.5 * BuffPlateHeightR,  0.5 * BuffPlateWidth},
            cv::Point3d{0,  0.5 * BuffPlateHeightL, -0.5 * BuffPlateWidth},
    };
    const cv::Mat TransmitterCameraMatrix(3, 3, CV_64F, camera_calibration_data::TransmitterCameraMatrixData);
    const cv::Mat TransmitterCameraDistCoeffs(1, 5, CV_64F, camera_calibration_data::TransmitterCameraDistCoeffsData);

    const double DefaultBulletSpeed = 8.0;
}

// enable specialized parameter

#include "Util/Parameter/Specialize/BalancedInfantry.h"
//#include "Util/Parameter/Specialize/McknumWheelInfantry.h"
//#include "Util/Parameter/Specialize/TopFeedingOmniWheelInfantry.h"
//#include "Util/Parameter/Specialize/Uav.h"
