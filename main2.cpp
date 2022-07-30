#include <opencv2/opencv.hpp>
#include <iostream>

/**
 *
 * @param KLeft             左相机内参-输入
 * @param KRight            右相机内参-输入
 * @param R                 外参-输入
 * @param T                 外参-输入
 * @param fundamentalMatrix 基本矩阵-输出
 */
void calcFundamentalMat(const cv::Mat &KLeft, const cv::Mat &KRight,
                        const cv::Mat &R, const cv::Mat &T,
                        cv::Mat &fundamentalMatrix) {
    cv::Mat T_ = (cv::Mat_<double>(3, 3)
            <<
            0, -T.at<double>(2), T.at<double>(1),
            T.at<double>(2), 0, -T.at<double>(0),
            -T.at<double>(1), T.at<double>(0), 0
    );

    fundamentalMatrix = KRight.inv().t() * T_ * R * KLeft.inv();
}

/**
 *
 * @param imgLeft           图像1-输入
 * @param imgRight          图像2-输入
 * @param fundamentalMatrix 基本矩阵
 * @param outLeft           绘制好极线的图像1-输出
 * @param outRight          绘制好极线的图像2-输出
 */
void drawLines(const cv::Mat &imgLeft, const cv::Mat &imgRight, const cv::Mat &fundamentalMatrix,
               cv::Mat &outLeft, cv::Mat &outRight) {
    outLeft = imgLeft.clone();
    outRight = imgRight.clone();
    int r = imgLeft.rows, c = imgLeft.cols;

    std::vector<cv::Point2i> ptsLeft, ptsRight;
    std::vector<cv::Vec<float, 3>> epipolarLeft, epipolarRight;

    for (int i = 0; i < r; i += 32) {
        ptsLeft.emplace_back(0, i);
    }
    computeCorrespondEpilines(ptsLeft, 1, fundamentalMatrix, epipolarRight);

    cv::RNG &rng = cv::theRNG();
    std::vector<cv::Scalar> colors;
    for (int i = 0; i < epipolarRight.size(); i++) {
        //随机产生颜色
        cv::Scalar color = cv::Scalar(rng(255), rng(255), rng(255));
        colors.push_back(color);
        ptsRight.emplace_back(0, -epipolarRight[i][2] / epipolarRight[i][1]);
        line(outRight, cv::Point2i(0, -epipolarRight[i][2] / epipolarRight[i][1]),
             cv::Point2i(c, -(epipolarRight[i][2] + epipolarRight[i][0] * c) / epipolarRight[i][1]), color);
    }

    computeCorrespondEpilines(ptsRight, 2, fundamentalMatrix, epipolarLeft);
    for (int i = 0; i < epipolarLeft.size(); i++) {
        //随机产生颜色
        cv::Scalar color = cv::Scalar(rng(255), rng(255), rng(255));
        line(outLeft, cv::Point2i(0, -epipolarLeft[i][2] / epipolarLeft[i][1]),
             cv::Point2i(c, -(epipolarLeft[i][2] + epipolarLeft[i][0] * c) / epipolarLeft[i][1]), colors.at(i));
    }
}

int main() {
    // 相机内参、外参
//    cv::Mat KLeft = (cv::Mat_<double>(3, 3)
//            <<
//            934.712585, 0, 375.183105,
//            0, 903.909607, 290.008118,
//            0, 0, 1
//    );
//
//    cv::Mat KRight = (cv::Mat_<double>(3, 3)
//            <<
//            933.506409, 0, 377.685547,
//            0, 907.118286, 287.696991,
//            0, 0, 1
//    );
//
//    cv::Mat RLW = (cv::Mat_<double>(3, 3)
//            <<
//            0.813460103736338, 0.011701861260389, -0.581503120077059,
//            -0.0758393891170004, 0.993395563285811, -0.0861003203586991,
//            0.576655297506125, 0.114139528943921, 0.808975498109269
//    );
//    cv::Mat TLW = (cv::Mat_<double>(3, 1) << -428.548281037998, -98.2826854831125, 1174.3716279384);
//    cv::Mat RRW = (cv::Mat_<double>(3, 3)
//            <<
//            0.811845287356437, 0.0128144063361871, -0.583732361568873,
//            -0.0750576971136984, 0.993754824513582, -0.082574266601717,
//            0.579028617020513, 0.110851181473674, 0.807737000240559
//    );
//    cv::Mat TRW = (cv::Mat_<double>(3, 1) << -37.3831190794408, -86.9943282002077, 1118.51493368714);


    // 相机内参、外参
//    cv::Mat KLeft = (cv::Mat_<double>(3, 3)
//            <<
//            1326.492432, 0, 224.745819,
//            0, 1884.341675, 240.309418,
//            0, 0, 1
//    );
//
//    cv::Mat KRight = (cv::Mat_<double>(3, 3)
//            <<
//            1338.458008, 0, 170.522949,
//            0, 1900.508301, 231.464188,
//            0, 0, 1
//    );
//
//    cv::Mat RLW = (cv::Mat_<double>(3, 3)
//            <<
//            0.761817893060008, 0.00101648825715559, -0.647790771072016,
//            -0.144060315184242, 0.97522288219469, -0.167889153498386,
//            0.631568857417315, 0.221222041231787, 0.74308847838626
//    );
//    cv::Mat TLW = (cv::Mat_<double>(3, 1) << -125.269687467299, -119.674863345434, 1631.78151476251);
//    cv::Mat RRW = (cv::Mat_<double>(3, 3)
//            <<
//            0.690249031744286, 0.0232326538439334, -0.723198670177138,
//            -0.153252221042246, 0.981503226037658, -0.114740077592588,
//            0.707156459757325, 0.190031616527086, 0.681041945171649
//    );
//    cv::Mat TRW = (cv::Mat_<double>(3, 1) << 25.3442447872248, -106.141357868609, 1757.10579682902);




    cv::Mat KLeft = (cv::Mat_<double>(3, 3)
            <<
            997.765564, 0, 262.475952,
            0, 1425.124756, 268.070984,
            0, 0, 1
    );

    cv::Mat KRight = (cv::Mat_<double>(3, 3)
            <<
            1006.320007, 0, 251.659058,
            0, 1437.093872, 270.067169,
            0, 0, 1
    );

    cv::Mat RLW = (cv::Mat_<double>(3, 3)
            <<
            0.589278434066988, -0.0306653938751527, -0.807348043999232,
            -0.12723238202472, 0.983288049206781, -0.130213701704456,
            0.797849157547487, 0.179452687223572, 0.575528816826891
    );
    cv::Mat TLW = (cv::Mat_<double>(3, 1) << -219.913802801091, -154.420541841717, 1008.55589153764);
    cv::Mat RRW = (cv::Mat_<double>(3, 3)
            <<
            0.579800270681204, -0.0275567742007928, -0.814292178653613,
            -0.131295403730433, 0.983205633609654, -0.126759376205231,
            0.804110598085151, 0.180407408621208, 0.566444557193045
    );
    cv::Mat TRW = (cv::Mat_<double>(3, 1) << -176.407443129471, -147.059620576664, 988.165111294294);


    cv::Mat R = RRW * RLW.t();
    std::cout << "R: " << std::endl << R << std::endl;
    cv::Mat T = TRW - R * TLW;
    std::cout << "T: " << std::endl << T << std::endl;

    cv::Mat fundamentalMatrix;
    calcFundamentalMat(KLeft, KRight, R, T, fundamentalMatrix);
    std::cout << "fundamentalMatrix: " << std::endl << fundamentalMatrix << std::endl;

    // 读取影像
//    std::string pathLeft = R"(..\data\left\Sport0_OG0.bmp)";
//    std::string pathRight = R"(..\data\right\Sport1_OG0.bmp)";
//    std::string pathLeft = R"(..\data\left\Color0_OG0.bmp)";
//    std::string pathRight = R"(..\data\right\Color1_OG0.bmp)";
    std::string pathLeft = R"(..\data\left\Angle0_OG0.bmp)";
    std::string pathRight = R"(..\data\right\Angle1_OG0.bmp)";
    cv::Mat imgLeft = cv::imread(pathLeft, cv::IMREAD_COLOR);
    cv::Mat imgRight = cv::imread(pathRight, cv::IMREAD_COLOR);
    cv::Size imageSize = imgLeft.size();

    // 显示校正前的图像
    cv::Mat mergeBeforeRectify;
    cv::hconcat(imgLeft, imgRight, mergeBeforeRectify);
    for (int i = 0; i < imageSize.height; i += 32) {
        line(mergeBeforeRectify, cv::Point(0, i), cv::Point(2 * imageSize.width, i), cv::Scalar(0, 0, 255), 1, 8);
    }
    cv::imshow("BeforeRectify", mergeBeforeRectify);
    cv::imwrite("../result/BeforeRectify.bmp", mergeBeforeRectify);

    cv::Mat outLeft, outRight, mergeOutBeforeRectify;
    drawLines(imgLeft, imgRight, fundamentalMatrix, outLeft, outRight);
    cv::hconcat(outLeft, outRight, mergeOutBeforeRectify);
    cv::imshow("OutBeforeRectify", mergeOutBeforeRectify);
    cv::imwrite("../result/BeforeRectifyEpipolar.bmp", mergeOutBeforeRectify);

    // 计算旋转矩阵和投影矩阵
    cv::Mat R1, R2, P1, P2, Q, imgLeftRectify, imgRightRectify;
    cv::Rect validPixROI1, validPixROI2;   //左右相机立体修正后有效像素的区域
    cv::stereoRectify(KLeft, cv::Mat::zeros(1, 5, CV_64FC1), KRight, cv::Mat::zeros(1, 5, CV_32F),
                      imageSize, R, T, R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, 1, imageSize,
                      &validPixROI1, &validPixROI2);
    // 计算映射
    cv::Mat rmap[2][2];
    initUndistortRectifyMap(KLeft, cv::Mat::zeros(1, 5, CV_64FC1), R1, P1, imageSize, CV_32FC1, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(KRight, cv::Mat::zeros(1, 5, CV_64FC1), R2, P2, imageSize, CV_32FC1, rmap[1][0],
                            rmap[1][1]);
    remap(imgLeft, imgLeftRectify, rmap[0][0], rmap[0][1], CV_INTER_AREA);//左校正
    remap(imgRight, imgRightRectify, rmap[1][0], rmap[1][1], CV_INTER_AREA);//右校正

    cv::imwrite("../result/left.bmp", imgLeftRectify);
    cv::imwrite("../result/right.bmp", imgRightRectify);

    cv::Mat K = (KLeft + KRight) / 2, mergeOutAfterRectify;
    calcFundamentalMat(K, K,
                       cv::Mat::eye(3, 3, CV_64FC1),
                       (cv::Mat_<double>(3, 1) << -std::sqrt(
                               T.at<double>(0) * T.at<double>(0) +
                               T.at<double>(1) * T.at<double>(1) +
                               T.at<double>(2) * T.at<double>(2)), 0, 0),
                       fundamentalMatrix);
    std::cout << "fundamentalMatrix: " << std::endl << fundamentalMatrix << std::endl;
    drawLines(imgLeftRectify, imgRightRectify, fundamentalMatrix, outLeft, outRight);
    cv::hconcat(outLeft, outRight, mergeOutAfterRectify);
    cv::imshow("OutAfterRectify", mergeOutAfterRectify);
    cv::imwrite("../result/AfterRectifyEpipolar.bmp", mergeOutAfterRectify);

    rectangle(imgLeftRectify, validPixROI1, cv::Scalar(0, 255, 0));
    rectangle(imgRightRectify, validPixROI2, cv::Scalar(0, 255, 0));

    // 显示校正后的图像
    cv::Mat mergeAfterRectify;
    cv::hconcat(imgLeftRectify, imgRightRectify, mergeAfterRectify);
    for (int i = 0; i < imageSize.height; i += 32) {
        line(mergeAfterRectify, cv::Point(0, i), cv::Point(2 * imageSize.width, i), cv::Scalar(0, 0, 255), 1, 8);
    }
    cv::imshow("AfterRectify", mergeAfterRectify);
    cv::imwrite("../result/AfterRectify.bmp", mergeAfterRectify);
    cv::waitKey(0);

    return 0;
}
