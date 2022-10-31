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
    cv::Mat KLeft = (cv::Mat_<double>(3, 3)
            <<
            555.80785, 0, 330.17268,
            0, 557.34962, 244.70437,
            0, 0, 1
    );
    cv::Mat DLeft = (cv::Mat_<double>(5, 1)
            << -0.21713, 0.12312, 0.00487, 0.00163, 0.0);
    cv::Mat KRight = (cv::Mat_<double>(3, 3)
            <<
            558.11555, 0, 314.72552,
            0, 559.49020, 232.64247,
            0, 0, 1
    );
    cv::Mat DRight = (cv::Mat_<double>(5, 1)
            << -0.22750, 0.14968, 0.00352, 0.00058, 0.0);

    cv::Mat om = (cv::Mat_<double>(3, 1)
            << 0.00189, -0.00040, 0.00653);
    cv::Mat R;
    cv::Rodrigues(om, R);
    std::cout << "R: " << std::endl << R << std::endl;
    cv::Mat T = (cv::Mat_<double>(3, 1) << -89.51204, -0.21901, -0.28905);

    cv::Mat fundamentalMatrix;
    calcFundamentalMat(KLeft, KRight, R, T, fundamentalMatrix);
    std::cout << "fundamentalMatrix: " << std::endl << fundamentalMatrix << std::endl;

    cv::Size imageSize = cv::Size(640, 480);

    // 读取影像
    std::string pathLeft = R"(..\data\left\Left0103.bmp)";
    std::string pathRight = R"(..\data\right\Right0103.bmp)";
    cv::Mat imgLeft = cv::imread(pathLeft, cv::IMREAD_COLOR);
    cv::Mat imgRight = cv::imread(pathRight, cv::IMREAD_COLOR);

    // 显示校正前的图像
    cv::Mat mergeBeforeRectify;
    cv::hconcat(imgLeft, imgRight, mergeBeforeRectify);
    for (int i = 0; i < imageSize.height; i += 32) {
        line(mergeBeforeRectify, cv::Point(0, i), cv::Point(2 * imageSize.width, i), cv::Scalar(0, 0, 255), 1, cv::LINE_8);
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
    cv::stereoRectify(KLeft, DLeft, KRight, DRight,
                      imageSize, R, T, R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, 0, imageSize,
                      &validPixROI1, &validPixROI2);
    // 计算映射
    cv::Mat rmap[2][2];
    initUndistortRectifyMap(KLeft, DLeft, R1, P1, imageSize, CV_32FC1, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(KRight, DRight, R2, P2, imageSize, CV_32FC1, rmap[1][0], rmap[1][1]);
    remap(imgLeft, imgLeftRectify, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);//左校正
    remap(imgRight, imgRightRectify, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);//右校正

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
