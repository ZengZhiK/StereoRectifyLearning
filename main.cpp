#include <opencv2/opencv.hpp>
#include <iostream>

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
    cv::Mat T = (cv::Mat_<double>(3, 1) << -89.51204, -0.21901, -0.28905);

    cv::Size imageSize = cv::Size(640, 480);

    // 读取影像
    std::string pathLeft = R"(..\data\left\Left1.bmp)";
    std::string pathRight = R"(..\data\right\Right1.bmp)";
    cv::Mat imgLeft = cv::imread(pathLeft, cv::IMREAD_COLOR);
    cv::Mat imgRight = cv::imread(pathRight, cv::IMREAD_COLOR);

    cv::Mat mergeBeforeRectify;
    cv::hconcat(imgLeft, imgLeft, mergeBeforeRectify);
    for (int i = 0; i < imageSize.height; i += 32) {
        line(mergeBeforeRectify, cv::Point(0, i), cv::Point(2 * imageSize.width, i), cv::Scalar(0, 0, 255), 1, 8);
    }
    cv::imshow("BeforeRectify", mergeBeforeRectify);


    // 计算旋转矩阵和投影矩阵
    cv::Mat R1, R2, P1, P2, Q, imgLeftRectify, imgRightRectify;
    cv::Rect validPixROI1, validPixROI2;   //左右相机立体修正后有效像素的区域
    cv::stereoRectify(KLeft, DLeft, KRight, DRight,
                      imageSize, R, T, R1, R2, P1, P2, Q,
                      cv::CALIB_ZERO_DISPARITY, 1, imageSize,
                      &validPixROI1, &validPixROI2);
    // 计算映射
    cv::Mat rmap[2][2];
    initUndistortRectifyMap(KLeft, DLeft, R1, P1, imageSize, CV_32FC1, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(KRight, DRight, R2, P2, imageSize, CV_32FC1, rmap[1][0], rmap[1][1]);
    remap(imgLeft, imgLeftRectify, rmap[0][0], rmap[0][1], CV_INTER_AREA);//左校正
    remap(imgRight, imgRightRectify, rmap[1][0], rmap[1][1], CV_INTER_AREA);//右校正
    rectangle(imgLeftRectify, validPixROI1, cv::Scalar(0, 255, 0));
    rectangle(imgRightRectify, validPixROI2, cv::Scalar(0, 255, 0));


    cv::Mat mergeAfterRectify;
    cv::hconcat(imgLeftRectify, imgRightRectify, mergeAfterRectify);
    for (int i = 0; i < imageSize.height; i += 32) {
        line(mergeAfterRectify, cv::Point(0, i), cv::Point(2 * imageSize.width, i), cv::Scalar(0, 0, 255), 1, 8);
    }
    cv::imshow("AfterRectify", mergeAfterRectify);
    cv::waitKey(0);
}
