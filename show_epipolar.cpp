//
// Created by tianhe on 2022/7/27.
//
#include <iostream>
#include <opencv2/opencv.hpp>

/**
 *
 * @param img1  图像1-输入
 * @param img2  图像2-输入
 * @param lines 极线-输入
 * @param pts1  特征点1-输入
 * @param pts2  特征点2-输入
 * @param out1  绘制好特征点和极线的图像1-输出
 * @param out2 绘制好特征点和极线的图像2-输出
 */
void drawLines(cv::Mat &img1, cv::Mat &img2,
               std::vector<cv::Vec<float, 3>> lines, std::vector<cv::Point2i> pts1, std::vector<cv::Point2i> pts2,
               cv::Mat &out1, cv::Mat &out2) {
    int c = img1.cols;
    cvtColor(img1, out1, cv::COLOR_GRAY2BGR);
    cvtColor(img2, out2, cv::COLOR_GRAY2BGR);

    cv::RNG &rng = cv::theRNG();
    for (int i = 0; i < lines.size(); i++) {
        //随机产生颜色
        cv::Scalar color = cv::Scalar(rng(255), rng(255), rng(255));
        line(out1, cv::Point2i(0, -lines[i][2] / lines[i][1]),
             cv::Point2i(c, -(lines[i][2] + lines[i][0] * c) / lines[i][1]), color);
        circle(out1, pts1[i], 5, color, -1);
        circle(out2, pts2[i], 5, color, -1);
    }
}

int main02() {
    std::string pathLeft = R"(..\data\left\Left0103.bmp)";
    std::string pathRight = R"(..\data\right\Right0103.bmp)";
//    std::string pathLeft = R"(..\result\left.bmp)";
//    std::string pathRight = R"(..\result\right.bmp)";
    cv::Mat imgLeft = cv::imread(pathLeft, cv::IMREAD_GRAYSCALE);
    cv::Mat imgRight = cv::imread(pathRight, cv::IMREAD_GRAYSCALE);

    // SIFT
    cv::Ptr<cv::SIFT> detector = cv::SIFT::create();

    // 特征点和描述子检测
    std::vector<cv::KeyPoint> kpLeft, kpRight;
    cv::Mat descLeft, descRight;
    detector->detectAndCompute(imgLeft, cv::Mat(), kpLeft, descLeft);
    detector->detectAndCompute(imgRight, cv::Mat(), kpRight, descRight);

    // 匹配描述子
    std::vector<std::vector<cv::DMatch>> matches;
    cv::FlannBasedMatcher matcher;

    matcher.knnMatch(descLeft, descRight, matches, 2);
    std::cout << "Find total " << matches.size() << " matches." << std::endl;

    // 筛选匹配对
    std::vector<cv::DMatch> goodMatches;
    std::vector<cv::Point2i> ptsLeft, ptsRight;
    for (size_t i = 0; i < matches.size(); i++) {
        if (matches[i][0].distance < 0.3 * matches[i][1].distance) {
            goodMatches.push_back(matches[i][0]);
            ptsLeft.push_back(kpLeft[matches[i][0].queryIdx].pt);
            ptsRight.push_back(kpRight[matches[i][0].trainIdx].pt);
        }
    }
    //首先根据对应点计算出两视图的基础矩阵，基础矩阵包含了两个相机的外参数关系
    cv::Mat fundamental_matrix = findFundamentalMat(ptsLeft, ptsRight, cv::FM_8POINT);
    std::cout << "fundamentalMatrix: " << std::endl << fundamental_matrix << std::endl;

    //计算对应点的外极线epilines是一个三元组(a,b,c)，表示点在另一视图中对应的外极线ax+by+c=0;
    std::vector<cv::Vec<float, 3>> epipolarLeft, epipolarRight;
    computeCorrespondEpilines(ptsRight, 2, fundamental_matrix, epipolarLeft);
    cv::Mat img5, img6;
    drawLines(imgLeft, imgRight, epipolarLeft, ptsLeft, ptsRight, img5, img6);

    computeCorrespondEpilines(ptsLeft, 1, fundamental_matrix, epipolarRight);
    cv::Mat img3, img4;
    drawLines(imgRight, imgLeft, epipolarRight, ptsRight, ptsLeft, img3, img4);

    cv::Mat merge;
    cv::hconcat(img5, img3, merge);
//    imshow("epipolarLeft", img5);
//    imshow("epipolarRight", img3);
    imshow("epipolar", merge);
    cv::imwrite("../result/epipolar.bmp", merge);
    cv::waitKey(0);
    return 0;
}
