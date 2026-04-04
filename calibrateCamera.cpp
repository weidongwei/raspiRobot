#include <vector>
#include <opencv2/opencv.hpp>



bool calibrateCameraFromImages(
    const std::vector<cv::Mat>& images,
    cv::Size boardSize,
    float squareSize,
    cv::Mat& cameraMatrix,
    cv::Mat& distCoeffs,
    double& reprojError,
    const std::string& debugSaveDir
) {
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;

    // 构造棋盘世界坐标
    std::vector<cv::Point3f> objp;
    for (int i = 0; i < boardSize.height; ++i) {
        for (int j = 0; j < boardSize.width; ++j) {
            objp.emplace_back(j * squareSize,
                              i * squareSize,
                              0.0f);
        }
    }

    cv::Size imageSize;
    int imgIdx = 0;

    for (const auto& img : images) {
        if (img.empty()) {
            imgIdx++;
            continue;
        }

        imageSize = img.size();

        cv::Mat gray;
        cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(
            gray,
            boardSize,
            corners,
            cv::CALIB_CB_ADAPTIVE_THRESH |
            cv::CALIB_CB_NORMALIZE_IMAGE
        );

        // ---------- 保存中间图像 ----------
        cv::Mat vis = img.clone();

        if (found) {
            cv::cornerSubPix(
                gray,
                corners,
                cv::Size(11, 11),
                cv::Size(-1, -1),
                cv::TermCriteria(
                    cv::TermCriteria::EPS |
                    cv::TermCriteria::COUNT,
                    30, 0.001
                )
            );

            imagePoints.push_back(corners);
            objectPoints.push_back(objp);

            cv::drawChessboardCorners(
                vis,
                boardSize,
                corners,
                found
            );
        }

        // 保存图像（无论成功还是失败）
        std::ostringstream name;
        name << debugSaveDir << "/calib_"
             << imgIdx
             << (found ? "_ok.jpg" : "_fail.jpg");

        cv::imwrite(name.str(), vis);
        imgIdx++;
    }

    if (imagePoints.size() < 10) {
        std::cerr << "Not enough valid calibration images." << std::endl;
        return false;
    }

    std::vector<cv::Mat> rvecs, tvecs;

    double rms = cv::calibrateCamera(
        objectPoints,
        imagePoints,
        imageSize,
        cameraMatrix,
        distCoeffs,
        rvecs,
        tvecs
    );

    // ---------- 计算重投影误差 ----------
    double totalError = 0.0;
    size_t totalPoints = 0;

    for (size_t i = 0; i < objectPoints.size(); ++i) {
        std::vector<cv::Point2f> projected;
        cv::projectPoints(
            objectPoints[i],
            rvecs[i],
            tvecs[i],
            cameraMatrix,
            distCoeffs,
            projected
        );

        double err = cv::norm(imagePoints[i], projected, cv::NORM_L2);
        totalError += err * err;
        totalPoints += objectPoints[i].size();
    }

    reprojError = std::sqrt(totalError / totalPoints);

    return true;
}

//相机标定
int biaoding(){
    std::vector<cv::Mat> images;
    // 读取标定图像
    for (int i = 1; i <= 22; ++i) {
        std::string path = "/home/dw/robot/image/origin_image/biaoding" + std::to_string(i) + ".jpg";
        cv::Mat img = cv::imread(path);
        if (!img.empty())
            images.push_back(img);
    }

    cv::Mat cameraMatrix, distCoeffs;
    double reprojError;

    bool ok = calibrateCameraFromImages(
        images,
        cv::Size(11, 8),
        3.0f,
        cameraMatrix,
        distCoeffs,
        reprojError,
        "/home/dw/robot/image/origin_image"
    );

    if (ok) {
        std::cout << "\n===== Camera Calibration Result =====\n";

        std::cout << "Camera Matrix (K):\n";
        std::cout << cameraMatrix << std::endl;

        std::cout << "\nDistortion Coefficients (D):\n";
        std::cout << distCoeffs << std::endl;

        std::cout << "\nReprojection Error:\n";
        std::cout << reprojError << std::endl;

        std::cout << "=====================================\n";
    }
    return 0;
}

//激光像素坐标转实际距离,平面到喷嘴的距离
double y_pixel_to_distance1(double y_pixel) {
    double a = 5.63095238;
    double b = -97.55952381;
    double c = 462.5952381;

    double A = a;
    double B = b;
    double C = c - y_pixel;

    double delta = B*B - 4*A*C;
    if (delta < 0) return -1; // 无解

    double d1 = (-B + sqrt(delta)) / (2*A);
    double d2 = (-B - sqrt(delta)) / (2*A);


    return (d1 >= 0 && d1 <= 9) ? d1 : d2;
}

double y_pixel_to_distance2(double y_pixel) {
    double a = 4.35714286;
    double b = -87.23809524;
    double c = 523.88095238;

    double A = a;
    double B = b;
    double C = c - y_pixel;

    double delta = B*B - 4*A*C;
    if (delta < 0) return -1; // 无解

    double d1 = (-B + sqrt(delta)) / (2*A);
    double d2 = (-B - sqrt(delta)) / (2*A);


    return (d1 >= 0 && d1 <= 9) ? d1 : d2;
}


