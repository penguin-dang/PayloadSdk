#include <opencv2/opencv.hpp>
#include <vector>

int main()
{
    cv::Mat img = cv::imread("/home/dangphan/Downloads/pattern_invert.png", 33);
    if (img.empty()) {
        std::cerr << "Error: Image not found!" << std::endl;
        return -1;
    }

    cv::imshow("image", img);
    cv::moveWindow("image", 40, 40);

    cv::Size patternsize(9, 6); // interior number of corners
    cv::Mat gray_tmp;
    
    cv::cvtColor(img, gray_tmp, cv::COLOR_BGR2GRAY); // source image

    // Invert the grayscale image
    cv::Mat gray = cv::Scalar::all(255) - gray_tmp;

    std::vector<cv::Point2f> corners; // this will be filled by the detected corners

    // CALIB_CB_FAST_CHECK saves a lot of time on images
    // that do not contain any chessboard corners
    bool patternfound = cv::findChessboardCorners(gray, patternsize, corners,
                        cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE
                        + cv::CALIB_CB_FAST_CHECK);

    if (patternfound)
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

    cv::drawChessboardCorners(img, patternsize, cv::Mat(corners), patternfound);

    cv::imshow("result", img);
    cv::moveWindow("result", img.cols / 2, 100);
    cv::waitKey(0);

    return 0;
}
