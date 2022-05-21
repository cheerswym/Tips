#ifndef ONBOARD_UTILS_IMSHOW_IMSHOW_H_
#define ONBOARD_UTILS_IMSHOW_IMSHOW_H_

#include <string>

#include "opencv2/opencv.hpp"

namespace qcraft {
void imshow(const std::string& winname, const cv::Mat& mat);
void destroyWindow(const std::string& winname);
void waitKey(int delay_us = 0);
}  // namespace qcraft

#endif  // ONBOARD_UTILS_IMSHOW_IMSHOW_H_
