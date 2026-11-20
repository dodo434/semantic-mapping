#include <ORBextractor.h>

#include <opencv2/core.hpp>

#include <iostream>
#include <vector>

int main()
{
  ORB_SLAM3::ORBextractor extractor(600, 1.2f, 8, 20, 7);
  cv::Mat image(480, 640, CV_8UC1);
  cv::randu(image, 0, 255);
  cv::Mat descriptors;
  std::vector<cv::KeyPoint> keypoints;
  std::vector<int> overlap{0, 0};
  extractor(image, cv::Mat(), keypoints, descriptors, overlap);
  if (keypoints.empty()) {
    std::cerr << "Expected features in the unmasked image\n";
    return 1;
  }
  extractor.SetDynamicMask(cv::Mat(image.size(), CV_8UC1, cv::Scalar(255)));
  extractor(image, cv::Mat(), keypoints, descriptors, overlap);
  if (!keypoints.empty() || !descriptors.empty()) {
    std::cerr << "Excluded pixels supplied features or descriptors\n";
    return 1;
  }
  return 0;
}
