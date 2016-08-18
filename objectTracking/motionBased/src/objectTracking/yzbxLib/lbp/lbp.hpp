#ifndef LBP_HPP_
#define LBP_HPP_

//! \author philipp <bytefish[at]gmx[dot]de>
//! \copyright BSD, see LICENSE.

#include <opencv/cv.h>
#include <limits>
#define LBP_THRESHOLD 10
using namespace cv;
using namespace std;

namespace lbp {

// templated functions
template <typename _Tp>
void OLBP_(const cv::Mat& src, cv::Mat& dst,const size_t LBP_Threshold=LBP_THRESHOLD);

template <typename _Tp>
void ELBP_(const cv::Mat& src, cv::Mat& dst, int radius = 1, int neighbors = 8,const size_t LBP_Threshold=LBP_THRESHOLD);

template <typename _Tp>
void VARLBP_(const cv::Mat& src, cv::Mat& dst, int radius = 1, int neighbors = 8);

// wrapper functions
void OLBP(const Mat& src, Mat& dst,const size_t LBP_Threshold=LBP_THRESHOLD);
void ELBP(const Mat& src, Mat& dst, int radius = 1, int neighbors = 8,const size_t LBP_Threshold=LBP_THRESHOLD);
void VARLBP(const Mat& src, Mat& dst, int radius = 1, int neighbors = 8);

// Mat return type functions
Mat OLBP(const Mat& src,const size_t LBP_Threshold=LBP_THRESHOLD);
Mat ELBP(const Mat& src, int radius = 1, int neighbors = 8,const size_t LBP_Threshold=LBP_THRESHOLD);
Mat VARLBP(const Mat& src, int radius = 1, int neighbors = 8);

}
#endif
