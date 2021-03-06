#ifndef YZBX_UTILITY_H
#define YZBX_UTILITY_H
#include <opencv2/opencv.hpp>
#include <iostream>
#include "lbp/lbp.hpp"
#include "lbp/histogram.hpp"
//#include "lib/LBP1/LBP1.hpp"
#include "yzbx_config.h"
#include <boost/lexical_cast.hpp>
#include <boost/chrono.hpp>
#include <QString>
#include <QStringList>
#include <fstream>
#include <QtCore>

#define MEASURE_TIME(x)                        \
{ auto startTime = cv::getTickCount(); \
    x;                                   \
    auto endTime = cv::getTickCount();   \
    std::cout << #x << " " << (endTime - startTime)<< std::endl; }

#define GET_CLASS_NAME string getClassName(){return (string)typeid(this).name();}

#define MEASURE_TIME_MS(x) \
{ \
    boost::chrono::nanoseconds start; \
    x; \
    boost::chrono::nanoseconds end; \
    typedef boost::chrono::milliseconds ms; \
    ms d = boost::chrono::duration_cast<ms>(end - start); \
    std::cout << #x << " " << d.count() <<"ms"<< std::endl; \
}

#define MEASURE_TIME_NS(x) \
{ \
    boost::chrono::nanoseconds start; \
    x; \
    boost::chrono::nanoseconds end; \
    typedef boost::chrono::nanoseconds ns; \
    ns d = boost::chrono::duration_cast<ns>(end - start); \
    std::cout << #x << " " << d.count() <<"ns"<< std::endl; \
}

namespace yzbxlib
{
QString getAbsoluteFilePath(QString currentPathOrFile, QString fileName);
bool isSameImage(const cv::Mat &A_8U,const cv::Mat &B_8U);
void showImageInWindow(string windowName,const cv::Mat &img);
void moveWindows(std::vector<std::string> &windowNames, int colNum=4);
void dumpVector(std::vector<int> &v);
void dumpVector(std::vector<float> &v);
bool isRectAInRectB(Rect_t A, Rect_t B);
bool isPointInRect(cv::Point2f p, Rect_t rect);
track_t getRectGap(Rect_t ra,Rect_t rb);
void drawMatch(cv::Mat &img,Point_t p1,Point_t p2,cv::Scalar color=cv::Scalar(0,0,255),int thickness=3);
Rect_t getMergedRect(Rect_t ra,Rect_t rb);
bool splitRect(const Rect_t &mergedRect,Rect_t &r1,Rect_t &r2);
void annotation(cv::Mat &showImg, Point p, const string title, cv::Scalar color=cv::Scalar(0.0,255));
void annotation(Mat &showImg, Rect_t r, const string title="", cv::Scalar color=cv::Scalar(0,0,255));

//return the overlap area.
track_t getOverlapArea(Rect_t ra,Rect_t rb);

//return the ratio for max(overlap/area(ra),overlap/area(rb))
track_t getOverlapRatio(Rect_t ra,Rect_t rb);

//return the rect with the minimal cost move subRect to mergedRect.
Rect_t getSubRect(Rect_t mergedRect,Rect_t subRect);
//void insertToSetList(std::list<std::set<int>> &setList,int value);

//return rect distance
track_t getOverlapDist(Rect_t ra,Rect_t rb);
Point_t rectCenter(Rect_t r);

//from input to output
QString getOutputFileName(QString inputFileName);

void getGrayMat(const cv::Mat &input,cv::Mat &output);

void csvToTrajectory(QString line,int &frameNum,int &id,Rect &r);

bool rectOnBoundary(Rect r, Mat &img);
void dumpMap(std::map<Index_t,std::set<Index_t>> &m);
void moveMaskAndFitRect(cv::Mat &mask,Point2i displace,Rect_t rect);

///input: p,prev_p, line f(x,y)=Ax+By+C=0;
/// method: f(p)*f(prev_p)>0 || f(p)==0&&f(prev_p)!=0
bool isLineCrossed(Point_t p,Point_t prev_p,double A,double B,double C);

///input RGB, mask, value
/// method: RGB(mask)=value.
void getMaskedRGB(cv::Mat &RGB,const cv::Mat &mask,Scalar value=Scalar(0,0,255));

void MyHconcat(const Mat& A,const Mat &B, Mat &AB);
void MyVconcat(const Mat& A,const Mat &B, Mat &AB);
}

//set qt sync with file.

#define yzbx_match_BF 0
#define yzbx_match_KNN 1

using namespace std;
using namespace cv;
/*
    version: 1.0
    author: hellogiser
    blog: http://www.cnblogs.com/hellogiser
    date: 2014/5/30
*/
// hamming distance of two integer 0-1 bits
unsigned yzbx_hamdist(unsigned x, unsigned y);
unsigned yzbx_d1(unsigned x,unsigned y);
unsigned yzbx_d2(unsigned x,unsigned y);

//distance_type 0 hamdist, 1 d1, 2 d2,...
unsigned yzbx_distance(unsigned x,unsigned y,int distance_type=2);
//color_space BGR 0,Lab 1, ab 2,L*ab 3,LBP 4,5,6, ...
unsigned yzbx_distance_Vec3b(Vec3b x,Vec3b y,int distance_type=2,int color_space=0);
void showImgInLoop(const cv::Mat img,int i,string title="");
void cvt2CV_8U(const cv::Mat input,cv::Mat &output);
void showMat8U(const string windowName, const cv::Mat input);

//bool separatedMatCheck(const Mat &a,const Mat& b);
//bool separatedMatsCheck(vector<Mat> &va, vector<Mat> &vb);

void yzbx_imfill(Mat &input);

void yzbx_lbp(const Mat &input, Mat &output, int lbp_operator=1);

void yzbx_match(Mat &descriptor1, Mat &descriptor2, vector<DMatch> &matches, int matchType=yzbx_match_BF);

/*
 * connect component analysis for 4-connected.
 * icvprCcaByTwoPass _binImg (CV_8UC1) -> _lableImg (CV_32SC1)
 * */
void icvprCcaByTwoPass(const Mat &_binImg, Mat &_lableImg,std::vector<int> *labelVector=NULL);
void compressMat(Mat &mCV32SC1, Mat &mCV8UC1);
Scalar icvprGetRandomColor();
void icvprLabelColor(const Mat &_labelImg, Mat &_colorLabelImg);
void connectedComponentSplit(Mat &nextFGMask, Mat &labelImg8UC1);

string getImgType(int imgTypeInt);
void CDNet_GetRoiNum(QString filename, int &a, int &b);

size_t getDirections(float dx,float dy);
void filterBinaryImageByArea(Mat &binImage,double minarea);
void yzbx_generateErrorMap(const Mat& truthGround,const Mat & FGMask,Mat &errorMat);

//double threshold connect for binary image.
void doubleThresholdConnect(const Mat &highThresholdMask,const Mat &lowThresholdMask,Mat &outputMask);

#define _DEBUG 1

#if _DEBUG
//#define LOG_MESSAGE(x) std::cout << __FILE__ <<__FUNCTION__<< " (" << __LINE__ << "): " << x << std::endl;
#define LOG_MESSAGE(x) std::cout <<__FUNCTION__<< " (" << __LINE__ << "): " << x << std::endl;
#else
#define LOG_MESSAGE(x)
#endif

#endif // UTILITY_HPP
