#ifndef OBJECTLOCALFEATUREMATCH_H
#define OBJECTLOCALFEATUREMATCH_H
//computer object local feature and return match for each object.
//that's to see, the pathWeights for GraphBasedTracker

#include "yzbx_config.h"
#include "yzbx_utility.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <QtCore>
using namespace std;
using namespace cv;

class ObjectLocalFeatureMatch
{
public:
    ObjectLocalFeatureMatch(int MaxFeaturePointNum=50);
    int MaxFeaturePointNum;
    void process(const cv::Mat &img1,const cv::Mat &mask1,const cv::Mat &img2,const cv::Mat &mask2){
        getGoodMatches(img1,mask1,img2,mask2);
    }
    void getFeaturePoint_Step1(const cv::Mat &img,const cv::Mat &mask, std::vector<cv::KeyPoint> &kps){
        if(!kps.empty()) kps.clear();

        Ptr<FeatureDetector> detector=new BRISK(10);
        assert(!detector.empty());

        vector<KeyPoint> keypoints;
        detector->detect(img, keypoints,mask);


        //NOTE detect(img,keypoints,mask) failed, keypoints out of mask!!!
//        assert(mask.channels()==1);
//        for(int i=0;i<keypoints.size();i++){
//            Point2f p=keypoints[i].pt;
////            assert(mask.at<uchar>(p.y,p.x)==255);
//            if(mask.at<uchar>(p.y,p.x)!=255){
//                keypoints.erase(keypoints.begin()+i);
//                i--;
//            }
//        }

        //use the best MaxFeaturePointNum feature point according to response.
        if(keypoints.size()<MaxFeaturePointNum){
            kps.swap(keypoints);
        }
        else{
            //sort from big to small!!!
            std::sort(keypoints.begin(), keypoints.end(),
                      [](KeyPoint const & a, KeyPoint const & b) -> bool
            { return a.response > b.response; } );

            assert(keypoints[0].response>=keypoints[1].response);

            const int n=keypoints.size();
            for(int i=0;i<MaxFeaturePointNum;i++){
                kps.push_back(keypoints[i]);
            }
        }
    }

    void getDescriptorMat_Step2(const cv::Mat &img,vector<KeyPoint> &kps ,cv::Mat &des){
        Ptr<DescriptorExtractor> extractor= DescriptorExtractor::create("FREAK");
        assert(!extractor.empty());
        extractor->compute(img, kps, des);
    }

    static void getGoodMatches_Step3(Mat &descriptors_1, Mat &descriptors_2, vector<DMatch> &good_matches){
        Ptr<DescriptorMatcher> matcher=new BFMatcher(NORM_HAMMING,true);
        matcher->match(descriptors_1, descriptors_2, good_matches);
    }

    void getGoodMatches(const cv::Mat &img1,const cv::Mat &mask1,const cv::Mat &img2,const cv::Mat &mask2){
        std::cout<<"image type: "<<getImgType(mask1.type())<<std::endl;
        std::cout<<"image type: "<<getImgType(mask2.type())<<std::endl;
        assert(img1.channels()==3&&img2.channels()==3);
        assert(mask1.type()==CV_8UC1&&mask2.type()==CV_8UC1);

        vector<KeyPoint> keypoints_1,keypoints_2;
        getFeaturePoint_Step1(img1,mask1,keypoints_1);
        getFeaturePoint_Step1(img2,mask2,keypoints_2);
        if(keypoints_1.empty()||keypoints_2.empty()){
            return;
        }

        //        cv::initModule_contrib();
        //        cv::initModule_features2d();
        Mat descriptors_1,descriptors_2;
        getDescriptorMat_Step2(img1,keypoints_1,descriptors_1);
        getDescriptorMat_Step2(img2,keypoints_2,descriptors_2);

        //img1-->img2
        vector<DMatch> good_matches;
        getGoodMatches_Step3(descriptors_1, descriptors_2, good_matches);

        Mat img_matches;
        drawMatches( img1, keypoints_1, img2, keypoints_2,
                     good_matches, img_matches);
        //-- Show detected matches
        imshow( "Good Matches", img_matches);

        //-- set global matches
        std::swap(global_keypoints_1,keypoints_1);
        std::swap(global_keypoints_2,keypoints_2);
        std::swap(global_good_matches,good_matches);
    }

    void getLIFMat(cv::Mat &LIFMat,std::vector<Point_t> &LIFPos,const cv::Mat &img1,const cv::Mat &mask1){
        vector<KeyPoint> keypoints_1;
        getFeaturePoint_Step1(img1,mask1,keypoints_1);

        if(keypoints_1.empty()){
            return;
        }

        if(!LIFMat.empty()) LIFMat.release();
        getDescriptorMat_Step2(img1,keypoints_1,LIFMat);
        if(!LIFPos.empty()) LIFPos.clear();
        for(int i=0;i<keypoints_1.size();i++){
            LIFPos.push_back(keypoints_1[i].pt);
        }
        assert(LIFPos.size()==LIFMat.rows);
    }

    double global_match_ratio=0.8;
    vector<KeyPoint> global_keypoints_1,global_keypoints_2;
    vector<DMatch> global_good_matches;
};

#endif // OBJECTLOCALFEATUREMATCH_H
