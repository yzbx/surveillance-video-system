#ifndef OBJECTLOCALFEATUREMATCH_H
#define OBJECTLOCALFEATUREMATCH_H
//computer object local feature and return match for each object.
//that's to see, the pathWeights for GraphBasedTracker

#include "yzbx_config.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>
using namespace std;
using namespace cv;

class ObjectLocalFeatureMatch
{
public:
    ObjectLocalFeatureMatch();
    void process(const cv::Mat &img1,const cv::Mat &img2,const cv::Mat &mask1,const cv::Mat &mask2){
        getGoodMatches(img1,img2,mask1,mask2);
    }

    void getGoodMatches(const cv::Mat &img1,const cv::Mat &img2,const cv::Mat &mask1,const cv::Mat &mask2){
        assert(img1.channels()==3&&img2.channels()==3);
        assert(mask1.type()==CV_8UC1&&img2.type()==CV_8UC1);

        Ptr<FeatureDetector> detector=FeatureDetector::create("FAST");
//        detector = new DynamicAdaptedFeatureDetector ( new FastAdjuster(10,true), 5000, 10000, 10);
        vector<KeyPoint> keypoints_1,keypoints_2;

        detector->detect(img1, keypoints_1,mask1);
        detector->detect(img2, keypoints_2,mask2);

        Mat descriptors_1, descriptors_2;
        Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("SIFT");
        extractor->compute( img1, keypoints_1, descriptors_1 );
        extractor->compute( img2, keypoints_2, descriptors_2 );

        //img1-->img2
        vector< vector<DMatch> > matches;
        Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
        matcher->knnMatch( descriptors_1, descriptors_2, matches, 2 );

        //look whether the match is inside a defined area of the image
        //only 25% of maximum of possible distance
//        double tresholdDist = 0.25 * sqrt(double(img1.size().height*img1.size().height + img1.size().width*img1.size().width));

//        vector< DMatch > good_matches2;
//        good_matches2.reserve(matches.size());
//        for (size_t i = 0; i < matches.size(); ++i)
//        {
//            for (int j = 0; j < matches[i].size(); j++)
//            {
//                Point2f from = keypoints_1[matches[i][j].queryIdx].pt;
//                Point2f to = keypoints_2[matches[i][j].trainIdx].pt;

//                //calculate local distance for each possible match
//                double dist = sqrt((from.x - to.x) * (from.x - to.x) + (from.y - to.y) * (from.y - to.y));

//                //save as best match if local distance is in specified area and on same height
//                if (dist < tresholdDist && abs(from.y-to.y)<5)
//                {
//                    good_matches2.push_back(matches[i][j]);
//                    j = matches[i].size();
//                }
//            }
//        }

        vector<KeyPoint> matched1, matched2;
        vector<DMatch> good_matches;
        for(size_t i = 0; i < matches.size(); i++) {
            DMatch first = matches[i][0];
            float dist1 = matches[i][0].distance;
            float dist2 = matches[i][1].distance;
            if(dist1 < global_match_ratio * dist2) {
                matched1.push_back(keypoints_1[first.queryIdx]);
                matched2.push_back(keypoints_2[first.trainIdx]);
                good_matches.push_back(first);
            }
        }

        //-- Draw only "good" matches
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

    double global_match_ratio=0.8;
    vector<KeyPoint> global_keypoints_1,global_keypoints_2;
    vector<DMatch> global_good_matches;
};

#endif // OBJECTLOCALFEATUREMATCH_H
