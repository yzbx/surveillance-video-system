#include "qyzbxTrackingFeatures.h"

QYzbxTrackingFeature::QYzbxTrackingFeature()
{

}

void QYzbxTrackingFeature::getObjects(const Mat &img_input, const Mat &fgMask, FrameFeature &frameFeatures)
{
    cv::Mat labelImg,binFgMask;
    vector<int> labelVector;

    cv::threshold (fgMask,binFgMask,1,1,CV_THRESH_BINARY);
    if(binFgMask.type()==CV_8UC3){
        cv::cvtColor(binFgMask,binFgMask,CV_BGR2GRAY);
    }
//    cv::imshow("binFgMask 001",binFgMask==1);
    cv::medianBlur(binFgMask,binFgMask,5);
    icvprCcaByTwoPass(binFgMask,labelImg,&labelVector);

    int row=fgMask.rows;
    int col=fgMask.cols;

//    cv::Mat newFgMask=cv::Mat::zeros(fgMask.size(),CV_8UC1);
    double areaThreshold=std::min(row*col/100.0,50.0);
    for(unsigned i=2;i<labelVector.size();i++){
        if(labelVector[i]==i){
            cv::Mat objMask=(labelImg==i);
            cv::Scalar area=cv::sum(objMask);
            if(area[0]>areaThreshold){
                ObjectFeature fea;
                fea.pos=getMomentFromObject(objMask);
                fea.size=area[0]/255;

                frameFeatures.features.push_back(fea);
//                newFgMask|=objMask;
            }
            else{
                qDebug()<<"small object, area="<<area[0]/255<<" in frame "<<frameFeatures.frameNum;
            }
        }
    }

//    imshow("new fgMask",newFgMask);
}

vector<Mat> QYzbxTrackingFeature::getObjectsFromMask(const Mat &fgMask,bool show)
{
    cv::Mat labelImg,binFgMask;
    vector<int> labelVector;

    cv::threshold (fgMask,binFgMask,1,1,CV_THRESH_BINARY);
    if(binFgMask.type()==CV_8UC3){
        cv::cvtColor(binFgMask,binFgMask,CV_BGR2GRAY);
    }
    cv::medianBlur(binFgMask,binFgMask,5);
    icvprCcaByTwoPass(binFgMask,labelImg,&labelVector);

    vector<Mat> objMasks;
    int areaThreshold=100*255;
    for(unsigned i=2;i<labelVector.size();i++){
        if(labelVector[i]==i){
            cv::Mat objMask=(labelImg==i);
            cv::Scalar area=cv::sum(objMask);
            if(area[0]>areaThreshold){
                objMasks.push_back(objMask);
            }
            else{
                qDebug()<<"small object, area="<<area[0];
            }
        }
    }

    //show the object masks
    if(show){
        imshow("foreground Mask",fgMask);
        for(unsigned i=0;i<objMasks.size();i++){
            showImgInLoop(objMasks[i],i,"object mask");
        }
    }

    return objMasks;
}

Point2f QYzbxTrackingFeature::getMomentFromObject(const Mat &objMask)
{
    cv::Moments m=cv::moments(objMask);
    return Point2f(m.m10/m.m00,m.m01/m.m00);
}
