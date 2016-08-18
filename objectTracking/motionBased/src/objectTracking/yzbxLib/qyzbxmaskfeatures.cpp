#include "qyzbxmaskfeatures.h"

qyzbxMaskFeatures::qyzbxMaskFeatures()
{

}

vector<Mat> qyzbxMaskFeatures::getObjectsFromMask(const Mat &fgMask,bool show)
{
    cv::Mat labelImg;
    vector<int> labelVector;
    icvprCcaByTwoPass(fgMask,labelImg,&labelVector);

    vector<Mat> objMasks;
    for(unsigned i=2;i<labelVector.size();i++){
        if(labelVector[i]==i){
            cv::Mat objMask=(labelImg==i);
            objMasks.push_back(objMask);
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
