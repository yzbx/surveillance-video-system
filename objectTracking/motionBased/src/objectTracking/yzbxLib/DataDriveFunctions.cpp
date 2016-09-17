#include "DataDriveFunctions.h"

namespace DataDrive {

bool Input::run()
{
    data->frameInput.getNextFrame(data->videoFilePath,data->img_input);
    data->frameNum++;
    if(data->img_input.empty()) return false;
    else return true;
}

bool Bgs::run()
{
    assert(!data->img_input.empty());
    if(!data->img_fg.empty()) data->img_fg.release();

    cv::Mat img_fg;
    data->bgs->process(data->img_input,img_fg,data->img_background);
    if(img_fg.empty()){
        return false;
    }
    else{
        if(img_fg.channels()==3){
            cv::cvtColor(img_fg,data->img_fg,CV_BGR2GRAY);
        }
        else{
            data->img_fg=img_fg;
        }

        return true;
    }
}

bool BlobFeature::run()
{
    assert(!data->img_fg.empty());
    assert(!data->img_input.empty());
    assert(data->fvlist.size()==data->imglist.size());
    data->blobFeatureDetector.getBlobFeature(data->img_input,data->img_fg,data->fv);
    data->fvlist.push_back(data->fv);
    data->imglist.push_back(std::make_pair(data->img_input,data->img_fg));

    if(data->fvlist.size()>data->MaxListLength){
        data->fvlist.pop_front();
        data->imglist.pop_front();
    }
    return true;
}

bool Tracker::run()
{
    return true;
}

bool KLTTracker::run()
{

    if(data->fvlist.size()<2){
        return false;
    }

    assert(data->img_input.channels()==3);
    auto it=std::prev(data->imglist.end(),2);
    cv::Mat gray,prev_gray;
    cv::cvtColor(data->img_input,gray,CV_BGR2GRAY);
    cv::cvtColor(it->first,prev_gray,CV_BGR2GRAY);

    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    Size subPixWinSize(10,10), winSize(31,31);
    const int MAX_COUNT = 500;
    vector<Point2f> points[2];
    //    goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
    //    cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
    goodFeaturesToTrack(prev_gray, points[0], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
    cornerSubPix(prev_gray, points[0], subPixWinSize, Size(-1,-1), termcrit);

    vector<uchar> status;
    vector<float> err;
    calcOpticalFlowPyrLK(prev_gray, gray, points[0], points[1], status, err, winSize,
            3, termcrit, 0, 0.001);

    assert(points[0].size()==points[1].size());
    size_t i, k;

    Mat image;
    data->img_input.copyTo(image);
    for( i = k = 0; i < points[1].size(); i++ )
    {
        if( !status[i] )
            continue;

        points[0][k]=points[0][i];
        points[1][k++] = points[1][i];
        circle( image, points[1][i], 3, Scalar(0,255,0), -1, 8);
        line(image,points[0][k],points[1][k],Scalar(0,0,255),3,8);
    }
    points[1].resize(k);

    cv::namedWindow("KLT",WINDOW_NORMAL);
    imshow("KLT",image);

    waitKey(0);
    return true;
}

}
