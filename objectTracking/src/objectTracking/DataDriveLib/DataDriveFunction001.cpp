#include "DataDriveFunction001.h"

namespace  DataDrive {
void dumpRectInTracker(const std::vector<std::unique_ptr<singleObjectTracker> > &tracks)
{
    for(uint i=0;i<tracks.size();i++){
        int id=tracks[i]->track_id;
        Rect_t rect=tracks[i]->feature->rect;
        cout<<"id="<<id<<", rect=["<<rect.x<<","<<rect.y<<","<<
              rect.width<<","<<rect.height<<"]"<<endl;
    }
}

void dumpRect(Rect_t rect)
{
    cout<<"rect=["<<rect.x<<","<<rect.y<<","<<
          rect.width<<","<<rect.height<<"]"<<endl;
}

void drawRectInTracker(const std::vector<std::unique_ptr<singleObjectTracker> > &tracks, const Mat &img,string title)
{
    namedWindow(title,WINDOW_NORMAL);

    Mat img_show=img.clone();
    for(uint i=0;i<tracks.size();i++){
        Rect_t rect=tracks[i]->feature->rect;
        rectangle(img_show,rect,Scalar::all(255),3);
    }

    imshow(title,img_show);
}

void dumpObjects(const std::vector<std::unique_ptr<singleObjectTracker> > &tracks,bool output)
{
    if(output){
        for(uint i=0;i<tracks.size();i++){
            tracks[i]->dumpToScreen();
        }
    }
}

void updateMatchedFeature(const std::shared_ptr<DataDriveMain> &data, ObjectFeature &of)
{
    (void)data;
    (void)of;
    assert(false);
}

track_t calcDist(std::shared_ptr<trackingObjectFeature> of1, std::shared_ptr<trackingObjectFeature> of2, int costType)
{
    if(costType==RectDist){
        Rect_t &r1=of1->rect;
        Rect_t &r2=of2->rect;
        Point_t p1=0.5f*(r1.tl()+r1.br());
        Point_t p2=0.5f*(r2.tl()+r2.br());
        track_t dist=fabs(r1.width-r2.width)+fabs(r1.height-r2.height)+cv::norm(p1-p2);

        return dist;
    }
    else if(costType==PosDist){
        Point_t p1=of1->pos;
        Point_t p2=of2->pos;
        return cv::norm(p1-p2);
    }
    else if(costType==KLTDist){
        assert(false);
        return -1.0f;
    }
    else{
        assert(false);
        return -1.0f;
    }
}

}
