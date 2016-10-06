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

}
