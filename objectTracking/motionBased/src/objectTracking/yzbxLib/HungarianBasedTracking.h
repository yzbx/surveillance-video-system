#ifndef HUNGARIANBASEDTRACKING_H
#define HUNGARIANBASEDTRACKING_H
#include <QtCore>
#include <opencv2/opencv.hpp>
#include "BlobDetector.h"
#include "../lib/tracking_yzbx.h"
#include "singleobjecttracker.h"
#include "HungarianAlg.h"
#include "yzbx_config.h"

class HungarianBasedTracking: public QThread
{
public:
    HungarianBasedTracking()
    {
        //tracks.push_back(std::make_unique<singleObjectTracker>(fv[i], dt, Accel_noise_mag, NextTrackID++));
        //KF(of.pos, dt, Accel_noise_mag)
        //the ID for every singleObjectTracker
        NextTrackID=0;
        dt=0.2f;
        Accel_noise_mag=0.1f;

        //if (Cost[i + assignment[i] * N] > dist_thres)
        dist_thres = 100;

        maximum_allowed_skipped_frames = 10;
        max_trace_length=100;

        globalFirstDump=true;
        globalFirstOutput=true;
        frameNum=0;
        outputFileName="out.txt";
        FirstFG_frameNum=0;
    }

    void init(QString dumpFileName){
        outputFileName=dumpFileName;
        globalFirstDump=true;
        globalFirstOutput=true;
        frameNum=0;
    }

    virtual void tracking(const cv::Mat &img_input, const cv::Mat &img_fg);
    int FirstFG_frameNum;

private:
    void hungarianTracking(vector<trackingObjectFeature> &fv);
    void dump(){
        QFile data(outputFileName);
        if(globalFirstDump){
            if (!data.open(QFile::WriteOnly|QFile::Truncate)) {
                qDebug()<<"cannot open file "<<outputFileName;
                exit(-1);
            }
        }
        else{
            if (!data.open(QFile::ReadWrite|QFile::Append)) {
                qDebug()<<"cannot open file "<<outputFileName;
                exit(-1);
            }
        }

        QTextStream out(&data);
        if(globalFirstDump){
            QStringList formatList;
            formatList<<"frameNum"<<"track_id"<<"status"
                     <<"predict.x"<<"predict.y"<<"skipped_frames"
                    <<"pox.x"<<"pos.y"
                   <<"rect.x"<<"rect.y"<<"rect.width"<<"rect.height"
                  <<"size"<<"radius"<<"Convexity"<<"Circularity"<<"Inertia";
            out<<formatList.join("\t")<<"\n";
        }
        for(uint i;i<tracks.size();i++){
            QString dumpstr=QString::number(frameNum-1)+"\t"+tracks[i]->dump()+"\n";
            //            qDebug()<<"dumpstr="<<dumpstr;
            out<<dumpstr;
        }
        data.close();
        globalFirstDump=false;
    }

    void output(int index){
        QFileInfo info(outputFileName);
        QString suffix=QString(".")+info.suffix();
        QString filterName=outputFileName;
        filterName.replace(suffix,QString("_filtered")+suffix);
        QFile data(filterName);
        if(globalFirstOutput){
            if (!data.open(QFile::WriteOnly|QFile::Truncate)) {
                qDebug()<<"cannot open file "<<filterName;
                exit(-1);
            }
        }
        else{
            if (!data.open(QFile::ReadWrite|QFile::Append)) {
                qDebug()<<"cannot open file "<<filterName;
                exit(-1);
            }
        }

        QTextStream out(&data);
        if(globalFirstOutput){
            QStringList formatList;
            formatList<<"frameNum"<<"track_id"
                     <<"rect.x"<<"rect.y"<<"rect.width"<<"rect.height";
            out<<formatList.join(",")<<"\n";
        }
        int n=tracks[index]->rects.size();
        for(uint i;i<n;i++){
            QStringList dumpList;
            dumpList<<QString::number(tracks[index]->rects[i].x)<<QString::number(tracks[index]->rects[i].y)<<
                      QString::number(tracks[index]->rects[i].width)<<QString::number(tracks[index]->rects[i].height);
            QString dumpstr=QString::number(frameNum-n+i)+","+QString::number(tracks[index]->track_id)+ \
                    ","+dumpList.join(",")+"\n";
            //            qDebug()<<"dumpstr="<<dumpstr;
            out<<dumpstr;
        }
        data.close();
        globalFirstOutput=false;
    }
protected:
    void showing(const cv::Mat &img_input, const cv::Mat &img_fg, std::vector<trackingObjectFeature> featureVector);
    void run();

    BlobDetector blobDetector;

    std::vector<std::unique_ptr<singleObjectTracker>> tracks;

    int NextTrackID;
    track_t dt;
    track_t Accel_noise_mag;

    track_t dist_thres;
    size_t maximum_allowed_skipped_frames;
    size_t max_trace_length;

    cv::Mat img_input,img_fg;

    int frameNum;
    QString outputFileName;
    bool globalFirstDump;
    bool globalFirstOutput;
};

#endif // HUNGARIANBASEDTRACKING_H
