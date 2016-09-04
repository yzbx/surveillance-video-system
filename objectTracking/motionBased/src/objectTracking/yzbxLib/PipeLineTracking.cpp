#include "PipeLineTracking.h"

PipeLineTracking::PipeLineTracking()
{

}

void PipeLineTracking::process(QString sourceData)
{
    FrameInput fin;
    cv::Mat img_input,img_fg,img_bg;
    bgsFactory_yzbx fac;
    IBGS *ibgs=fac.getBgsAlgorithm(bgsType);
    BlobDetector bd;
    RectFloatTracker tracker;
    TrackingBlobsMatchAnnotation matcher;
    std::vector<string> wins;
    wins.push_back("img_input");
    wins.push_back("img_fg");
    wins.push_back("mask_i");
    wins.push_back("MultiCueBGS FG");
    wins.push_back("featurePoint");
    wins.push_back("binaryImage");
    yzbxlib::moveWindows(wins,3);
    int frameNum=0;
    QString annTxt="/home/yzbx/Downloads/BitSync/surveillance-video-system/UrbanTracker-Annotation/MOT2D2015/atrium.csv";
    while(1){
        fin.process(sourceData,img_input);
        if(img_input.empty()){
            break;
        }
        ibgs->process(img_input,img_fg,img_bg);
        if(img_fg.empty()){
            qWarning()<<"empty img_fg for frameNum="<<frameNum;
        }
        else{
            std::vector<trackingObjectFeature> fv;
            bd.process(img_input,img_fg,fv);
            cv::imshow("img_input",img_input);
            cv::imshow("img_fg",img_fg);
            cv::waitKey(30);
//            tracker.process(img_input,img_fg,fv);
            vector<int> ids;
            matcher.process(frameNum,dist_thres,fv,annTxt,ids);
            PipeLine_DumpFV(frameNum,ids,fv);
        }
        frameNum++;
    }

    PipeLine_Replay(sourceData,annTxt);
}

void PipeLineTracking::PipeLine_DumpFV(int frameNum,const vector<int> &ids,vector<trackingObjectFeature> &fv)
{
    QString dumpFileName="dumpFV.txt";
    QFile file(dumpFileName);
    if(firstDump){
        if(!file.open(QIODevice::WriteOnly|QIODevice::Truncate)){
            qDebug()<<"cannot open file "<<dumpFileName;
            assert(false);
        }

        firstDump=false;
    }
    else{
        if(!file.open(QIODevice::WriteOnly|QIODevice::Append)){
            qDebug()<<"cannot open file "<<dumpFileName;
            assert(false);
        }
    }


    QTextStream out(&file);
    for(int i=0;i<fv.size();i++){
        if(ids[i]==-1){
            continue;
        }

        Rect_t &r=fv[i].rect;
        QStringList strList;
        strList<<QString::number(frameNum)<<QString::number(ids[i])<<QString::number(r.x)<<QString::number(r.y)<<QString::number(r.width)<<QString::number(r.height);

        out<<strList.join(",")<<"\n";
    }

    file.close();
}

void PipeLineTracking::PipeLine_Replay(QString dataSource,QString replaySource){
    TrackingResultReplay replay;
    replay.process(dataSource,replaySource,bgsType);
}
