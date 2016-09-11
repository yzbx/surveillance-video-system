#include "PipeLineTracking.h"

PipeLineTracking::PipeLineTracking()
{

}

void PipeLineTracking::process(QString sourceData,QString annTxt)
{
    ibgs=fac.getBgsAlgorithm(bgsType);

    std::vector<string> wins;
    wins.push_back("img_input");
    wins.push_back("img_fg");
    wins.push_back("mask_i");
    wins.push_back("MultiCueBGS FG");
    wins.push_back("featurePoint");
//    wins.push_back("binaryImage");
    yzbxlib::moveWindows(wins,3);
    int frameNum=0;
    if(annTxt==""){
        annTxt="/home/yzbx/Downloads/BitSync/surveillance-video-system/UrbanTracker-Annotation/MOT2D2015/atrium.csv";
    }

    while(1){
        PipeLine_Input(sourceData);
        if(img_input.empty()){
            break;
        }
        PipeLine_Bgs();
        if(img_fg.empty()){
            qWarning()<<"empty img_fg for frameNum="<<frameNum;
        }
        else{
            std::vector<trackingObjectFeature> fv;
            PipeLine_Features(fv);
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

void PipeLineTracking::process(QString sourceData){
    ibgs=fac.getBgsAlgorithm(bgsType);

    std::vector<string> wins;
    wins.push_back("img_input");
    wins.push_back("img_fg");
//    wins.push_back("mask_i");
    wins.push_back("MultiCueBGS FG");
//    wins.push_back("featurePoint");
//    wins.push_back("binaryImage");
//    wins.push_back("matched feature");
//    wins.push_back("matched feature 2");
//    wins.push_back("matched feature 3");
//    wins.push_back("matched feature 4");
    yzbxlib::moveWindows(wins,3);
    int frameNum=0;

    while(1){
        PipeLine_Input(sourceData);
        if(img_input.empty()){
            break;
        }
        PipeLine_Bgs();
        if(img_fg.empty()){
            qWarning()<<"empty img_fg for frameNum="<<frameNum;
        }
        else{
            std::vector<trackingObjectFeature> fv;
            PipeLine_Features(fv);
            cv::imshow("img_input",img_input);
            cv::imshow("img_fg",img_fg);
            cv::waitKey(30);
            tracker.process(img_input,img_fg,fv);
//            vector<int> ids;
//            matcher.process(frameNum,dist_thres,fv,annTxt,ids);
//            PipeLine_DumpFV(frameNum,ids,fv);
        }
        frameNum++;
    }

//    PipeLine_Replay(sourceData,annTxt);
}

void PipeLineTracking::PipeLine_Bgs()
{
    cv::Mat fg;
    ibgs->process(img_input,fg,img_bg);
    if(fg.channels()==3){
        cvtColor(fg,img_fg,CV_BGR2GRAY);
    }
    else{
        img_fg=fg;
    }
}

void PipeLineTracking::PipeLine_Features(std::vector<trackingObjectFeature> &fv)
{
    bd.process(img_input,img_fg,fv);
}

void PipeLineTracking::PipeLine_Input(QString sourceData)
{
    cv::Mat input;
    fin.process(sourceData,input);
    if(input.channels()==3){
        img_input=input;
    }
    else{
        cvtColor(input,img_input,CV_GRAY2BGR);
    }
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

void PipeLineTracking::PipeLine_Replay(QString dataSource, QString replaySource, bool saveVideo){
    TrackingResultReplay replay;
    if(replaySource==""){
        replaySource="/home/yzbx/Downloads/BitSync/surveillance-video-system/UrbanTracker-Annotation/MOT2D2015/atrium.csv";
    }
    replay.process(dataSource,replaySource,bgsType,saveVideo);
}
