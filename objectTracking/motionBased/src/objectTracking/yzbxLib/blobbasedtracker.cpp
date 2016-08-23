#include "blobbasedtracker.h"


BlobBasedTracker::BlobBasedTracker()
{
    //tracks.push_back(std::make_unique<singleObjectTracker>(fv[i], dt, Accel_noise_mag, NextTrackID++));
    //KF(of.pos, dt, Accel_noise_mag)
    //the ID for every singleObjectTracker
    NextTrackID=0;
    dt=0.2f;
    Accel_noise_mag=0.1f;


    //no use m_minObjectSize in BlobBasedTracker.
    m_minObjectSize.width=0;
    m_minObjectSize.height=0;

    //if (Cost[i + assignment[i] * N] > dist_thres)
    dist_thres = 60;

    maximum_allowed_skipped_frames = 10;
    max_trace_length=100;

}

void BlobBasedTracker::process(QString configFile, QString videoFile, TrackingStatus *status)
{
    qDebug()<<"BlobBasedTracker .................";

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(configFile.toStdString(),pt);
    QString BGSType=QString::fromStdString(pt.get<std::string>("General.BGSType"));
    bgsFactory_yzbx bgsFac;

    if(status==NULL){
        qDebug()<<"wait to implement";
        exit(-1);
    }
    else{
        if(status->ibgs==NULL||BGSType!=status->BGSType){
            if(status->ibgs!=NULL) delete status->ibgs;
            status->BGSType=BGSType;
            status->ibgs=bgsFac.getBgsAlgorithm(BGSType);
            status->frameinput.init(videoFile);
            status->frameinput.initBgs(status->ibgs,status->initFrameNum);
            status->bgsInited=true;
        }

        cv::Mat img_input,img_foreground,img_background;
        status->frameinput.getNextFrame(videoFile,img_input);
        processOne(img_input,img_foreground,img_background,status);
    }

    globalTrackingStatus=status;
    qDebug()<<"start new process........................................................";
    this->start();
}

void BlobBasedTracker::processOne(const Mat &img_input, Mat &img_foreground, Mat &img_background, TrackingStatus *status)
{
    qDebug()<<"use BlobBasedTracker::processOne";
    cv::Scalar Colors[] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255), cv::Scalar(255, 127, 255), cv::Scalar(127, 0, 255), cv::Scalar(127, 0, 127) };

    status->ibgs->process(img_input,img_foreground,img_background);
    if(!img_foreground.empty()){
        vector<trackingObjectFeature> featureVector;
        if(img_foreground.channels()==3){
            cvtColor(img_foreground,img_foreground,CV_BGR2GRAY);
        }
        blobDetector.getBlobFeature(img_input,img_foreground,featureVector);
        //use the Tracking() function from MultiObjectTracking by inherit.
        Tracking(featureVector);

        cv::Mat img_tracking=img_input.clone();
        for (int i=0;i<featureVector.size();i++)
        {
            cv::circle(img_tracking, featureVector[i].pos, 3, cv::Scalar(0, 255, 0), 1, CV_AA);
        }


        for (int i = 0; i < tracks.size(); i++)
        {
            if (tracks[i]->trace.size() > 1)
            {
                for (int j = 0; j < tracks[i]->trace.size() - 1; j++)
                {
                    cv::line(img_tracking, tracks[i]->trace[j], tracks[i]->trace[j + 1], Colors[tracks[i]->track_id % 9], 2, CV_AA);
                }
            }
        }

        imshow("img_tracking",img_tracking);
    }
}

BlobBasedTracker::~BlobBasedTracker()
{

}

void BlobBasedTracker::run()
{
    QString videoFilePath=globalTrackingStatus->frameinput.videoFilePath;
    while(!globalStop){
        cv::Mat img_input,img_foreground,img_background;
        globalTrackingStatus->frameinput.getNextFrame(videoFilePath,img_input);
        if(img_input.empty()){
            break;
        }

        processOne(img_input,img_foreground,img_background,globalTrackingStatus);
    }
    globalStop=false;
}
