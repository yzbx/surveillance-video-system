#include "multiobjecttracking.h"

MultiObjectTracking::MultiObjectTracking()
{
    m_minObjectSize.width=10;
    m_minObjectSize.height=10;
}

void MultiObjectTracking::process(QString configFile, QString videoFile, TrackingStatus *status)
{
    qDebug()<<"MultiObjectTracking .................";
    IBGS *ibgs;
    QYzbxTrackingFeature *maskFeature=new QYzbxTrackingFeature;
    maskFeature=new QYzbxTrackingFeature;

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(configFile.toStdString(),pt);
    QString BGSType=QString::fromStdString(pt.get<std::string>("General.BGSType"));
    bgsFactory_yzbx bgsFac;

    if(status==NULL){
        qDebug()<<"wait to implement";
        exit(-1);
    }
    else{
        if(BGSType!=status->BGSType){
            delete status->ibgs;
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
    this->start();
}

vector<trackingObjectFeature> MultiObjectTracking::getObjects(const cv::Mat &img_input,const cv::Mat &fgMask){
    vector<trackingObjectFeature> featureVector;

    Mat fgMaskC1;
    if(fgMask.type()==CV_8UC3){
        cvtColor(fgMask,fgMaskC1,CV_BGR2GRAY);
    }
    else{
        fgMaskC1=fgMask;
    }
    cv::medianBlur(fgMaskC1,fgMaskC1,5);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::Mat edges;
    cv::Canny(fgMaskC1, edges, 50, 190, 3);
    cv::findContours(edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, cv::Point());
    if (contours.size() > 0)
    {
        for (size_t i = 0; i < contours.size(); i++)
        {
            cv::Rect r = cv::boundingRect(contours[i]);
//            cv::Rect r = cv::minAreaRect(contours[i]);


            if (r.width >= m_minObjectSize.width &&
                    r.height >= m_minObjectSize.height)
            {
                trackingObjectFeature fea;
                fea.rect=r;
                fea.pos=((r.br() + r.tl())*0.5);
                fea.size=r.width*r.height;
                featureVector.push_back(fea);
            }
        }
    }

    return featureVector;
}

void MultiObjectTracking::Tracking(vector<trackingObjectFeature> &fv){
    // -----------------------------------
    // If there is no tracks yet, then every cv::Point begins its own track.
    // -----------------------------------
    if (tracks.size() == 0)
    {
        // If no tracks yet
        for (size_t i = 0; i < fv.size(); ++i)
        {
            tracks.push_back(std::make_unique<singleObjectTracker>(fv[i], dt, Accel_noise_mag, NextTrackID++));
        }
    }

    size_t N = tracks.size();		// треки
    size_t M = fv.size();	// детекты

    assignments_t assignment; // назначения

    if (!tracks.empty())
    {
        // Матрица расстояний от N-ного трека до M-ного детекта.
        distMatrix_t Cost(N * M);

        for (size_t i = 0; i < tracks.size(); i++)
        {
            for (size_t j = 0; j < fv.size(); j++)
            {
                Cost[i + j * N] = tracks[i]->CalcDist(fv[j]);
            }
        }

        // -----------------------------------
        // Solving assignment problem (tracks and predictions of Kalman filter)
        // -----------------------------------
        AssignmentProblemSolver APS;
        APS.Solve(Cost, N, M, assignment, AssignmentProblemSolver::optimal);

        // -----------------------------------
        // clean assignment from pairs with large distance
        // -----------------------------------
        for (size_t i = 0; i < assignment.size(); i++)
        {
            if (assignment[i] != -1)
            {
                if (Cost[i + assignment[i] * N] > dist_thres)
                {
                    assignment[i] = -1;
                    tracks[i]->skipped_frames = 1;
                }
            }
            else
            {
                // If track have no assigned detect, then increment skipped frames counter.
                tracks[i]->skipped_frames++;
            }
        }

        // -----------------------------------
        // If track didn't get detects long time, remove it.
        // -----------------------------------
        for (int i = 0; i < static_cast<int>(tracks.size()); i++)
        {
            if (tracks[i]->skipped_frames > maximum_allowed_skipped_frames)
            {
                tracks.erase(tracks.begin() + i);
                assignment.erase(assignment.begin() + i);
                i--;
            }
        }
    }

    // -----------------------------------
    // Search for unassigned detects and start new tracks for them.
    // -----------------------------------
    for (size_t i = 0; i < fv.size(); ++i)
    {
        if (find(assignment.begin(), assignment.end(), i) == assignment.end())
        {
            tracks.push_back(std::make_unique<singleObjectTracker>(fv[i], dt, Accel_noise_mag, NextTrackID++));
        }
    }

    // Update Kalman Filters state

    for (size_t i = 0; i<assignment.size(); i++)
    {
        // If track updated less than one time, than filter state is not correct.

        if (assignment[i] != -1) // If we have assigned detect, then update using its coordinates,
        {
            tracks[i]->skipped_frames = 0;
            tracks[i]->Update(fv[assignment[i]], true, max_trace_length);
        }
        else				     // if not continue using predictions
        {
            trackingObjectFeature nullOf;
            tracks[i]->Update(nullOf, false, max_trace_length);
        }
    }
}

void MultiObjectTracking::processOne(const Mat &img_input, Mat &img_foreground, Mat &img_background, TrackingStatus *status)
{
    cv::Scalar Colors[] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255), cv::Scalar(255, 127, 255), cv::Scalar(127, 0, 255), cv::Scalar(127, 0, 127) };

    status->ibgs->process(img_input,img_foreground,img_background);
    if(!img_foreground.empty()){
        vector<trackingObjectFeature> featureVector=getObjects(img_input,img_foreground);

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

void MultiObjectTracking::run()
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

void MultiObjectTracking::stop()
{
    globalStop=true;
}

MultiObjectTracking::~MultiObjectTracking()
{

}
