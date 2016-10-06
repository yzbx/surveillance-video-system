#include "motionbasedtracker.h"

MotionBasedTracker::MotionBasedTracker()
{

}

void MotionBasedTracker::process(QString configFile, QString videoFile, TrackingStatus *status)
{
    qDebug()<<"MotionBasedTracking .................";
//    maskFeature=new QYzbxTrackingFeature;

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

void MotionBasedTracker::processOne(const Mat &img_input, Mat &img_foreground, Mat &img_background, TrackingStatus *status)
{
    cv::Scalar Colors[] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255), cv::Scalar(255, 127, 255), cv::Scalar(127, 0, 255), cv::Scalar(127, 0, 127) };

    status->ibgs->process(img_input,img_foreground,img_background);
    QYzbxTrackingFeature *trackingFeature=new QYzbxTrackingFeature;
    FrameFeature frameFeature;
    frameFeature.frameNum=status->frameinput.frameNum;
    if(!img_foreground.empty()){
        //        vector<Mat> objects;
        //        objects=maskFeature->getObjectsFromMask(img_foreground,true);
        trackingFeature->getObjects(img_input,img_foreground,frameFeature);
        //        status->frameFeatureList.push_back(frameFeature);

        //        if(status->frameFeatureList.size()>status->trackingWindowSize){
        //            status->frameFeatureList.pop_front();
        //        }

        //        TrackingObjectAssociation toa;
        //        toa.process(status);

        qDebug()<<"dump frameFeature *********************************";
        for(uint i=0;i<frameFeature.features.size();i++){
            std::cout<<"["<<frameFeature.features[i].pos<<" "<<frameFeature.features[i].size<<"],";
        }
        std::cout<<std::endl;

        singleFrameTracking(frameFeature);

        cv::Mat img_tracking=img_input.clone();
        for (uint i=0;i<frameFeature.features.size();i++)
        {
            cv::circle(img_tracking, frameFeature.features[i].pos, 3, cv::Scalar(0, 255, 0), 1, CV_AA);
        }


        for (uint i = 0; i < tracks.size(); i++)
        {
            if (tracks[i]->trace.size() > 1)
            {
                for (uint j = 0; j < tracks[i]->trace.size() - 1; j++)
                {
                    cv::line(img_tracking, tracks[i]->trace[j], tracks[i]->trace[j + 1], Colors[tracks[i]->TrackID % 9], 2, CV_AA);
                }
            }
        }

        imshow("img_tracking",img_tracking);
    }

}

void MotionBasedTracker::objectTracking(DirectedGraph &g)
{
    (void)g;
}

void MotionBasedTracker::singleFrameTracking(FrameFeature &ff)
{
    //init
    if (tracks.size() == 0)
    {
        // If no tracks yet
        for (size_t i = 0; i < ff.features.size(); ++i)
        {
            tracks.push_back(std::make_unique<singleFrameTracker>(ff.features[i],NextTrackID++));
        }
    }

    size_t N = tracks.size();
    size_t M = ff.features.size();

    assignments_t assignment;

    if (!tracks.empty()&&!ff.features.empty())
    {
        //create and init dist matrix/cost matrix.
        distMatrix_t Cost(N * M);

        std::cout<<"M="<<M<<" N="<<N<<std::endl;
        std::cout<<"dist matrix is: "<<std::endl;

        for (size_t i = 0; i < tracks.size(); i++)
        {
            for (size_t j = 0; j < ff.features.size(); j++)
            {
                Cost[i + j * N] = tracks[i]->CalcDistance(ff.features[j]);
                std::cout<<Cost[i+j*N]<<" ";
            }
            std::cout<<std::endl;
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
                if (Cost[i + assignment[i] * N] > Distance_Threshold)
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

        qDebug()<<"tracks.size()="<<tracks.size();
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
    for (size_t i = 0; i < ff.features.size(); ++i)
    {
        if (find(assignment.begin(), assignment.end(), i) == assignment.end())
        {
            tracks.push_back(std::make_unique<singleFrameTracker>(ff.features[i],NextTrackID++));
        }
    }

    // Update Kalman Filters state
    if(ff.features.empty()){
        for(uint i=0;i<tracks.size();i++){
            assignment.push_back(-1);
        }
    }

    for (size_t i = 0; i<assignment.size(); i++)
    {
        // If track updated less than one time, than filter state is not correct.

        if (assignment[i] != -1) // If we have assigned detect, then update using its coordinates,
        {
            tracks[i]->skipped_frames = 0;
            tracks[i]->Update(ff.features[assignment[i]], true, max_trace_length);
        }
        else				     // if not continue using predictions
        {
            ObjectFeature of;
            tracks[i]->Update(of,false, max_trace_length);
        }
    }
}

void MotionBasedTracker::initBgs(TrackingStatus *status)
{
    status->frameinput.initBgs(status->ibgs,status->initFrameNum);
    status->bgsInited=true;
}

void MotionBasedTracker::run()
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

void MotionBasedTracker::stop()
{
    globalStop=true;
}

void singleFrameTracker::Update(ObjectFeature &of, bool dataCorrect, size_t max_trace_length)
{
    //update pos kalman
    Mat prediction = pos_kalman.predict();
    cv::Point2f predictPt(prediction.at<float>(0),prediction.at<float>(1));
    cv::Mat measurement(2,1,CV_32FC1);
    if(!dataCorrect){
        measurement.at<float>(0)=predictPt.x;
        measurement.at<float>(1)=predictPt.y;
    }
    else{
        measurement.at<float>(0) =of.pos.x;
        measurement.at<float>(1) =of.pos.y;
        this->objectFeature->pos=of.pos;
    }
    cv::Mat estiMated = pos_kalman.correct(measurement);
    cv::Point2f correctPt(estiMated.at<float>(0),estiMated.at<float>(0));
    this->objectFeature->pos_predict= correctPt;

    //update trace
    if (trace.size() > max_trace_length)
    {
        trace.erase(trace.begin(), trace.end() - max_trace_length);
    }

    trace.push_back(of.pos_predict);
}

float singleFrameTracker::CalcDistance(ObjectFeature &b)
{
    cv::Point2d pa=this->objectFeature->pos_predict;
    cv::Point2d pb=b.pos;
    float sa=this->objectFeature->size;
    float sb=b.size;

    qDebug()<<"CalcDistance: ";
    qDebug()<<pa.x<<pa.y<<sa;
    qDebug()<<pb.x<<pb.y<<sb;
    float dx=(pa.x-pb.x)/sa;
    float dy=(pa.y-pb.y)/sa;
    float posDist=sqrt(dx*dx+dy*dy);
    float sizeDist=max(sa/sb,sb/sa)-1;

    float alpha=0.5;
    float weight=alpha*posDist+(1-alpha)*sizeDist;

    if(weight<=0||sa==0||sb==0){
        qDebug()<<"what's the fuck!!!";
    }

    return weight;
}

singleFrameTracker::singleFrameTracker(ObjectFeature &of, int id)
{
    TrackID=id;
    this->objectFeature=&of;
    objectFeature->pos_predict=objectFeature->pos;
    objectFeature->size=objectFeature->size;

    // intialization of KF...
    pos_kalman.init(4,2,0);
    pos_kalman.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);

    pos_kalman.statePre.at<float>(0) = of.pos.x;
    pos_kalman.statePre.at<float>(1) = of.pos.y;
    pos_kalman.statePre.at<float>(2) = 0;
    pos_kalman.statePre.at<float>(3) = 0;
    setIdentity(pos_kalman.measurementMatrix);
    setIdentity(pos_kalman.processNoiseCov, Scalar::all(1e-4));
    setIdentity(pos_kalman.measurementNoiseCov, Scalar::all(10));
    setIdentity(pos_kalman.errorCovPost, Scalar::all(.1));

    trace.push_back(of.pos);
}

singleFrameTracker::~singleFrameTracker()
{

}
