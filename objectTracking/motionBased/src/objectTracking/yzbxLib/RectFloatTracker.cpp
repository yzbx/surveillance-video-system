#include "RectFloatTracker.h"

RectFloatTracker::RectFloatTracker()
{
    globalChannel=3;
    maxListLength=2;
    NextTrackID=0;
    dt=0.2f;
    Accel_noise_mag=0.1f;

    //if (Cost[i + assignment[i] * N] > dist_thres)
    dist_thres = 100;

    maximum_allowed_skipped_frames = 100;
    max_trace_length=100;

    globalFirstDump=true;
    globalFirstOutput=true;
    frameNum=0;
    outputFileName="out.txt";
    FirstFG_frameNum=0;
}

void RectFloatTracker::tracking(const cv::Mat &img_input, const cv::Mat &img_fg)
{
    assert(!img_input.empty());
    assert(!img_fg.empty());

    //NOTE clone for save history! but this may not necessary for Mat
    this->img_input=img_input.clone();
    //the channel of img_input cannot change!
    if(img_input.channels()!=globalChannel){
        assert(frameNum==0);
        globalChannel=img_input.channels();
    }

    //convert img_fg to CV_8UC1 if necessary
    if(img_fg.channels()==3){
        cvtColor(img_fg,this->img_fg,CV_BGR2GRAY);
    }
    else{
        this->img_fg=img_fg.clone();
    }

    frameNum++;
    imageList.push_back(std::make_pair(this->img_input,this->img_fg));
    if(imageList.size()>maxListLength){
        imageList.pop_front();
    }
    //start()
    runInSingleThread();
}

void RectFloatTracker::runInSingleThread()
{
    std::vector<trackingObjectFeature> featureVector;
    blobDetector.getBlobFeature(img_input,img_fg,featureVector);
    assignments_t assignment=getHungarainAssignment(featureVector);
    featureVectorList.push_back(featureVector);
    if(featureVectorList.size()>maxListLength){
        featureVectorList.pop_front();
    }
    cv::Mat assignmentMat;
    getLocalFeatureAssignment(assignmentMat);
    doAssignment(assignment,featureVector,assignmentMat);
    showing(img_input,img_fg,featureVector);
}

void RectFloatTracker::run()
{
    runInSingleThread();
}

assignments_t RectFloatTracker::getHungarainAssignment(vector<trackingObjectFeature> &fv){
    if(fv.empty()){

    }
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

    size_t N = tracks.size();
    size_t M = fv.size();

    assignments_t assignment;

    if (!tracks.empty()&&!fv.empty())
    {
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
        for (size_t i = 0; i < assignment.size(); i++)
        {
            if (assignment[i] != -1)
            {
                if (Cost[i + assignment[i] * N] > dist_thres)
                {
                    assignment[i] = -1;
                    //                tracks[i]->skipped_frames = 1;
                    //                tracks[i]->skipped_frames++;
                }
            }
        }
    }
    else{
        assert(fv.empty());
        //the object in tracks all missing
        for(int i=0;i<tracks.size();i++){
            assignment.push_back(-1);
        }
    }

    return assignment;
}

void RectFloatTracker::doAssignment(assignments_t assignment,vector<trackingObjectFeature> &fv,cv::Mat assignmentMat){
    assert(assignment.size()==tracks.size());

    std::set<int> oldObjectId;
    // -----------------------------------
    // clean assignment from pairs with large distance
    // -----------------------------------
    for (size_t i = 0; i < assignment.size(); i++)
    {
        if (assignment[i] == -1)
        {
            // If track have no assigned detect, then increment skipped frames counter.
            if(!assignmentMat.empty()){
                for(int col=0;col<assignmentMat.cols;col++){
                    if(assignmentMat.at<uchar>(i,col)>=3){
                        assignment[i]=-1;
                        tracks[i]->skipped_frames = 0;
                        trackingObjectFeature of;
                        tracks[i]->predict(of);
                        tracks[i]->Update(of, true, max_trace_length);
                        //let long merge disppear
//                        tracks[i]->skipped_frames=0;
                        oldObjectId.insert(col);
                        break;
                    }
                }
            }
            tracks[i]->skipped_frames++;
        }
    }

    // -----------------------------------
    // If track didn't get detects long time, remove it.
    // -----------------------------------
    for (uint i = 0; i < tracks.size(); i++)
    {
        if (tracks[i]->skipped_frames > (int)maximum_allowed_skipped_frames)
        {
            outputAndRemove(i);
            tracks.erase(tracks.begin() + i);
            assignment.erase(assignment.begin() + i);
            i--;
        }
    }

    // -----------------------------------
    // Search for unassigned detects and start new tracks for them.
    // -----------------------------------
    for (size_t i = 0; i < fv.size(); ++i)
    {
        if (find(assignment.begin(), assignment.end(), i) == assignment.end())
        {
            //not match with the old objects.
            if(oldObjectId.find(i)==oldObjectId.end()){
                tracks.push_back(std::make_unique<singleObjectTracker>(fv[i], dt, Accel_noise_mag, NextTrackID++));
            }

            //not in the split of old objects 1->2
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

///remove tracks[i] in MergeAndSplitTree
void RectFloatTracker::outputAndRemove(uint index){
    //TODO remode node in MergeAndSplitTree
    if(tracks[index]->catch_frames>10){

    }
}

bool RectFloatTracker::isRectAInRectB(Rect_t A,Rect_t B){
    Point_t at=A.tl(),ab=A.br();
    Point_t bt=B.tl(),bb=B.br();

    Point_t dt=at-bt,db=ab-bb;
    int floatThreshold=10;
    int T=floatThreshold;
    if(dt.x>=-T&&db.x<=T&&dt.y>=-T&&db.y<=T){
        return true;
    }
    else{
        return false;
    }
}

void RectFloatTracker::showing(const cv::Mat &img_input,const cv::Mat &img_fg,std::vector<trackingObjectFeature> featureVector){
    (void)img_fg;
    cv::Scalar Colors[] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255), cv::Scalar(255, 127, 255), cv::Scalar(127, 0, 255), cv::Scalar(127, 0, 127) };

    cv::Mat img_tracking=img_input.clone();
    for (uint i=0;i<featureVector.size();i++)
    {
        cv::circle(img_tracking, featureVector[i].pos, 3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

    for (uint i = 0; i < tracks.size(); i++)
    {
        // void polylines(InputOutputArray img, InputArrayOfArrays pts, bool isClosed, const Scalar& color, int thickness=1, int lineType=8, int shift=0 )
//        cv::polylines(img_tracking,tracks[i]->trace,false,Colors[tracks[i]->track_id %9],2,CV_AA);
        if (tracks[i]->trace.size() > 1)
        {
            for (uint j = 0; j < tracks[i]->trace.size() - 1; j++)
            {
                cv::line(img_tracking, tracks[i]->trace[j], tracks[i]->trace[j + 1], Colors[tracks[i]->track_id % 9], 2, CV_AA);
            }
        }

        Rect_t r=tracks[i]->feature->rect;
        cv::rectangle(img_tracking,r,Colors[tracks[i]->track_id % 9]);
        std::string text;
        if(tracks[i]->status==NEW_STATUS)   text="new";
        else if(tracks[i]->status==MISSING_STATUS) text="missing";
        else if(tracks[i]->status==NORMAL_STATUS) text="normal";
        else text="error";

        std::string numstr=boost::lexical_cast<string>(tracks[i]->track_id);
        text=numstr+" : "+text;


        cv::putText(img_tracking, text, r.tl(), FONT_HERSHEY_COMPLEX, 0.5,
                    cv::Scalar(0,0,255), 2, 8);
    }

    cv::namedWindow("img_tracking",WINDOW_NORMAL);
    cv::imshow("img_tracking",img_tracking);
}

track_t RectFloatTracker::calcPathWeight(std::shared_ptr<trackingObjectFeature> of1,std::shared_ptr<trackingObjectFeature> of2,ObjectLocalFeatureMatch &matcher){
    track_t pathWeight=0.0;
    vector<DMatch> &good_matches=matcher.global_good_matches;
    for(auto match=good_matches.begin();match!=good_matches.end();match++){
        assert(match->queryIdx<matcher.global_keypoints_1.size());
        assert(match->trainIdx<matcher.global_keypoints_2.size());
        KeyPoint &kp1=matcher.global_keypoints_1[match->queryIdx];
        KeyPoint &kp2=matcher.global_keypoints_2[match->trainIdx];

        //test KeyPoint's position in trackingObjectFeature's rect
        if(isPointInRect(kp1.pt,of1->rect)&&isPointInRect(kp2.pt,of2->rect)){
            pathWeight+=1.0;
        }
    }
    return pathWeight;
}

void RectFloatTracker::getLocalFeatureAssignment(cv::Mat &assignmentMat){
    assert(featureVectorList.size()==imageList.size());
    if(featureVectorList.size()<2){
        return;
    }

    int m=featureVectorList.front().size();
    int n=featureVectorList.back().size();
    if(m==0||n==0){
        return;
    }

    std::vector<trackingObjectFeature> &fv1=featureVectorList.front();
    std::vector<trackingObjectFeature> &fv2=featureVectorList.back();
    if(!assignmentMat.empty()) assignmentMat.release();
    assignmentMat.create(m,n,CV_8UC1);

    ObjectLocalFeatureMatch matcher;
    std::pair<cv::Mat,cv::Mat> &p1=imageList.front();
    std::pair<cv::Mat,cv::Mat> &p2=imageList.back();
    matcher.getGoodMatches(p1.first,p1.second,p2.first,p2.second);

    for(int i=0;i<m;i++){
        for(int j=0;j<n;j++){
            assignmentMat.at<uchar>(i,j)=calcPathWeight(std::make_shared<trackingObjectFeature>(fv1[i]),\
                                                        std::make_shared<trackingObjectFeature>(fv2[j]),matcher);
        }
    }
}

bool RectFloatTracker::isPointInRect(cv::Point2f p, Rect_t rect)
{
    if(p.x>rect.x&&p.x>rect.y&&p.x<rect.x+rect.width&&p.y<rect.y+rect.height){
        return true;
    }
    else{
        return false;
    }
}
