#include "RectFloatTracker.h"

RectFloatTracker::RectFloatTracker()
{
    globalChannel=3;
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
    //start()
    runInSingleThread();
}

void RectFloatTracker::runInSingleThread()
{
    std::vector<trackingObjectFeature> featureVector;
    blobDetector.getBlobFeature(img_input,img_fg,featureVector);
    assignments_t assignment=getHungarainAssignment(featureVector);
    doAssignment(assignment,featureVector);
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

void RectFloatTracker::doAssignment(assignments_t assignment,vector<trackingObjectFeature> &fv){
    assert(assignment.size()==tracks.size());

    // -----------------------------------
    // clean assignment from pairs with large distance
    // -----------------------------------
    for (size_t i = 0; i < assignment.size(); i++)
    {
        if (assignment[i] == -1)
        {
            // If track have no assigned detect, then increment skipped frames counter.
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

///remove tracks[i] in MergeAndSplitTree
void RectFloatTracker::outputAndRemove(uint index){
    //TODO remode node in MergeAndSplitTree
    if(tracks[index]->catch_frames>10){

    }
}
