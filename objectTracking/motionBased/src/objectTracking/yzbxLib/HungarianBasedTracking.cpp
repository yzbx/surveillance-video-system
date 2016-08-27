#include "HungarianBasedTracking.h"

void HungarianBasedTracking::run(){
    std::vector<trackingObjectFeature> featureVector;
    blobDetector.getBlobFeature(img_fg,img_fg,featureVector);
    hungarianTracking(featureVector);
    dump();
    showing(img_input,img_fg,featureVector);
}

void HungarianBasedTracking::tracking(const cv::Mat &img_input,const cv::Mat &img_fg){
    this->img_input=img_input;
    this->img_fg=img_fg;
    frameNum++;
    start();
}

void HungarianBasedTracking::showing(const cv::Mat &img_input,const cv::Mat &img_fg,std::vector<trackingObjectFeature> featureVector){
    (void)img_fg;
    cv::Scalar Colors[] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255), cv::Scalar(255, 127, 255), cv::Scalar(127, 0, 255), cv::Scalar(127, 0, 127) };

    cv::Mat img_tracking=img_input.clone();
    for (uint i=0;i<featureVector.size();i++)
    {
        cv::circle(img_tracking, featureVector[i].pos, 3, cv::Scalar(0, 255, 0), 1, CV_AA);
    }

    for (uint i = 0; i < tracks.size(); i++)
    {
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

        cv::putText(img_tracking, text, r.tl(), FONT_HERSHEY_COMPLEX, 0.5,
                cv::Scalar(0,0,255), 2, 8);
    }

    imshow("img_tracking",img_tracking);
}

void HungarianBasedTracking::hungarianTracking(vector<trackingObjectFeature> &fv){
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
        for (uint i = 0; i < tracks.size(); i++)
        {
            if (tracks[i]->skipped_frames > (int)maximum_allowed_skipped_frames)
            {
                if(tracks[i]->catch_frames>10){
                    output(i);
                }
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
