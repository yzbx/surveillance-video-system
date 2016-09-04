#include "TrackingBlobsMatchAnnotation.h"

TrackingBlobsMatchAnnotation::TrackingBlobsMatchAnnotation()
{

}

void TrackingBlobsMatchAnnotation::process(int frameNum,track_t dist_thres,std::vector<trackingObjectFeature> &fv, QString annotationTxt,std::vector<int> &ids)
{
    assignments_t assignment;
    if(!assignment.empty()){
        assignment.clear();
    }
    if(!ids.empty()){
        ids.clear();
    }

    QFile file(annotationTxt);
    if(!file.open(QIODevice::ReadOnly)){
        assert(false);
    }

    QTextStream in(&file);
    std::vector<std::pair<int,Rect_t>> ann;
    while(!in.atEnd()){
        QString line=in.readLine();
        QStringList numstr=line.split(",",QString::SkipEmptyParts);
        int fn=numstr[0].toInt();
        if(fn<frameNum){
            continue;
        }
        else if(fn>frameNum){
            break;
        }

        int id=numstr[1].toInt();
        track_t rect[4];
        for(int i=0;i<4;i++){
            rect[i]=numstr[2+i].toFloat();
        }
        Rect_t r(rect[0],rect[1],rect[2],rect[3]);
        ann.push_back(std::make_pair(id,r));
    }

    int N=ann.size();
    int M=fv.size();

    distMatrix_t Cost(N * M);

    for (size_t i = 0; i < N; i++)
    {
        for (size_t j = 0; j < M; j++)
        {
            Cost[i + j * N] =getRectDistance(ann[i].second,fv[j].rect);
        }
    }


    // -----------------------------------
    // Solving assignment problem (tracks and predictions of Kalman filter)
    // -----------------------------------
    AssignmentProblemSolver APS;
    APS.Solve(Cost, N, M, assignment, AssignmentProblemSolver::optimal);
    assert(N==assignment.size());
    for (size_t i = 0; i < assignment.size(); i++)
    {
        if (assignment[i] != -1)
        {
            if (Cost[i + assignment[i] * N] > dist_thres)
            {
                assignment[i] = -1;
            }
        }
    }

    for(size_t i=0;i<fv.size();i++){
        auto it=std::find(assignment.begin(),assignment.end(),i);
        if(it==assignment.end()){
            ids.push_back(-1);
        }
        else{
            int index=it-assignment.begin();
            ids.push_back(ann[index].first);
        }
    }
}

track_t TrackingBlobsMatchAnnotation::getRectDistance(Rect_t blob_r,Rect_t annotation_r){
    Point_t blob_center=(blob_r.br()+blob_r.tl())*0.5f;
    Point_t annotation_center=(annotation_r.br()+annotation_r.tl())*0.5f;
    Rect_t &br=blob_r;
    Rect_t &ar=annotation_r;
    track_t dist=cv::norm(blob_center-annotation_center)+std::abs(br.width-ar.width)+std::abs(br.height-ar.height);
    return dist;
}
