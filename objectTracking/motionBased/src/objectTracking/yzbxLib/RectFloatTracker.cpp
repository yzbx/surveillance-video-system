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

void RectFloatTracker::process(const Mat &img_input, const Mat &img_fg,vector<trackingObjectFeature> &fv)
{
    imageList.push_back(std::make_pair(img_input,img_fg));
    if(imageList.size()>maxListLength){
        imageList.pop_front();
    }
    featureVectorList.push_back(fv);
    if(featureVectorList.size()>maxListLength){
        featureVectorList.pop_front();
    }

    ///Hungarain->LIF, without split/merge support
    //    assignments_t assignment;
    //    getHungarainAssignment(assignment);
    //    showAssignment(assignment,fv);

    //    std::cout<<"assignment= ";
    //    for(auto it=assignment.begin();it!=assignment.end();it++){
    //        std::cout<<*it<<" ,";
    //    }
    //    std::cout<<std::endl;
    //    cv::Mat matchMat;
    //    getLocalFeatureAssignment(matchMat);
    //    doAssignment(assignment,fv,matchMat);

    ///LIF->Hungarian, with split/merge support
    cv::Mat matchMat;
    getLocalFeatureAssignment(matchMat);
    getUnmatchedHungarainAssignment(matchMat);
    doAssignment();
    showing(img_input,img_fg,fv);
}

void RectFloatTracker::getUnmatchedHungarainAssignment(cv::Mat matchMat){
    bool emptyBlobs;

    if(featureVectorList.empty()){
        emptyBlobs=true;
    }
    else
    {
        vector<trackingObjectFeature> &fv=featureVectorList.back();
        if(fv.empty()){
            emptyBlobs=true;
        }
        else{
            emptyBlobs=false;
        }
    }

    bool emptyObjects;
    if(tracks.empty()){
        emptyObjects=true;
    }
    else{
        emptyObjects=false;
    }

    if(!matchedBlob.empty())matchedBlob.clear();
    if(!matchedObject.empty())matchedObject.clear();
    if(!ObjectToBlob.empty())ObjectToBlob.clear();
    if(!BlobToObject.empty())BlobToObject.clear();

    if(emptyBlobs||emptyObjects){
        qDebug()<<"empyt blobs or objects";
    }
    else{
        int m=matchMat.rows;
        int n=matchMat.cols;
        assert(tracks.size()==m);
        vector<trackingObjectFeature> &fv=featureVectorList.back();
        assert(fv.size()==n);

        for(int i=0;i<m;i++){
            for(int j=0;j<n;j++){
                uchar featureNum=matchMat.at<uchar>(i,j);
                //local feature point for stable match!
                if(featureNum>3){
                    ObjectToBlob[i].insert(j);
                    BlobToObject[j].insert(i);
                    matchedBlob.insert(j);
                    matchedObject.insert(i);
                }
            }
        }

        //hungarian for unstable match
        int unMatchedObjectNum=m-matchedObject.size();
        int unMatchedBlobNum=n-matchedBlob.size();
        if(unMatchedBlobNum==0||unMatchedObjectNum==0){
            qDebug()<<"nothing for unmatched hungarain";
        }
        else{
            //vector<trackIdx>
            vector<int> unMatchedObjects;
            //vector<featureVectorIdx>
            vector<int> unMatchedBlobs;
            for(int i=0;i<m;i++){
                if(matchedObject.find(i)==matchedObject.end()){
                    unMatchedObjects.push_back(i);
                }
            }
            for(int j=0;j<n;j++){
                if(matchedBlob.find(j)==matchedBlob.end()){
                    unMatchedBlobs.push_back(j);
                }
            }

            size_t N = unMatchedObjects.size();
            size_t M = unMatchedBlobs.size();

            distMatrix_t Cost(N * M);
            for(int i=0;i<unMatchedObjects.size();i++){
                for(int j=0;j<unMatchedBlobs.size();j++){
                    Cost[i+j*N]=calcCost(std::make_shared<trackingObjectFeature>(*(tracks[unMatchedObjects[i]]->feature)),
                            std::make_shared<trackingObjectFeature>(fv[unMatchedBlobs[j]]),RectDist);
                }
            }

            AssignmentProblemSolver APS;
            assignments_t unMatchedAssignment;
            APS.Solve(Cost, N, M, unMatchedAssignment, AssignmentProblemSolver::optimal);
            for (size_t i = 0; i < unMatchedAssignment.size(); i++)
            {
                if (unMatchedAssignment[i] != -1)
                {
                    if (Cost[i + unMatchedAssignment[i] * N] > dist_thres)
                    {
                        unMatchedAssignment[i] = -1;
                    }
                    else{
                        int trackIdx=unMatchedObjects[i];
                        int featureVectorIdx=unMatchedBlobs[unMatchedAssignment[i]];
                        ObjectToBlob[trackIdx].insert(featureVectorIdx);
                        BlobToObject[featureVectorIdx].insert(trackIdx);
                        matchedBlob.insert(featureVectorIdx);
                        matchedObject.insert(trackIdx);
                    }
                }
            }

        }
    }

}

void RectFloatTracker::getHungarainAssignment(assignments_t &assignment,int costType){
    bool emptyBlobs;

    if(featureVectorList.empty()){
        emptyBlobs=true;
    }
    else
    {
        vector<trackingObjectFeature> &fv=featureVectorList.back();
        if(fv.empty()){
            emptyBlobs=true;
        }
        else{
            emptyBlobs=false;
        }
    }

    bool emptyObjects;
    if(tracks.empty()){
        emptyObjects=true;
    }
    else{
        emptyObjects=false;
    }

    if(!assignment.empty()){
        assignment.clear();
    }

    if(emptyBlobs&&emptyObjects){
        qDebug()<<"empyt blobs and objects";
    }
    else if(emptyBlobs){
        //update missed objects

        for(size_t i=0;i<tracks.size();i++){
            assignment.push_back(-1);
        }
    }
    else if(emptyObjects){
        //init objects
        vector<trackingObjectFeature> &fv=featureVectorList.back();
        for (size_t i = 0; i < fv.size(); ++i)
        {
            tracks.push_back(std::make_unique<singleObjectTracker>(fv[i], dt, Accel_noise_mag, NextTrackID++));
            assignment.push_back(i);
        }
    }
    else{
        vector<trackingObjectFeature> &fv=featureVectorList.back();
        size_t N = tracks.size();
        size_t M = fv.size();

        distMatrix_t Cost(N * M);

        for (size_t i = 0; i < tracks.size(); i++)
        {
            for (size_t j = 0; j < fv.size(); j++)
            {
                //                Cost[i + j * N] = tracks[i]->CalcDist(fv[j]);
                Cost[i+j*N]=calcCost(std::make_shared<trackingObjectFeature>(*(tracks[i]->feature)),
                                     std::make_shared<trackingObjectFeature>(fv[j]),costType);
            }
        }

        qDebug()<<"Cost =";
        yzbxlib::dumpVector(Cost);
        qDebug()<<"dist_thres="<<dist_thres;

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
                }
            }
        }
    }
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
    cv::Mat costMat;
    getLocalFeatureAssignment(costMat);
    std::cout << "costMat = "<< std::endl << " "  << costMat << std::endl << std::endl;

    assignments_t assignment=getHungarainAssignment(featureVector);
    showAssignment(assignment,featureVector);

    std::cout<<"assignment= ";
    for(auto it=assignment.begin();it!=assignment.end();it++){
        std::cout<<*it<<" ,";
    }
    std::cout<<std::endl;



    doAssignment(assignment,featureVector,costMat);
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

        qDebug()<<"Cost =";
        yzbxlib::dumpVector(Cost);
        qDebug()<<"dist_thres="<<dist_thres;

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

void RectFloatTracker::doAssignment(){
    bool emptyBlobs;

    if(featureVectorList.empty()){
        emptyBlobs=true;
    }
    else
    {
        vector<trackingObjectFeature> &fv=featureVectorList.back();
        if(fv.empty()){
            emptyBlobs=true;
        }
        else{
            emptyBlobs=false;
        }
    }

    bool emptyObjects;
    if(tracks.empty()){
        emptyObjects=true;
    }
    else{
        emptyObjects=false;
    }

    if(emptyBlobs&&emptyObjects){
        qDebug()<<"empty Blobs and empty objects";
    }
    else if(emptyBlobs){
        //handle missed object
        for(int i=0;i<tracks.size();i++){
            mUnmatchObjects.insert(i);
        }
        handleMissedObjects();
    }
    else if(emptyObjects){
        //handle new object
        vector<trackingObjectFeature> &fv=featureVectorList.back();
        for(int i=0;i<fv.size();i++){
            mNewBlobs.insert(i);
        }
        handleNewObjects();
    }
    else{
        for(auto ia=ObjectToBlob.begin();ia!=ObjectToBlob.end();ia++){
            //tracksIdx
            int trackIdx=ia->first;
            if(ia->second.size()==1){
                int fvIdx=*(ia->second.begin());
                if(BlobToObject[fvIdx].size()==1){
                    mOneToOne.insert(std::make_pair(trackIdx,fvIdx));
                }
                else{
                    mNToOne.insert(std::make_pair(BlobToObject[fvIdx],fvIdx));
                }
            }
            else{
                mOneToN.insert(std::make_pair(trackIdx,ia->second));
            }
        }

        vector<trackingObjectFeature> &fv=featureVectorList.back();
        //handle new object
        for(int i=0;i<fv.size();i++){
            if(matchedBlob.find(i)==matchedBlob.end()){
                mNewBlobs.insert(i);
            }
        }
        handleNewObjects();
        //handle missed object
        for(int i=0;i<tracks.size();i++){
            if(matchedObject.find(i)==matchedObject.end()){
                mUnmatchObjects.insert(i);
            }
        }
        handleMissedObjects();
        //handle one to one object
        handleOneToOneObjects();
        //handle merged object
        handleNToOneObjects();
        //handle split object
        handleOneToNObjects();

        //remove objects which need delete!!!

        for(int i=0;i<tracks.size();i++){
            int id=tracks[i]->track_id;
            auto it=deleteLaterObjects.find(id);
            if(it!=deleteLaterObjects.end()){
                tracks.erase(tracks.begin()+i);
                i--;

                //for split
                objectSplitProvocation.erase(id);
                //for merge
                for(auto mergeIt=objectMergeProvocation.begin();mergeIt!=objectMergeProvocation.end();mergeIt++){
                    int friendId=mergeIt->first;
                    std::map<int,int> &friendMap=mergeIt->second;
                    friendMap.erase(id);

                    if(friendMap.size()==0){
                        auto pre=std::prev(mergeIt,1);
                        objectMergeProvocation.erase(mergeIt);
                        mergeIt=pre;
                    }
                }
                objectMergeProvocation.erase(id);
            }
        }

        deleteLaterObjects.clear();
    }
}

void RectFloatTracker::handleOneToNObjects(){
    vector<trackingObjectFeature> &fv=featureVectorList.back();

    //1. the blob is away from other blobs
    //2. split for many times when moving!
    for(auto it=mOneToN.begin();it!=mOneToN.end();it++){
        int trackId=it->first;
        const std::set<int> &nblobs=(it->second);
        //1. the distance is big enough!
        set<int> splitToNew;
        for(auto ia=nblobs.begin();ia!=nblobs.end();ia++){
            track_t minRectGap=std::numeric_limits<track_t>::max();
            for(auto ib=nblobs.begin();ib!=nblobs.end();ib++){
                if(*ia==*ib){
                    continue;
                }

                Rect_t &ra=fv[*ia].rect;
                Rect_t &rb=fv[*ib].rect;
                minRectGap=std::min(minRectGap,getRectGap(ra,rb));
            }
            if(minRectGap>MinSplitGap){
                //new object
                tracks.push_back(std::make_unique<singleObjectTracker>(fv[*ia], dt, Accel_noise_mag, NextTrackID++));

                splitToNew.insert(*ia);
            }
        }
        if(nblobs.size()-splitToNew.size()==0){
            deleteLaterObjects.insert(tracks[trackId]->track_id);
            break;
        }
        assert(nblobs.size()-splitToNew.size()>1);

        //mOneToN is set, we cannot modify std::set's element directly.
        //so we create newNblobs, erase and then insert!
        std::set<int> newNblobs;
        if(splitToNew.empty()){
            newNblobs=nblobs;
        }
        else{
            for(auto ia=nblobs.begin();ia!=nblobs.end();ia++){
                if(splitToNew.find(*ia)==splitToNew.end()){
                    newNblobs.insert(*ia);
                }
            }
            ///FIXME how to modify set!
//            it->second=newNblobs;
        }
//        for(auto ia=splitToNew.begin();ia!=splitToNew.end();ia++){
//            auto ib=nblobs.find(*ia);
//            assert(ib!=nblobs.end());
//            nblobs.erase(ib);
//        }


        //2. the patience is out for provocation
        int objectId=tracks[trackId]->track_id;
        auto osp=objectSplitProvocation.find(objectId);
        if(osp==objectSplitProvocation.end()){
            objectSplitProvocation[objectId]=std::make_pair(1,Point_t(0,0));
        }
        else{
            vector<Point_t> &trace=tracks[trackId]->trace;
            assert(trace.size()>1);
            track_t r[4];
            cv::Mat unSplitLIF;
            for(auto ia=newNblobs.begin();ia!=newNblobs.end();ia++){
                Rect_t &rect=fv[*ia].rect;
                if(ia==newNblobs.begin()){
                    r[0]=rect.tl().x;
                    r[1]=rect.tl().y;
                    r[2]=rect.br().x;
                    r[3]=rect.br().y;
                }
                else{
                    r[0]=std::min(r[0],rect.tl().x);
                    r[1]=std::min(r[1],rect.tl().y);
                    r[2]=std::max(r[2],rect.br().x);
                    r[3]=std::max(r[3],rect.br().y);
                }

                if(ia==newNblobs.begin()){
                    unSplitLIF=fv[*ia].LIFMat;
                }
                else{
                    vconcat(unSplitLIF,fv[*ia].LIFMat,unSplitLIF);
                }
            }
            Rect_t unSplitRect(r[0],r[1],r[2]-r[0],r[3]-r[1]);
            Point_t unSplitCenter(r[0]+r[2],r[1]+r[3]);
            Point_t displacement=trace.back()-0.5f*unSplitCenter;
            Point_t history_displacement=objectSplitProvocation[trackId].second;
            track_t provocation=objectSplitProvocation[trackId].first;
            Point_t new_displacement=displacement+history_displacement;
            if(norm(new_displacement)>norm(history_displacement)+5){
                provocation+=1.0f;
            }
            else if(cv::norm(new_displacement)>cv::norm(history_displacement)){
                provocation+=0.1f;
            }
            else{
                provocation=0;
            }

            if(provocation>ObjectSplitPatience){
                qDebug()<<"out of patience";
                deleteLaterObjects.insert(objectId);
                for(auto ia=newNblobs.begin();ia!=newNblobs.end();ia++){
                    tracks.push_back(std::make_unique<singleObjectTracker>(fv[*ia], dt, Accel_noise_mag, NextTrackID++));
                }
            }
            else{
                objectSplitProvocation[trackId]=std::make_pair(provocation,new_displacement);
                trackingObjectFeature *of=(tracks[trackId]->feature);
                of->LIFMat=unSplitLIF;
                of->pos=unSplitCenter;
                of->rect=unSplitRect;
                of->size=unSplitRect.area();

                tracks[trackId]->Update(*of, true, max_trace_length);
            }
        }
    }
}

track_t RectFloatTracker::getRectGap(Rect_t ra,Rect_t rb){
    track_t width=(ra.width+rb.width)*0.5f;
    track_t height=(ra.height+rb.height)*0.5f;
    Point_t p=(ra.tl()+ra.br()-rb.tl()-rb.br())*0.5f;
    if(p.x>width&&p.y>height){
        p.x-=width;
        p.y-=height;
        track_t dist=cv::norm(p);
        return dist;
    }
    else if(p.x>width){
        track_t dx=p.x-width;
        track_t dist=dx*cv::norm(p)/p.x;
        return dist;
    }
    else if(p.y>height){
        track_t dy=p.y-height;
        track_t dist=dy*cv::norm(p)/p.y;
        return dist;
    }
    else{
        return 0.0f;
    }
}

void RectFloatTracker::handleNToOneObjects(){
    vector<trackingObjectFeature> &fv=featureVectorList.back();

    std::map<int,std::set<int>> mergeLater;
    for(auto git=mNToOne.begin();git!=mNToOne.end();git++){
        //the key of map is const, cannot modify directly
        //the track id set.
        const std::set<int> &ids=(git->first);
        int fvIdx=(git->second);
        for(auto ia=ids.begin();ia!=ids.end();ia++){
            for(auto ib=std::next(ia,1);ib!=ids.end();ib++){
                vector<Point_t> &traceA=tracks[*ia]->trace;
                vector<Point_t> &traceB=tracks[*ib]->trace;

                bool flag=isMergedTrace(traceA,traceB);
                if(flag){
                    int objectIdA=tracks[*ia]->track_id;
                    int objectIdB=tracks[*ib]->track_id;
                    int smallId=std::min(objectIdA,objectIdB);
                    int bigId=std::max(objectIdA,objectIdB);
                    auto mit=objectMergeProvocation.find(smallId);
                    if(mit==objectMergeProvocation.end()){
                        std::map<int,int> mergeMap;
                        mergeMap[bigId]=1;
                        objectMergeProvocation[smallId]=mergeMap;
                    }
                    else{
                        std::map<int,int> &mergeMap=objectMergeProvocation[smallId];

                        auto msetIt=mergeMap.find(bigId);
                        if(msetIt==mergeMap.end()){
                            mergeMap[bigId]=1;
                        }
                        else{
                            int mergeTimes=msetIt->second+1;
                            ///FIXME how to modify read-only object!!!
                            /// change set<pair<>> to map!
                            msetIt->second=mergeTimes;
                            if(mergeTimes>objectMergePatience){
                                qDebug()<<"merge them later";
                                auto mergeLaterId=mergeLater.find(fvIdx);
                                if(mergeLaterId!=mergeLater.end()){
                                    mergeLaterId->second.insert(objectIdA);
                                    mergeLaterId->second.insert(objectIdB);
                                }
                                else{
                                    std::set<int> mergeLaterSet;
                                    mergeLaterSet.insert(objectIdA);
                                    mergeLaterSet.insert(objectIdB);
                                    mergeLater[fvIdx]=mergeLaterSet;
                                }
                            }
                        }
                    }
                }
                else{
                    //TODO do nothing or remove objectMergePovocation
                }
            }
        }
    }

    for(auto git=mergeLater.begin();git!=mergeLater.end();git++){
        int fvIdx=git->first;
        std::set<int> &mergeSet=git->second;

        tracks.push_back(std::make_unique<singleObjectTracker>(fv[fvIdx], dt, Accel_noise_mag, NextTrackID++));

        for(auto it=mergeSet.begin();it!=mergeSet.end();it++){
            int objectId=*it;
            deleteLaterObjects.insert(objectId);
        }
    }
}

bool RectFloatTracker::isMergedTrace(vector<Point_t> &traceA,vector<Point_t> &traceB){
    int currentDist;
    for(auto ia=traceA.end(),ib=traceB.end();ia!=traceA.begin()&&ib!=traceB.end();ia--,ib--){
        auto ita=std::prev(ia,1);
        auto itb=std::prev(ib,1);
        if(ia==traceA.end()){
            currentDist=cv::norm(*ita-*itb);
        }
        else{
            track_t d=cv::norm(*ita-*itb);
            if(abs(d-currentDist)>MaxDistForMergingTrace){
                return false;
            }
        }
    }

    return true;
}

void RectFloatTracker::handleNewObjects(){
    vector<trackingObjectFeature> &fv=featureVectorList.back();
    for(auto newit=mNewBlobs.begin();newit!=mNewBlobs.end();newit++){
        tracks.push_back(std::make_unique<singleObjectTracker>(fv[*newit], dt, Accel_noise_mag, NextTrackID++));
    }
}

void RectFloatTracker::handleMissedObjects(){
    for(auto missit=mUnmatchObjects.begin();missit!=mUnmatchObjects.end();missit++){
        tracks[*missit]->skipped_frames +=1;
        if(tracks[*missit]->skipped_frames>maximum_allowed_skipped_frames){
            deleteLaterObjects.insert(*missit);
        }
    }
}

void RectFloatTracker::handleOneToOneObjects(){
    vector<trackingObjectFeature> &fv=featureVectorList.back();
    for(auto it=mOneToOne.begin();it!=mOneToOne.end();it++){
        int trackIdx=it->first;
        int fvIdx=it->second;
        tracks[trackIdx]->Update(fv[fvIdx], true, max_trace_length);
    }
}

void RectFloatTracker::doAssignment(assignments_t assignment,vector<trackingObjectFeature> &fv,cv::Mat costMat){

    assert(assignment.size()==tracks.size());
    assert(costMat.empty()||assignment.size()==costMat.rows);
    std::set<int> oldObjectId;
    // -----------------------------------
    // clean assignment from pairs with large distance
    // -----------------------------------
    for (size_t i = 0; i < assignment.size(); i++)
    {
        if (assignment[i] == -1)
        {
            // If track have no assigned detect, then increment skipped frames counter.
            if(!costMat.empty()){
                for(int col=0;col<costMat.cols;col++){
                    if(costMat.at<uchar>(i,col)>=3){
                        //the object is under other object, so we update ourself!
                        //BUG, the assignment is -1 for only one object!
                        //                        assignment[i]=-1;
                        //                        tracks[i]->skipped_frames = 0;
                        //                        trackingObjectFeature of;
                        //                        tracks[i]->predict(of);
                        //                        tracks[i]->Update(of, true, max_trace_length);
                        //                        //let long merge disppear
                        ////                        tracks[i]->skipped_frames=0;
                        //                        oldObjectId.insert(col);

                        assignment[i]=col;
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

track_t RectFloatTracker::calcPathWeight(std::shared_ptr<trackingObjectFeature> of1, std::shared_ptr<trackingObjectFeature> of2){

    if(of1->LIFMat.empty()){
        qDebug()<<"empty LIFMat in of1";
        return 0.0f;
    }
    if(of2->LIFMat.empty()){
        qDebug()<<"empty LIFMat in of2";
        return 0.0f;
    }
    ObjectLocalFeatureMatch matcher;
    vector<DMatch> good_matchers;
    matcher.getGoodMatches(of1->LIFMat,of2->LIFMat,good_matchers);

    return good_matchers.size();
}

track_t RectFloatTracker::calcCost(std::shared_ptr<trackingObjectFeature> of1, std::shared_ptr<trackingObjectFeature> of2,int costType){
    if(costType==LIFDist){
        track_t maxMatchedLIF=10.0f;
        track_t m=calcPathWeight(of1,of2);
        if(m>=maxMatchedLIF){
            return 0.0f;
        }
        else{
            return maxMatchedLIF-m;
        }
    }
    else if(costType==RectDist){
        Rect_t &r1=of1->rect;
        Rect_t &r2=of2->rect;
        Point_t p1=0.5f*(r1.tl()+r1.br());
        Point_t p2=0.5f*(r2.tl()+r2.br());
        track_t dist=fabs(r1.width-r2.width)+fabs(r1.height-r2.height)+cv::norm(p1-p2);

        return dist;
    }
    else{
        assert(false);
        return -1.0f;
    }
}

void RectFloatTracker::getLocalFeatureAssignment(cv::Mat &matchMat){
    assert(featureVectorList.size()==imageList.size());
    int m=tracks.size();
    int n=featureVectorList.back().size();
    if(m==0||n==0){
        return;
    }

    //    std::vector<trackingObjectFeature> &fv1=featureVectorList.front();
    std::vector<trackingObjectFeature> &fv2=featureVectorList.back();

    if(!matchMat.empty()) matchMat.release();
    matchMat.create(m,n,CV_8UC1);

    for(int i=0;i<m;i++){
        for(int j=0;j<n;j++){
            matchMat.at<uchar>(i,j)=calcPathWeight(std::make_shared<trackingObjectFeature>(*(tracks[i]->feature)),\
                                                   std::make_shared<trackingObjectFeature>(fv2[j]));
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

void RectFloatTracker::showAssignment(assignments_t &assignments,std::vector<trackingObjectFeature> &fv){
    cv::Mat img_objects=img_input.clone();
    cv::Mat img_blobs=img_input.clone();
    assert(assignments.size()==tracks.size());

    for(int i=0;i<tracks.size();i++){
        Point_t p1=tracks[i]->prediction;
        Point_t p2=tracks[i]->feature->pos;
        cv::circle(img_blobs,p1,5,Scalar(0,0,255),3);
        cv::circle(img_blobs,p2,5,Scalar(0,0,255),CV_FILLED);
        cv::line(img_objects,p2,p1,Scalar(255,0,0),3);
    }

    for(int i=0;i<fv.size();i++){
        trackingObjectFeature &of=fv[i];
        Point_t p=of.pos;
        cv::circle(img_blobs,p,5,Scalar(0,0,255),CV_FILLED);
    }

    Mat img_assign;
    cv::hconcat(img_objects,img_blobs,img_assign);
    for(int i=0;i<assignments.size();i++){
        if(assignments[i]!=-1){
            assert(assignments[i]<fv.size());
            trackingObjectFeature &of=fv[assignments[i]];

            Point_t p1=tracks[i]->prediction;
            Point_t p2=of.pos;
            p2.x+=img_input.cols;

            cv::line(img_assign,p1,p2,Scalar(255,0,0),3);
        }
    }

    imshow("img_assign",img_assign);
}
