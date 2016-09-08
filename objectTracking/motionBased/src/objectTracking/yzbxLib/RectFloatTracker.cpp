#include "RectFloatTracker.h"

RectFloatTracker::RectFloatTracker()
{
    maxListLength=2;
    param.NextTrackID=0;
    param.dt=0.2f;
    param.Accel_noise_mag=0.1f;

    //if (Cost[i + assignment[i] * N] > dist_thres)
    param.dist_thres = 100;

    param.maximum_allowed_skipped_frames = 100;
    param.max_trace_length=100;
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

    ///LIF->Hungarian, with split/merge support
    /// LIF get stable tracking for 1-N,N-1,1-1,0-1,1-0
    /// Hungarian only convert 0-1 and 1-0 to 1-1

    cv::Mat matchMat;
    ///matchMat may be empty for empty tracks or empty fv
    getLocalFeatureAssignment(matchMat);


    ///the feature have keypoints+descriptor
    /// LIF get stable tracking when matched feture > StableFeatureNumber(default=3)
    /// use hungarian to tracking featureless objects or small objects
    /// update mObjectToBlobMap, mBlobToObjectMap, mMatchedBlobSet, mMatchedObjectSet
    getUnmatchedHungarainAssignment(matchMat);
    //show the matched feature
    showMatchedFeature();

    ///handle merge(N-1),split(1-N),normal(1-1),missing(1-0),new(0-1)
    /// update mNToOneMap, mOneToNMap, mOneToOneMap, mOneToZeroSet, mZeroToOneSet
    doAssignment();
    showing(img_input,img_fg,fv);
}

void RectFloatTracker::getUnmatchedHungarainAssignment(cv::Mat matchMat){

    assert(!featureVectorList.empty());
    vector<trackingObjectFeature> &fv=featureVectorList.back();

    bool emptyBlobs=fv.empty();
    bool emptyObjects=tracks.empty();

    if(!mObjectToBlobMap.empty())mObjectToBlobMap.clear();
    if(!mBlobToObjectMap.empty())mBlobToObjectMap.clear();
    if(!mMatchedBlobSet.empty())mMatchedBlobSet.clear();
    if(!mMatchedObjectSet.empty())mMatchedObjectSet.clear();

    if(emptyBlobs||emptyObjects){
        //        qDebug()<<"empyt blobs or objects";
    }
    else{
        int m=matchMat.rows;
        int n=matchMat.cols;
        assert(tracks.size()==m);
        assert(fv.size()==n);

        for(int i=0;i<m;i++){
            for(int j=0;j<n;j++){
                uchar featureNum=matchMat.at<uchar>(i,j);
                //local feature point for stable match!
                if(featureNum>StableFeatureNumber){
                    mObjectToBlobMap[i].insert(j);
                    mBlobToObjectMap[j].insert(i);
                    mMatchedBlobSet.insert(j);
                    mMatchedObjectSet.insert(i);
                }
            }
        }

        //hungarian for unstable match
        int unMatchedObjectNum=m-mMatchedObjectSet.size();
        int unMatchedBlobNum=n-mMatchedBlobSet.size();
        if(unMatchedBlobNum==0||unMatchedObjectNum==0){
            //            qDebug()<<"nothing for unmatched hungarain";
        }
        else{
            //vector<trackIdx>
            vector<Index_t> unMatchedObjects;
            //vector<featureVectorIdx>
            vector<Index_t> unMatchedBlobs;
            for(int i=0;i<m;i++){
                if(mMatchedObjectSet.find(i)==mMatchedObjectSet.end()){
                    unMatchedObjects.push_back(i);
                }
            }
            for(int j=0;j<n;j++){
                if(mMatchedBlobSet.find(j)==mMatchedBlobSet.end()){
                    unMatchedBlobs.push_back(j);
                }
            }

            size_t N = unMatchedObjects.size();
            size_t M = unMatchedBlobs.size();
            assert(N>0&&M>0);

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
                    if (Cost[i + unMatchedAssignment[i] * N] > param.dist_thres)
                    {
                        unMatchedAssignment[i] = -1;
                    }
                    else{
                        Index_t trackIdx=unMatchedObjects[i];
                        Index_t featureVectorIdx=unMatchedBlobs[unMatchedAssignment[i]];
                        mObjectToBlobMap[trackIdx].insert(featureVectorIdx);
                        mBlobToObjectMap[featureVectorIdx].insert(trackIdx);
                        mMatchedBlobSet.insert(featureVectorIdx);
                        mMatchedObjectSet.insert(trackIdx);
                    }
                }
            }

        }
    }

}

void RectFloatTracker::runInSingleThread()
{
    qWarning()<<"wait to implement!";
}

void RectFloatTracker::run()
{
    runInSingleThread();
}

void RectFloatTracker::doAssignment(){
    assert(!featureVectorList.empty());
    vector<trackingObjectFeature> &fv=featureVectorList.back();

    bool emptyBlobs=fv.empty();
    bool emptyObjects=tracks.empty();

    if(!mOneToZeroSet.empty())mOneToZeroSet.clear();
    if(!mZeroToOneSet.empty())mZeroToOneSet.clear();
    if(!mOneToNMap.empty())mOneToNMap.clear();
    if(!mOneToOneMap.empty())mOneToOneMap.clear();
    if(!mNToOneMap.empty())mNToOneMap.clear();

    if(emptyBlobs&&emptyObjects){
        //        qDebug()<<"empty Blobs and empty objects";
    }
    else if(emptyBlobs){
        //handle missed object
        for(int i=0;i<tracks.size();i++){
            mOneToZeroSet.insert(i);
        }
        handleMissedObjects();
    }
    else if(emptyObjects){
        //handle new object
        for(int i=0;i<fv.size();i++){
            mZeroToOneSet.insert(i);
        }
        handleNewObjects();
    }
    else{
        ///update mOneToOneMap mNToOneMap mOneToNMap
        for(auto ia=mObjectToBlobMap.begin();ia!=mObjectToBlobMap.end();ia++){
            //tracksIdx
            Index_t trackIdx=ia->first;
            if(ia->second.size()==1){
                Index_t fvIdx=*(ia->second.begin());
                if(mBlobToObjectMap[fvIdx].size()==1){
                    mOneToOneMap[trackIdx]=fvIdx;
                }
                else{
                    mNToOneMap[fvIdx]=mBlobToObjectMap[fvIdx];
                }
            }
            else{
                mOneToNMap[trackIdx]=ia->second;
            }
        }

        ///handle missed object, not change tracks.size()
        for(int i=0;i<tracks.size();i++){
            if(mMatchedObjectSet.find(i)==mMatchedObjectSet.end()){
                mOneToZeroSet.insert(i);
            }
        }
        handleMissedObjects();

        //handle new object, will change tracks.size()
        for(int i=0;i<fv.size();i++){
            if(mMatchedBlobSet.find(i)==mMatchedBlobSet.end()){
                mZeroToOneSet.insert(i);
            }
        }
        handleNewObjects();


        //handle one to one object
        handleOneToOneObjects();
        //handle split object
        handleOneToNObjects();
        //handle merged object
        handleNToOneObjects();

        ///remove objects which need delete later!!!
        for(auto it=tracks.begin();it!=tracks.end();it++){
            Id_t id=(*it)->track_id;
            if(deleteLaterObjectSet.find(id)!=deleteLaterObjectSet.end()){
                auto pre=prev(it,1);
                tracks.erase(it);
                it=pre;

                ///splitProvocationMap keep the weight and displacemnt for each id.
                splitProvocationMap.erase(id);
                ///mergeProvocationMap keep the id to be merged with and the weight
                mergeProvocationMap.erase(id);
                for(auto mergeIt=mergeProvocationMap.begin();mergeIt!=mergeProvocationMap.end();mergeIt++){
                    std::map<Id_t,uint> &friendMap=mergeIt->second;
                    friendMap.erase(id);

                    if(friendMap.size()==0){
                        auto pre=std::prev(mergeIt,1);
                        mergeProvocationMap.erase(mergeIt);
                        mergeIt=pre;
                    }
                }
            }
        }

    }
}

void RectFloatTracker::handleOneToNObjects(){
    vector<trackingObjectFeature> &fv=featureVectorList.back();

    ///1. the blob is away from other blobs
    ///2. the blob is go from faraway to nearby, size from small to big!
    ///3. the blob split for many times when moving!

    std::map<Index_t,std::set<Index_t>> splitToNewMap;
    ///1. the blob is away from other blobs, the distance is big enough!
    for(auto it=mOneToNMap.begin();it!=mOneToNMap.end();it++){
        Index_t trackIdx=it->first;
        std::set<Index_t> &splitBlobSet=(it->second);
        /// empty set will be remove, size()==1 is error status!
        assert(splitBlobSet.size()>1);

        for(auto ia=splitBlobSet.begin();ia!=splitBlobSet.end();ia++){
            track_t minRectGap=std::numeric_limits<track_t>::max();
            for(auto ib=splitBlobSet.begin();ib!=splitBlobSet.end();ib++){
                if(*ia==*ib){
                    continue;
                }

                Rect_t &ra=fv[*ia].rect;
                Rect_t &rb=fv[*ib].rect;
                minRectGap=std::min(minRectGap,yzbxlib::getRectGap(ra,rb));
            }
            ///the minimal distance for *ia to other blobs association to object trackIdx.
            if(minRectGap>MinSplitGap){
                //new object
                tracks.push_back(std::make_unique<singleObjectTracker>(fv[*ia], param.dt, param.Accel_noise_mag, param.NextTrackID++));
                splitToNewMap[trackIdx].insert(*ia);
            }
        }
    }
    for(auto it=splitToNewMap.begin();it!=splitToNewMap.end();it++){
        Index_t trackIdx=it->first;
        std::set<Index_t> &newBlobSet=it->second;
        for(auto newIt=newBlobSet.begin();newIt!=newBlobSet.end();newIt++){
            mOneToNMap[trackIdx].erase(*newIt);
        }
        if(mOneToNMap[trackIdx].empty()){
            mOneToNMap.erase(trackIdx);
            deleteLaterObjectSet.insert(tracks[trackIdx]->track_id);
        }
    }
    splitToNewMap.clear();
    ///2. the blob is go from faraway to nearby, size from small to big!
    /// use a small MinSplitGap will work for this one, but not work for false negative in foreground mask.
    //    for(auto it=mOneToNMap.begin();it!=mOneToNMap.end();it++){
    //        Index_t trackIdx=it->first;
    //        std::set<Index_t> &splitBlobSet=(it->second);
    //        /// empty set will be remove, size()==1 is error status!
    //        assert(splitBlobSet.size()>1);
    //    }

    ///3. split for a lot of times when moving; eg: two friends go together.
    for(auto it=mOneToNMap.begin();it!=mOneToNMap.end();it++){
        Index_t trackIdx=it->first;
        std::set<Index_t> &splitBlobSet=(it->second);
        /// empty set will be remove, size()==1 is error status!
        assert(splitBlobSet.size()>1);

        int objectId=tracks[trackIdx]->track_id;
        auto osp=splitProvocationMap.find(objectId);
        ///the first time to split
        if(osp==splitProvocationMap.end()){
            splitProvocationMap[objectId]=std::make_pair(1,Point_t(0,0));
        }
        else{
            ///split while moving!!!
            vector<Point_t> &trace=tracks[trackIdx]->trace;
            assert(trace.size()>1);

            track_t r[4];
            for(auto ia=splitBlobSet.begin();ia!=splitBlobSet.end();ia++){
                Rect_t &rect=fv[*ia].rect;
                if(ia==splitBlobSet.begin()){
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
            }
            Point_t unSplitCenter(r[0]+r[2],r[1]+r[3]);
            Point_t displacement=0.5f*unSplitCenter-trace.back();
            Point_t history_displacement=splitProvocationMap[trackIdx].second;
            track_t provocation=splitProvocationMap[trackIdx].first;
            Point_t new_displacement=displacement+history_displacement;
            //keep moving, like two friends
            if(norm(new_displacement)>norm(history_displacement)+5){
                provocation+=1.0f;
            }//move slow, have not go out of background cluster.
            else if(cv::norm(new_displacement)>cv::norm(history_displacement)){
                provocation+=0.1f;
            }//not move, linger on background cluster.
            else{
                provocation=0;
            }

            if(provocation>MaxProvocationTimesForSplit){
                qDebug()<<"out of patience, split it!!!";
                deleteLaterObjectSet.insert(objectId);
                //split it all!!! for it's not for background cluster.
                for(auto ia=splitBlobSet.begin();ia!=splitBlobSet.end();ia++){
                    splitToNewMap[trackIdx].insert(*ia);
                    tracks.push_back(std::make_unique<singleObjectTracker>(fv[*ia], param.dt, param.Accel_noise_mag, param.NextTrackID++));
                }
            }
            else{
                splitProvocationMap[objectId]=std::make_pair(provocation,new_displacement);
            }
        }
    }

    for(auto it=splitToNewMap.begin();it!=splitToNewMap.end();it++){
        Index_t trackIdx=it->first;
        std::set<Index_t> &newBlobSet=it->second;
        for(auto newIt=newBlobSet.begin();newIt!=newBlobSet.end();newIt++){
            std::set<Index_t> &nset=mOneToNMap[trackIdx];
            auto rmIt=nset.find(*newIt);
            nset.erase(rmIt);
        }
        if(mOneToNMap[trackIdx].empty()){
            mOneToNMap.erase(trackIdx);
            deleteLaterObjectSet.insert(tracks[trackIdx]->track_id);
        }
    }
    splitToNewMap.clear();

    ///4. merge feature for object which not split!
    for(auto it=mOneToNMap.begin();it!=mOneToNMap.end();it++){
        Index_t trackIdx=it->first;
        std::set<Index_t> &splitBlobSet=(it->second);
        /// empty set will be remove, size()==1 is error status!
        assert(splitBlobSet.size()>1);

        ///get merged feature for objectId
        track_t r[4];
        cv::Mat unSplitLIF;
        for(auto ia=splitBlobSet.begin();ia!=splitBlobSet.end();ia++){
            Rect_t &rect=fv[*ia].rect;
            if(ia==splitBlobSet.begin()){
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

            //NOTE need add keypoints here
            if(ia==splitBlobSet.begin()){
                unSplitLIF=fv[*ia].LIFMat;
            }
            else{
                vconcat(unSplitLIF,fv[*ia].LIFMat,unSplitLIF);
            }
        }

        Rect_t unSplitRect(r[0],r[1],r[2]-r[0],r[3]-r[1]);
        Point_t unSplitCenter((r[0]+r[2])*0.5f,(r[1]+r[3])*0.5f);
        trackingObjectFeature *of=(tracks[trackIdx]->feature);
        of->LIFMat=unSplitLIF;
        of->pos=unSplitCenter;
        of->rect=unSplitRect;
        of->size=unSplitRect.area();

        tracks[trackIdx]->Update(*of, true, param.max_trace_length);
    }
}

void RectFloatTracker::handleNToOneObjects(){
    vector<trackingObjectFeature> &fv=featureVectorList.back();
    //std::map<fvIdx,std::set<objectId>>
    std::map<int,std::set<int>> mergeLater;
    for(auto git=mNToOneMap.begin();git!=mNToOneMap.end();git++){
        //the key of map is const, cannot modify directly
        //the track id set.
        std::set<Index_t> &ids=git->second;
        int fvIdx=git->first;
        for(auto ia=ids.begin();ia!=ids.end();ia++){
            for(auto ib=std::next(ia,1);ib!=ids.end();ib++){
                vector<Point_t> &traceA=tracks[*ia]->trace;
                vector<Point_t> &traceB=tracks[*ib]->trace;

                bool flag=isTraceMergable(traceA,traceB);
                if(flag){
                    int objectIdA=tracks[*ia]->track_id;
                    int objectIdB=tracks[*ib]->track_id;
                    int smallId=std::min(objectIdA,objectIdB);
                    int bigId=std::max(objectIdA,objectIdB);
                    auto mit=mergeProvocationMap.find(smallId);
                    if(mit==mergeProvocationMap.end()){
                        //                        std::map<int,int> mergeMap;
                        //                        mergeMap[bigId]=1;
                        //                        mergeProvocationMap[smallId]=mergeMap;
                        mergeProvocationMap[smallId][bigId]=1;
                        //                        mergeChanceMap[smallId][bigId]=InitMergeChance;
                    }
                    else{
                        std::map<Id_t,uint> &mergeMap=mergeProvocationMap[smallId];

                        auto msetIt=mergeMap.find(bigId);
                        if(msetIt==mergeMap.end()){
                            mergeMap[bigId]=1;
                        }
                        else{
                            int mergeTimes=msetIt->second+1;
                            ///FIXME how to modify read-only object!!!
                            /// change set<pair<>> to map!
                            msetIt->second=mergeTimes;
                            if(mergeTimes>MaxProvocationTimesForMerge){
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

        tracks.push_back(std::make_unique<singleObjectTracker>(fv[fvIdx], param.dt, param.Accel_noise_mag, param.NextTrackID++));

        for(auto it=mergeSet.begin();it!=mergeSet.end();it++){
            int objectId=*it;
            deleteLaterObjectSet.insert(objectId);
        }
    }

    //TODO update for other objects and pos.
}

bool RectFloatTracker::isTraceMergable(vector<Point_t> &traceA,vector<Point_t> &traceB){
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
    for(auto newit=mZeroToOneSet.begin();newit!=mZeroToOneSet.end();newit++){
        tracks.push_back(std::make_unique<singleObjectTracker>(fv[*newit], param.dt, param.Accel_noise_mag, param.NextTrackID++));
    }
}

void RectFloatTracker::handleMissedObjects(){
    for(auto missit=mOneToZeroSet.begin();missit!=mOneToZeroSet.end();missit++){
        assert(*missit<tracks.size());
        tracks[*missit]->skipped_frames +=1;
        if(tracks[*missit]->skipped_frames>param.maximum_allowed_skipped_frames){
            deleteLaterObjectSet.insert(*missit);
        }
    }
}

void RectFloatTracker::handleOneToOneObjects(){
    vector<trackingObjectFeature> &fv=featureVectorList.back();
    for(auto it=mOneToOneMap.begin();it!=mOneToOneMap.end();it++){
        int trackIdx=it->first;
        int fvIdx=it->second;
        tracks[trackIdx]->Update(fv[fvIdx], true, param.max_trace_length);
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

        if(r.tl().y>20){
            cv::putText(img_tracking, text, r.tl(), FONT_HERSHEY_COMPLEX, 0.5,
                        cv::Scalar(0,0,255), 2, 8);
        }
        else{
            Point_t p=r.tl();
            p.y=r.br().y+10;
            cv::putText(img_tracking, text, p, FONT_HERSHEY_COMPLEX, 0.5,
                        cv::Scalar(0,0,255), 2, 8);
        }
    }

    cv::imshow("img_tracking",img_tracking);
}

track_t RectFloatTracker::calcMatchedFeatureNum(std::shared_ptr<trackingObjectFeature> of1, std::shared_ptr<trackingObjectFeature> of2){

    if(of1->LIFMat.empty()){
        //        qDebug()<<"empty LIFMat in of1";
        return 0.0f;
    }
    if(of2->LIFMat.empty()){
        //        qDebug()<<"empty LIFMat in of2";
        return 0.0f;
    }
    ObjectLocalFeatureMatch matcher;
    vector<DMatch> good_matchers;
    matcher.getGoodMatches(of1->LIFMat,of2->LIFMat,good_matchers);

    return good_matchers.size();
}

void RectFloatTracker::showMatchedFeature(){
    assert(!featureVectorList.empty());
    vector<trackingObjectFeature> &fv=featureVectorList.back();

    assert(!imageList.empty());
    if(imageList.size()<2) return;

    auto imgIt=imageList.end();
    imgIt=std::prev(imgIt,2);

    cv::Mat img=imgIt->first;
    cv::Mat input;
    if(img.channels()==3){
        input=img.clone();
    }
    else{
        cvtColor(img,input,CV_GRAY2BGR);
    }
    cv::Mat fg0=imageList.back().second;
    cv::Mat fg;
    if(fg0.channels()==3){
        fg=fg0.clone();
    }
    else{
        cvtColor(fg0,fg,CV_GRAY2BGR);
    }

    cv::Mat showImg;
    hconcat(input,fg,showImg);
    for(int i=0;i<tracks.size();i++){
        std::shared_ptr<trackingObjectFeature> of1=std::make_shared<trackingObjectFeature>(*(tracks[i]->feature));
        for(int j=0;j<fv.size();j++){
            std::shared_ptr<trackingObjectFeature> of2=std::make_shared<trackingObjectFeature>(fv[j]);

            if(of1->LIFMat.empty()){
                continue;
            }
            if(of2->LIFMat.empty()){
                continue;
            }
            vector<DMatch> good_matches;

            vector< vector<DMatch> > matches;
            Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce-Hamming");
            //knnMatch(query,train,...)
            matcher->knnMatch( of1->LIFMat, of2->LIFMat, matches, 2 );

            for(size_t i = 0; i < matches.size(); i++) {
                DMatch first = matches[i][0];
                if(matches[i].size()>1){
                    float dist1 = matches[i][0].distance;
                    float dist2 = matches[i][1].distance;
                    if(dist1 < global_match_ratio * dist2) {
                        good_matches.push_back(first);
                    }
                }
                else{
                    good_matches.push_back(first);
                }
            }

            for(int m=0;m<good_matches.size();m++){
                int queryIdx=good_matches[m].queryIdx;
                int trainIdx=good_matches[m].trainIdx;
                Point_t pos1=of1->LIFPos[queryIdx];
                Point_t pos2=of2->LIFPos[trainIdx];
                yzbxlib::drawMatch(showImg,pos1,pos2);
            }
        }
    }

    imshow("matched feature",showImg);
}

track_t RectFloatTracker::calcCost(std::shared_ptr<trackingObjectFeature> of1, std::shared_ptr<trackingObjectFeature> of2,int costType){
    if(costType==LIFDist){
        track_t maxMatchedLIF=10.0f;
        track_t m=calcMatchedFeatureNum(of1,of2);
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
    assert(!featureVectorList.empty());
    std::vector<trackingObjectFeature> &fv=featureVectorList.back();
    if(!matchMat.empty()) matchMat.release();
    int m=tracks.size();
    int n=fv.size();
    if(m==0||n==0){
        return;
    }

    matchMat.create(m,n,CV_8UC1);

    for(int i=0;i<m;i++){
        for(int j=0;j<n;j++){
            matchMat.at<uchar>(i,j)=calcMatchedFeatureNum(std::make_shared<trackingObjectFeature>(*(tracks[i]->feature)),\
                                                          std::make_shared<trackingObjectFeature>(fv[j]));
        }
    }
}


//void RectFloatTracker::showAssignment(assignments_t &assignments,std::vector<trackingObjectFeature> &fv){
//    cv::Mat img_objects=img_input.clone();
//    cv::Mat img_blobs=img_input.clone();
//    assert(assignments.size()==tracks.size());

//    for(int i=0;i<tracks.size();i++){
//        Point_t p1=tracks[i]->prediction;
//        Point_t p2=tracks[i]->feature->pos;
//        cv::circle(img_blobs,p1,5,Scalar(0,0,255),3);
//        cv::circle(img_blobs,p2,5,Scalar(0,0,255),CV_FILLED);
//        cv::line(img_objects,p2,p1,Scalar(255,0,0),3);
//    }

//    for(int i=0;i<fv.size();i++){
//        trackingObjectFeature &of=fv[i];
//        Point_t p=of.pos;
//        cv::circle(img_blobs,p,5,Scalar(0,0,255),CV_FILLED);
//    }

//    Mat img_assign;
//    cv::hconcat(img_objects,img_blobs,img_assign);
//    for(int i=0;i<assignments.size();i++){
//        if(assignments[i]!=-1){
//            assert(assignments[i]<fv.size());
//            trackingObjectFeature &of=fv[assignments[i]];

//            Point_t p1=tracks[i]->prediction;
//            Point_t p2=of.pos;
//            p2.x+=img_input.cols;

//            cv::line(img_assign,p1,p2,Scalar(255,0,0),3);
//        }
//    }

//    imshow("img_assign",img_assign);
//}
