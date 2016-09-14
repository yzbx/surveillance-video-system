#include "RectFloatTracker.h"

RectFloatTracker::RectFloatTracker()
{
    maxListLength=2;
    param.NextTrackID=0;
    param.dt=0.2f;
    param.Accel_noise_mag=0.1f;

    //if (Cost[i + assignment[i] * N] > dist_thres)
    param.dist_thres = 100;

    param.maximum_allowed_skipped_frames = 5;
    param.max_trace_length=100;
}

void RectFloatTracker::process(const Mat &img_input, const Mat &img_fg,vector<trackingObjectFeature> &fv)
{
    ///PipeLine must ensure this!!!
    assert(img_input.channels()==3);
    assert(img_fg.channels()==1);

    imageList.push_back(std::make_pair(img_input,img_fg));
    if(imageList.size()>maxListLength){
        imageList.pop_front();
    }
    featureVectorList.push_back(fv);
    if(featureVectorList.size()>maxListLength){
        featureVectorList.pop_front();
    }
    showKalmanFeature();
    showBlobFeature();
    ///LIF->Hungarian, with split/merge support
    /// LIF get stable tracking for 1-N,N-1,1-1,0-1,1-0
    /// Hungarian only convert 0-1 and 1-0 to 1-1
    getRectOverlapAssignment_step1();
    showNewBlobFeature();

    cv::Mat matchMat;
    ///matchMat may be empty for empty tracks or empty fv
    getLocalFeatureAssignment_step2(matchMat);


    ///the feature have keypoints+descriptor
    /// LIF get stable tracking when matched feture > StableFeatureNumber(default=3)
    /// use hungarian to tracking featureless objects or small objects
    /// update mObjectToBlobMap, mBlobToObjectMap, mMatchedBlobSet, mMatchedObjectSet
    getUnmatchedHungarainAssignment_step3(matchMat);
    //show the matched feature through p2p match.
    //    showMatchedFeature(matchMat);

    //show the assignment
    showAssignment();

    ///handle merge(N-1),split(1-N),normal(1-1),missing(1-0),new(0-1)
    /// update mNToOneMap, mOneToNMap, mOneToOneMap, mOneToZeroSet, mZeroToOneSet
    doAssignment_step4();
    showing(img_input,img_fg,fv);
    waitKey(0);
}

void RectFloatTracker::showAssignment(){
    assert(!featureVectorList.empty());
    vector<trackingObjectFeature> &fv=featureVectorList.back();
    assert(!imageList.empty());
    Mat input=imageList.back().first.clone();

    for(auto obj=mObjectToBlobMap.begin();obj!=mObjectToBlobMap.end();obj++){
        Index_t objIdx=obj->first;
        Point_t pt1=tracks[objIdx]->feature->pos;
        Rect_t r1=tracks[objIdx]->feature->rect;
        //        cv::rectangle(input,r1,Scalar(255,0,0),3);
        int lifetime=tracks[objIdx]->get_lifetime();
        string text=boost::lexical_cast<string>(lifetime);
        string idstr=boost::lexical_cast<string>(objIdx);
        text=idstr+": "+text;
        yzbxlib::annotation(input,r1,text,Scalar(255,0,0));
        const std::set<Index_t> &blobset=obj->second;
        for(auto blob=blobset.begin();blob!=blobset.end();blob++){
            Index_t blobIdx=*blob;
            Point_t pt2=fv[blobIdx].pos;
            Rect_t r2=fv[blobIdx].rect;
            cv::line(input,pt1,pt2,Scalar(0,0,255),3);
            cv::rectangle(input,r2,Scalar(0,255,255),3);
        }
    }

    cv::namedWindow("assignment objectToBlob",WINDOW_NORMAL);
    cv::imshow("assignment objectToBlob",input);

    input=imageList.back().first.clone();
    for(auto blob=mBlobToObjectMap.begin();blob!=mBlobToObjectMap.end();blob++){
        Index_t blobIdx=blob->first;
        Point_t pt1=fv[blobIdx].pos;
        Rect_t r1=fv[blobIdx].rect;
        cv::rectangle(input,r1,Scalar(0,255,255),3);
        const std::set<Index_t> &objset=blob->second;
        for(auto obj=objset.begin();obj!=objset.end();obj++){
            Index_t objIdx=*obj;
            Point_t pt2=tracks[objIdx]->feature->pos;
            Rect_t r2=tracks[objIdx]->feature->rect;
            cv::line(input,pt2,pt1,Scalar(0,0,255),3);
            //            cv::rectangle(input,r2,Scalar(255,0,0),3);
            int lifetime=tracks[objIdx]->get_lifetime();
            string text=boost::lexical_cast<string>(lifetime);
            string idstr=boost::lexical_cast<string>(objIdx);
            text=idstr+": "+text;
            yzbxlib::annotation(input,r2,text,Scalar(255,0,0));
        }
    }

    cv::namedWindow("assignment blobToObject",WINDOW_NORMAL);
    cv::imshow("assignment blobToObject",input);
}

void RectFloatTracker::showKalmanFeature(){
    assert(!featureVectorList.empty());
    vector<trackingObjectFeature> &fv=featureVectorList.back();
    assert(!imageList.empty());
    Mat input=imageList.back().first.clone();
    Mat fg=imageList.back().second.clone();

    for(int i=0;i<tracks.size();i++){
        Point_t p=tracks[i]->feature->pos;
        Rect_t r=tracks[i]->feature->rect;
        //        Point_t predict_p=tracks[i]->KF.LastResult;
        Point_t predict_p=tracks[i]->KF.GetPrediction();

        cv::line(input,p,predict_p,Scalar(0,0,255),3);
        cv::circle(input,p,5,Scalar(255,0,0),3);
        cv::circle(input,predict_p,5,Scalar(0,255,0),3);
        cv::rectangle(input,r,Scalar(0,0,255),5);

        cv::rectangle(fg,r,Scalar::all(255),5);
    }

    for(int i=0;i<fv.size();i++){
        Point_t p=fv[i].pos;
        Rect_t r=fv[i].rect;

        cv::circle(input,p,5,Scalar(0,255,255),3);
        cv::rectangle(input,r,Scalar(0,255,255),5);

        cv::rectangle(fg,r,Scalar::all(255),5);
    }

    cv::namedWindow("kalman input",WINDOW_NORMAL);
    cv::namedWindow("kalman fg",WINDOW_NORMAL);
    cv::imshow("kalman input",input);
    cv::imshow("kalman fg",fg);
}

void RectFloatTracker::getRectOverlapAssignment_step1(){
    assert(!featureVectorList.empty());
    vector<trackingObjectFeature> &fv=featureVectorList.back();

    bool emptyBlobs=fv.empty();
    bool emptyObjects=tracks.empty();

    //    if(!mObjectToBlobMap.empty())mObjectToBlobMap.clear();
    //    if(!mBlobToObjectMap.empty())mBlobToObjectMap.clear();
    //    if(!mMatchedBlobSet.empty())mMatchedBlobSet.clear();
    //    if(!mMatchedObjectSet.empty())mMatchedObjectSet.clear();

    std::map<Index_t,std::set<Index_t>> trackToBlobMap;
    std::map<Index_t,std::set<Index_t>> blobToTrackMap;

    if(emptyBlobs||emptyObjects){
        //        qDebug()<<"empyt blobs or objects";
    }
    else{
        //use kalman to predict rect
        for(int i=0;i<tracks.size();i++){
            Point_t p=tracks[i]->feature->pos;
            Rect_t &r=tracks[i]->feature->rect;
            //        Point_t predict_p=tracks[i]->KF.LastResult;
            Point_t predict_p=tracks[i]->KF.GetPrediction();

            r.x+=predict_p.x-p.x;
            r.y+=predict_p.y-p.y;
        }

        for(int i=0;i<tracks.size();i++){
            Rect_t ra=tracks[i]->feature->rect;
            int lifetime=tracks[i]->get_lifetime();
            if(lifetime<MaxFreshObjectLifeTime||
                    tracks[i]->status!=NORMAL_STATUS) continue;

            for(int j=0;j<fv.size();j++){
                Rect_t rb=fv[j].rect;

                //TODO add some check

                if(yzbxlib::getOverlapRatio(ra,rb)>0.5){
                    trackToBlobMap[i].insert(j);
                    blobToTrackMap[j].insert(i);
                }
            }
        }

        //trackId, blobId1,blobId2
        std::map<Index_t,std::set<Index_t>> trackToSplitLater;
        for(int i=0;i<trackToBlobMap.size();i++){
            std::set<Index_t> &blobSet=trackToBlobMap[i];
            std::set<Index_t> blobToOneTrackSet;
            for(auto blob=blobSet.begin();blob!=blobSet.end();blob++){
                Index_t blobIdx=*blob;
                if(blobToTrackMap[blobIdx].size()==1){
                    blobToOneTrackSet.insert(blobIdx);
                }
            }
            //do split and only split into two or more rects
            //BUG split as soon as possible
            if(blobToOneTrackSet.size()>=2){
                trackToSplitLater[i]=blobToOneTrackSet;
            }
        }

        std::map<Index_t,std::set<Index_t>> blobToSplitLater;
        for(int i=0;i<blobToTrackMap.size();i++){
            std::set<Index_t> &trackSet=blobToTrackMap[i];
            std::set<Index_t> trackToOneBlobSet;
            for(auto track=trackSet.begin();track!=trackSet.end();track++){
                Index_t trackIdx=*track;
                if(trackToBlobMap[trackIdx].size()==1){
                    trackToOneBlobSet.insert(trackIdx);
                }
            }

            //do split and only split into two or more rects
            //BUG split as soon as possible
            if(trackToOneBlobSet.size()>=2){
                blobToSplitLater[i]=trackToOneBlobSet;
            }
        }

        //split track
        for(int i=0;i<trackToSplitLater.size();i++){
            if(!trackToSplitLater[i].empty()){
                doRectOverlap_splitTrack(i,trackToSplitLater[i]);
            }
        }
        //split blob
        for(int i=0;i<blobToSplitLater.size();i++){
            if(!blobToSplitLater[i].empty()){
                doRectOverlap_splitBlob(i,blobToSplitLater[i]);
            }
        }
        //        //delete track
        //        for(int i=0;i<trackToSplitLater.size();i++){
        //            if(!trackToSplitLater[i].empty()){
        //                doRectOverlap_deleteTrack(i);
        //            }
        //        }
        //        //delete blob
        //        for(int i=0;i<blobToSplitLater.size();i++){
        //            if(!blobToSplitLater[i].empty()){
        //                doRectOverlap_deleteBlob(i);
        //            }
        //        }
    }
}
void RectFloatTracker::doRectOverlap_splitTrack(Index_t trackIdx,std::set<Index_t> &blobset){
    assert(blobset.size()>=2);
    vector<trackingObjectFeature> &fv=featureVectorList.back();

    //get max blob, assign it to origin objectIdx
    float maxArea=0;
    Index_t maxAreaBlobIdx;
    for(auto blob=blobset.begin();blob!=blobset.end();blob++){
        if(blob==blobset.begin()){
            maxArea=fv[*blob].size;
            maxAreaBlobIdx=*blob;
        }
        else{
            if(maxArea<fv[*blob].size){
                maxArea=fv[*blob].size;
                maxAreaBlobIdx=*blob;
            }
        }
    }
    //change blob feature
    tracks[trackIdx]->PreUpdateForBiggestBlob(fv[maxAreaBlobIdx]);

    for(auto blob=blobset.begin();blob!=blobset.end();blob++){
        if(*blob!=maxAreaBlobIdx){
            tracks.push_back(std::make_unique<singleObjectTracker>(fv[*blob],param.dt,
                             param.Accel_noise_mag,param.NextTrackID++,frameNum));
            tracks.back()->AvoidUpdateTwice();
        }
    }
    //    //TODO split the trace
    //    tracks[trackIdx]=std::make_unique<singleObjectTracker>(fv[*it1],param.dt,param.Accel_noise_mag,param.NextTrackID++);
    ////    tracks.push_back(std::make_unique<singleObjectTracker>(fv[*it1],param.dt,param.Accel_noise_mag,param.NextTrackID++));
    //    tracks.push_back(std::make_unique<singleObjectTracker>(fv[*it2],param.dt,param.Accel_noise_mag,param.NextTrackID++));
}
void RectFloatTracker::doRectOverlap_splitBlob(Index_t blobIdx,std::set<Index_t> &trackset){
    assert(trackset.size()>=2);
    vector<trackingObjectFeature> &fv=featureVectorList.back();

    bool flag;
    if(trackset.size()==2){
        Rect_t rb=fv[blobIdx].rect;
        auto it1=trackset.begin();
        auto it2=std::next(it1,1);
        Rect_t rt1=tracks[*it1]->feature->rect;
        Rect_t rt2=tracks[*it2]->feature->rect;
        flag=yzbxlib::splitRect(rb,rt1,rt2);
        static int tag=0;
        //split okay
        if(flag){
            qDebug()<<tag++<<"split blob";
            const cv::Mat LIFMat=fv[blobIdx].LIFMat;
            const std::vector<Point_t> &LIFPos=fv[blobIdx].LIFPos;
            const std::vector<cv::Point3i> LIFColor=fv[blobIdx].LIFColor;
            cv::Mat LIFMat1(0,LIFMat.cols,LIFMat.type());
            cv::Mat LIFMat2(0,LIFMat.cols,LIFMat.type());
            trackingObjectFeature of1,of2;
            of1.copy(fv[blobIdx]);
            of2.copy(fv[blobIdx]);
            of1.LIFMat=LIFMat1;
            of2.LIFMat=LIFMat2;
            of1.LIFPos.clear();
            of2.LIFPos.clear();
            of1.LIFColor.clear();
            of2.LIFColor.clear();

            for(int i=0;i<LIFPos.size();i++){

                bool flag1=yzbxlib::isPointInRect(LIFPos[i],rt1);
                bool flag2=yzbxlib::isPointInRect(LIFPos[i],rt2);
                if(flag1&&!flag2){
                    of1.LIFMat.push_back(LIFMat.row(i));
                    of1.LIFPos.push_back(LIFPos[i]);
                    of1.LIFColor.push_back(LIFColor[i]);
                }
                else if(flag2&&!flag1){
                    of2.LIFMat.push_back(LIFMat.row(i));
                    of2.LIFPos.push_back(LIFPos[i]);
                    of2.LIFColor.push_back(LIFColor[i]);
                }
                else{
                    qDebug()<<"unexpected thing happended !!!";
                }
            }

            of1.rect=rt1;
            of2.rect=rt2;
            of1.pos=tracks[*it1]->KF.GetPrediction();
            of2.pos=tracks[*it2]->KF.GetPrediction();
            fv[blobIdx]=of1;
            fv.push_back(of2);
        }
        else{
            trackingObjectFeature of1,of2;
            of1.copy(fv[blobIdx]);
            of2.copy(fv[blobIdx]);

            Rect_t r1=tracks[*it1]->feature->rect;
            Rect_t r2=tracks[*it2]->feature->rect;
            of1.rect=yzbxlib::getSubRect(rb,r1);
            of2.rect=yzbxlib::getSubRect(rb,r2);

            of1.pos=tracks[*it1]->KF.GetPrediction();
            of2.pos=tracks[*it2]->KF.GetPrediction();
            of1.pos+=(of1.rect.tl()-r1.tl());
            of2.pos+=(of2.rect.tl()-r2.tl());

            fv[blobIdx]=of1;
            fv.push_back(of2);
            assert(tracks[*it1]->feature->LIFMat.empty());
            assert(tracks[*it2]->feature->LIFMat.empty());
        }
    }
    else{

        Rect_t rb=fv[blobIdx].rect;
        for(auto track=trackset.begin();track!=trackset.end();track++){
            assert(tracks[*track]->status==PREUPDATE_STATUS||
                    tracks[*track]->get_lifetime()>=MaxFreshObjectLifeTime);

            trackingObjectFeature of;
            of.copy(fv[blobIdx]);
            of.pos=tracks[*track]->KF.GetPrediction();
            Rect_t predict_rect=tracks[*track]->feature->rect;
            Rect_t subRect=yzbxlib::getSubRect(rb,predict_rect);
            of.rect=subRect;
            of.pos+=(of.rect.tl()-predict_rect.tl());

            if(track==trackset.begin()){
                fv[blobIdx]=of;
            }
            else{
                fv.push_back(of);
            }
            assert(tracks[*track]->feature->LIFMat.empty());
        }
    }

}

void RectFloatTracker::showBlobFeature(){
    assert(!featureVectorList.empty());
    vector<trackingObjectFeature> &fv=featureVectorList.back();
    assert(!imageList.empty());
    Mat input=imageList.back().first.clone();
    Mat fg=imageList.back().second.clone();

    for(int i=0;i<fv.size();i++){
        trackingObjectFeature &of=fv[i];
        rectangle(input,of.rect,Scalar(0,0,255),2);
        rectangle(fg,of.rect,Scalar::all(255),2);
        for(int j=0;j<of.LIFPos.size();j++){
            circle(input,of.LIFPos[j],3,Scalar(255,0,0),2);
        }
    }

    namedWindow("blob feature input",WINDOW_NORMAL);
    namedWindow("blob feature fg",WINDOW_NORMAL);
    imshow("blob feature input",input);
    imshow("blob feature fg",fg);
}

void RectFloatTracker::showNewBlobFeature(){
    assert(!featureVectorList.empty());
    vector<trackingObjectFeature> &fv=featureVectorList.back();
    assert(!imageList.empty());
    Mat input=imageList.back().first.clone();
    Mat fg=imageList.back().second.clone();

    for(int i=0;i<fv.size();i++){
        trackingObjectFeature &of=fv[i];
        rectangle(input,of.rect,Scalar(0,0,255),2);
        rectangle(fg,of.rect,Scalar::all(255),2);
        for(int j=0;j<of.LIFPos.size();j++){
            circle(input,of.LIFPos[j],3,Scalar(255,0,0),2);
        }
    }

    namedWindow("new blob feature input",WINDOW_NORMAL);
    namedWindow("new blob feature fg",WINDOW_NORMAL);
    imshow("new blob feature input",input);
    imshow("new blob feature fg",fg);
}


void RectFloatTracker::getUnmatchedHungarainAssignment_step3(cv::Mat matchMat){

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

void RectFloatTracker::doAssignment_step4(){
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
                dumpDeleteObject(it-tracks.begin());
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

void RectFloatTracker::dumpDeleteObject(Index_t trackIdx){
    if(tracks[trackIdx]->get_lifetime()>MinDumpLifeTime){
        vector<Rect_t> &rects=tracks[trackIdx]->rects;
        vector<Point_t> &trace=tracks[trackIdx]->trace;
        vector<int> &frames=tracks[trackIdx]->frames;
        vector<STATUS> &vec_status=tracks[trackIdx]->vec_status;

        int n=rects.size();
        assert(n==trace.size());
        assert(n==frames.size());
        assert(n==vec_status.size());

        QString idstr=QString::number(tracks[trackIdx]->track_id);

        QFile file(globalRecordFile);
        if(!file.open(QIODevice::WriteOnly|QIODevice::Append)){
            assert(false);
        }
        QTextStream out(&file);
        for(int i=0;i<n;i++){
            if(vec_status[i]!=MISSING_STATUS){
                QStringList line;
                int x=(int)rects[i].x;
                int y=(int)rects[i].y;
                int width=(int)rects[i].width;
                int height=(int)rects[i].height;
                line<<QString::number(frames[i])<<idstr<<QString::number(x)<<QString::number(y)
                   <<QString::number(width)<<QString::number(height);

                out<<line.join(",")<<"\n";
            }
        }

        file.close();
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
                //new object, set splitMergeStatus
                tracks.push_back(std::make_unique<singleObjectTracker>(fv[*ia], param.dt, param.Accel_noise_mag, param.NextTrackID++,frameNum));
                tracks.back()->trace=tracks[trackIdx]->trace;
                tracks.back()->trace.push_back(fv[*ia].pos);
                tracks.back()->rects=tracks[trackIdx]->rects;
                tracks.back()->rects.push_back(fv[*ia].rect);
                tracks.back()->set_lifetime(tracks[trackIdx]->get_lifetime());
                tracks.back()->set_skipped_frames(tracks[trackIdx]->get_skipped_frames());
                tracks.back()->set_catch_frames(tracks[trackIdx]->get_catch_frames());
                tracks.back()->splitMergeType=SPLITED_SMTYPE;

                tracks[trackIdx]->status=DELETE_TO_SPLIT;
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
                    tracks.push_back(std::make_unique<singleObjectTracker>(fv[*ia], param.dt, param.Accel_noise_mag, param.NextTrackID++,frameNum));


                    tracks.back()->trace=tracks[trackIdx]->trace;
                    tracks.back()->trace.push_back(fv[*ia].pos);
                    tracks.back()->rects=tracks[trackIdx]->rects;
                    tracks.back()->rects.push_back(fv[*ia].rect);
                    tracks.back()->set_lifetime(tracks[trackIdx]->get_lifetime()+1);
                    tracks.back()->set_skipped_frames(tracks[trackIdx]->get_skipped_frames());
                    tracks.back()->set_catch_frames(tracks[trackIdx]->get_catch_frames()+1);
                    tracks.back()->splitMergeType=SPLITED_SMTYPE;

                    tracks[trackIdx]->status=DELETE_TO_SPLIT;
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
        vector<Point_t> unSplitPos;
        vector<Point3i> unSplitColor;
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

            for(int i=0;i<fv[*ia].LIFPos.size();i++){
                unSplitPos.push_back(fv[*ia].LIFPos[i]);
            }

            for(int i=0;i<fv[*ia].LIFColor.size();i++){
                unSplitColor.push_back(fv[*ia].LIFColor[i]);
            }
        }

        Rect_t unSplitRect(r[0],r[1],r[2]-r[0],r[3]-r[1]);
        Point_t unSplitCenter((r[0]+r[2])*0.5f,(r[1]+r[3])*0.5f);
        trackingObjectFeature *of=(tracks[trackIdx]->feature);
        of->LIFMat=unSplitLIF;
        of->LIFPos=unSplitPos;
        of->LIFColor=unSplitColor;
        of->pos=unSplitCenter;
        of->rect=unSplitRect;
        of->size=unSplitRect.area();

        //        tracks[trackIdx]->Update(*of, true, param.max_trace_length);
        tracks[trackIdx]->NormalUpdate(*of,frameNum);
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
            //cannot merge !!!
            if(tracks[*ia]->splitMergeType==SPLITED_SMTYPE||tracks[*ia]->get_lifetime()>MaxFreshObjectLifeTime){
                continue;
            }

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

        tracks.push_back(std::make_unique<singleObjectTracker>(fv[fvIdx],
                                                               param.dt,
                                                               param.Accel_noise_mag,
                                                               param.NextTrackID++,
                                                               frameNum));

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
        tracks.push_back(std::make_unique<singleObjectTracker>(fv[*newit],
                         param.dt, param.Accel_noise_mag, param.NextTrackID++,frameNum));
    }
}

void RectFloatTracker::handleMissedObjects(){
    for(auto missit=mOneToZeroSet.begin();missit!=mOneToZeroSet.end();missit++){
        assert(*missit<tracks.size());
        trackingObjectFeature of;
        //        tracks[*missit]->skipped_frames +=1;
        //        tracks[*missit]->Update(of,false,param.max_trace_length);
        tracks[*missit]->MissUpdate(frameNum);
        if(tracks[*missit]->get_lifetime()<MaxFreshObjectLifeTime||
                tracks[*missit]->get_skipped_frames()>param.maximum_allowed_skipped_frames){
            deleteLaterObjectSet.insert(tracks[*missit]->track_id);
        }
    }
}

void RectFloatTracker::handleOneToOneObjects(){
    vector<trackingObjectFeature> &fv=featureVectorList.back();
    for(auto it=mOneToOneMap.begin();it!=mOneToOneMap.end();it++){
        int trackIdx=it->first;
        int fvIdx=it->second;
        //        tracks[trackIdx]->Update(fv[fvIdx], true, param.max_trace_length);
        tracks[trackIdx]->NormalUpdate(fv[fvIdx],frameNum);
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
        cv::rectangle(img_tracking,r,Colors[tracks[i]->track_id % 9],3);
        std::string text;
        if(tracks[i]->status==NEW_STATUS)   text="new";
        else if(tracks[i]->status==MISSING_STATUS) text="missing";
        else if(tracks[i]->status==NORMAL_STATUS) text="normal";
        else text="error";

        std::string numstr=boost::lexical_cast<string>(tracks[i]->track_id);
        text=numstr+" : "+text;

        if(r.tl().y>20){
            cv::putText(img_tracking, text, r.tl(), FONT_HERSHEY_COMPLEX, 1,
                        cv::Scalar(0,0,255), 2, 8);
        }
        else{
            Point_t p=r.tl();
            p.y=r.br().y+10;
            cv::putText(img_tracking, text, p, FONT_HERSHEY_COMPLEX, 1,
                        cv::Scalar(0,0,255), 2, 8);
        }
    }

    cv::namedWindow("img_tracking",WINDOW_NORMAL);
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
    matcher.getGoodMatches_Step3(of1->LIFMat,of2->LIFMat,good_matchers);

    return good_matchers.size();
}

void RectFloatTracker::showMatchedFeature(const cv::Mat matchMat){
    assert(!featureVectorList.empty());
    vector<trackingObjectFeature> &fv=featureVectorList.back();

    assert(!imageList.empty());
    if(imageList.size()<2) return;

    auto imgIt=imageList.end();
    imgIt=std::prev(imgIt,2);

    cv::Mat input1=imgIt->first;
    cv::Mat input2=imageList.back().first;
    cv::Mat fg1=imgIt->second;
    cv::Mat fg2=imageList.back().second;

    cv::Mat showImgInput,showImgFg;

    //step 1, draw rect
    for(int i=0;i<tracks.size();i++){
        std::shared_ptr<trackingObjectFeature> of1=std::make_shared<trackingObjectFeature>(*(tracks[i]->feature));
        cv::rectangle(input1,of1->rect,Scalar(255,0,0),3);
        cv::rectangle(fg1,of1->rect,Scalar::all(255),3);
    }
    for(int i=0;i<fv.size();i++){
        std::shared_ptr<trackingObjectFeature> of2=std::make_shared<trackingObjectFeature>(fv[i]);
        cv::rectangle(input2,of2->rect,Scalar(0,0,255),3);
        cv::rectangle(fg2,of2->rect,Scalar::all(255),3);
    }
    hconcat(input1,input2,showImgInput);
    hconcat(fg1,fg2,showImgFg);

    //step 2, draw match number, match line
    int m=matchMat.rows;
    int n=matchMat.cols;
    for(int i=0;i<m;i++){
        for(int j=0;j<n;j++){

            Point_t p1=tracks[i]->feature->pos;
            Point_t p2=fv[j].pos;
            int matchNum=(int)matchMat.at<uchar>(i,j);

            if(matchNum>StableFeatureNumber){
                yzbxlib::drawMatch(showImgInput,p1,p2,cv::Scalar(0,0,255));
            }
            else if(matchNum>0){
                yzbxlib::drawMatch(showImgInput,p1,p2,cv::Scalar(255,0,0));
            }

            if(matchNum>0){
                string text=boost::lexical_cast<string>(matchNum);
                Point_t pt=(p1+p2+Point_t(input1.cols,0))*0.5f;
                cv::putText(showImgInput, text, pt, FONT_HERSHEY_COMPLEX, 1,
                            cv::Scalar(0,255,255), 2, 8);
            }
        }
    }


    //step 3 show
    namedWindow("show matched feature input",WINDOW_NORMAL);
    namedWindow("show matched feature fg",WINDOW_NORMAL);
    imshow("show matched feature input",showImgInput);
    imshow("show matched feature fg",showImgFg);
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

void RectFloatTracker::getLocalFeatureAssignment_step2(cv::Mat &matchMat){
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
    matchMat.setTo(0);

    std::map<int,int> trackIdxToidMap;
    int offset=0;
    for(int i=0;i<m;i++){
        int newOffset=offset+tracks[i]->feature->LIFPos.size();
        for(int k=offset;k<newOffset;k++){
            trackIdxToidMap[k]=i;
        }
        offset=newOffset;
    }

    vector<Point_t> trackPos;
    vector<Point3i> trackColor;
    Mat trackMat;
    for(int i=0;i<m;i++){
        Mat &mat=tracks[i]->feature->LIFMat;

        if(mat.empty()) continue;

        if(trackMat.empty()){
            trackMat=mat.clone();

        }
        else{
            vconcat(trackMat,mat,trackMat);
        }

        vector<Point_t> &ps=tracks[i]->feature->LIFPos;
        assert(!ps.empty());
        for(int i=0;i<ps.size();i++){
            trackPos.push_back(ps[i]);
        }
        vector<Point3i> &color=tracks[i]->feature->LIFColor;
        assert(ps.size()==color.size());
        for(int i=0;i<color.size();i++){
            trackColor.push_back(color[i]);
        }
        //        std::copy(ps.begin(),ps.end(),trackPos.end());
    }

    std::map<int,int> fvIdxToIdMap;
    offset=0;
    for(int j=0;j<n;j++){
        int newOffset=offset+fv[j].LIFPos.size();
        for(int k=offset;k<newOffset;k++){
            fvIdxToIdMap[k]=j;
        }
        offset=newOffset;
    }

    Mat fvMat;
    vector<Point_t> fvPos;
    vector<Point3i> fvColor;
    for(int j=0;j<n;j++){
        Mat mat=fv[j].LIFMat;
        if(mat.empty()) continue;

        if(fvMat.empty()){
            fvMat=mat.clone();
        }
        else{
            vconcat(fvMat,mat,fvMat);
        }

        vector<Point_t> &ps=fv[j].LIFPos;
        assert(!ps.empty());
        for(int i=0;i<ps.size();i++){
            fvPos.push_back(ps[i]);
        }
        vector<Point3i> &color=fv[j].LIFColor;
        assert(ps.size()==color.size());
        for(int i=0;i<color.size();i++){
            fvColor.push_back(color[i]);
        }
    }

    if(trackMat.empty()){
        return;
    }
    if(fvMat.empty()){
        return;
    }
    vector<DMatch> good_matches;
    ObjectLocalFeatureMatch::getGoodMatches_Step3(trackMat,fvMat,good_matches);
    //    ObjectLocalFeatureMatch::getGoodMatches_Step4(good_matches,trackPos,fvPos,trackColor,fvColor);


    for(int m=0;m<good_matches.size();m++){
        int queryIdx=good_matches[m].queryIdx;
        int trainIdx=good_matches[m].trainIdx;

        int i=trackIdxToidMap[queryIdx];
        int j=fvIdxToIdMap[trainIdx];
        uchar num=matchMat.at<uchar>(i,j);
        matchMat.at<uchar>(i,j)=num+1;
    }

}
