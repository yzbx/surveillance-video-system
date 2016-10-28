#include "DataDriveFunction002.h"

bool DataDrive::BlobToBlobAssignment_KLT::run()
{
    loadConfig();

    if(data->fvlist.size()<2) return false;
    AssignmentVecSetMap &b2b=data->BlobToBlob;
    b2b.clear();


    const vector<trackingObjectFeature> &fv=data->fvlist.back();
    auto fvlistIter=std::prev(data->fvlist.end(),2);
    const vector<trackingObjectFeature> &prev_fv=*fvlistIter;

    if(fv.empty()||prev_fv.empty()){
        return true;
    }

    assert(data->img_input.channels()==3);

    auto imglistIter=std::prev(data->imglist.end(),2);
    cv::Mat gray,prev_gray;
    cv::cvtColor(data->img_input,gray,CV_BGR2GRAY);
    cv::cvtColor(imglistIter->first,prev_gray,CV_BGR2GRAY);

    TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
    Size subPixWinSize(10,10), winSize(31,31);
    const int MAX_COUNT = 500;

    vector<Point_KLT> points[2];
    goodFeaturesToTrack(prev_gray, points[0], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
    cornerSubPix(prev_gray, points[0], subPixWinSize, Size(-1,-1), termcrit);

    vector<uchar> status;
    vector<float> err;
    calcOpticalFlowPyrLK(prev_gray, gray, points[0], points[1], status, err, winSize,
            3, termcrit, 0, 0.001);

    assert(points[0].size()==points[1].size());

    // for matchNum > 255
    cv::Mat matchMat(prev_fv.size(),fv.size(),CV_8UC1);
    matchMat=Scalar::all(0);

    int i,k;
    for(i = k = 0; i < (int)points[1].size(); i++ )
    {
        if( !status[i] )
            continue;

        points[0][k]=points[0][i];
        points[1][k++] = points[1][i];
        Point2i p=points[1][i];
        Point2i prev_p=points[0][i];
        int matchMat_x=-1,matchMat_y=-1;
        for(uint i=0;i<prev_fv.size();i++){
            cv::Mat mask=prev_fv[i].mask;

            assert(!mask.empty());
            if(mask.at<uchar>(prev_p.y,prev_p.x)==255){
                matchMat_y=i;
                break;
            }
        }
        for(uint i=0;i<fv.size();i++){
            cv::Mat mask=fv[i].mask;

            assert(!mask.empty());
            if(mask.at<uchar>(p.y,p.x)==255){
                matchMat_x=i;
                break;
            }
        }

        if(matchMat_x==-1||matchMat_y==-1){
            //out of mask
        }
        else{
            matchMat.at<uchar>(matchMat_y,matchMat_x)+=((uchar)1);
        }
    }

    matToAssignment(matchMat);

    dump();
    //check
    b2b.checkAssignment();
    return true;
}


void DataDrive::BlobToBlobAssignment_KLT::dump()
{
    const vector<trackingObjectFeature> &fv=data->fvlist.back();
    auto fvlistIter=std::prev(data->fvlist.end(),2);
    const vector<trackingObjectFeature> &prev_fv=*fvlistIter;
    if(fv.size()>1||prev_fv.size()>1){
        data->BlobToBlob.dumpMatch();
    }
}

void DataDrive::BlobToBlobAssignment_KLT::matToAssignment(const Mat &matchMat)
{
    assert(configLoaded);
    AssignmentVecSetMap &b2b=data->BlobToBlob;
    int img_rows=matchMat.rows;
    int img_cols=matchMat.cols;
    for(int i=0;i<img_rows;i++){
        for(int j=0;j<img_cols;j++){
            uchar c=matchMat.at<uchar>(i,j);
            if(c>MinMatchThreshold){
                b2b.AToB[i].insert(j);
                b2b.BToA[j].insert(i);
                b2b.matchedASet.insert(i);
                b2b.matchedBSet.insert(j);
            }
        }
    }
}

bool DataDrive::BlobToBlobAssignment_Hungarian::run()
{
    loadConfig();

    if(data->fvlist.size()<2) return false;

    const vector<trackingObjectFeature> &fv=data->fvlist.back();
    auto fvlistIter=std::prev(data->fvlist.end(),2);
    const vector<trackingObjectFeature> &prev_fv=*fvlistIter;

    std::set<Index_t> &mMatchedPrevSet=data->BlobToBlob.matchedASet;
    std::set<Index_t> &mMatchedCurrSet=data->BlobToBlob.matchedBSet;

    int m=prev_fv.size(),n=fv.size();
    int um=m-mMatchedPrevSet.size(),un=n-mMatchedCurrSet.size();
    if(um==0||un==0){

    }
    else{
        //vector<trackIdx>
        vector<Index_t> unMatchedPrev;
        //vector<featureVectorIdx>
        vector<Index_t> unMatchedCurr;

        for(int i=0;i<m;i++){
            if(mMatchedPrevSet.find(i)==mMatchedPrevSet.end()){
                unMatchedPrev.push_back(i);
            }
        }
        for(int j=0;j<n;j++){
            if(mMatchedCurrSet.find(j)==mMatchedCurrSet.end()){
                unMatchedCurr.push_back(j);
            }
        }

        size_t M=unMatchedCurr.size(),N=unMatchedPrev.size();
        assert(M>0&&N>0);

        distMatrix_t Cost(N * M);
        for(uint i=0;i<N;i++){
            for(uint j=0;j<M;j++){
                Cost[i+j*N]=DataDrive::calcDist(std::make_shared<trackingObjectFeature>(prev_fv[unMatchedPrev[i]]),
                        std::make_shared<trackingObjectFeature>(fv[unMatchedCurr[j]]),RectDist);
            }
        }

        AssignmentProblemSolver APS;
        assignments_t unMatchedAssignment;
        APS.Solve(Cost, N, M, unMatchedAssignment, AssignmentProblemSolver::optimal);

        auto &b2b=data->BlobToBlob;
        for (size_t i = 0; i < unMatchedAssignment.size(); i++)
        {
            if (unMatchedAssignment[i] != -1)
            {
                if (Cost[i + unMatchedAssignment[i] * N] > MinUnMatchDistanceThreshold)
                {
                    unMatchedAssignment[i] = -1;
                }
                else{
                    Index_t PrevIdx=unMatchedPrev[i];
                    Index_t CurrIdx=unMatchedCurr[unMatchedAssignment[i]];

                    b2b.AToB[PrevIdx].insert(CurrIdx);
                    b2b.BToA[CurrIdx].insert(PrevIdx);
                    b2b.matchedASet.insert(PrevIdx);
                    b2b.matchedBSet.insert(CurrIdx);
                }
            }
        }
    }

    dump();

    data->BlobToBlob.checkAssignment();
    return true;
}

void DataDrive::BlobToBlobAssignment_Hungarian::dump()
{
    const vector<trackingObjectFeature> &fv=data->fvlist.back();
    auto fvlistIter=std::prev(data->fvlist.end(),2);
    const vector<trackingObjectFeature> &prev_fv=*fvlistIter;
    if(fv.size()>1||prev_fv.size()>1){
        data->BlobToBlob.dumpMatch();
    }
}

bool DataDrive::BlobToBlobAssignment_SplitMerge::run()
{
    loadConfig();
    if(data->fvlist.size()<2){
        assert(false);
    }

    if(!data->mOneToZeroSet.empty())data->mOneToZeroSet.clear();
    if(!data->mZeroToOneSet.empty())data->mZeroToOneSet.clear();
    if(!data->mOneToNMap.empty())data->mOneToNMap.clear();
    if(!data->mOneToOneMap.empty())data->mOneToOneMap.clear();
    if(!data->mNToOneMap.empty())data->mNToOneMap.clear();

    const vector<trackingObjectFeature> &fv=data->fvlist.back();
    auto fvlistIter=std::prev(data->fvlist.end(),2);
    const vector<trackingObjectFeature> &prev_fv=*fvlistIter;
    bool emptyPrevBlobs=prev_fv.empty();
    bool emptyCurrBlobs=fv.empty();
    bool emptyObjects=data->tracks.empty();

    B2BDiscuss();
    data->BlobToBlob.dumpAll();
    dumpBlobToTrack(prevB2T);
    if(emptyPrevBlobs&&emptyCurrBlobs&&emptyObjects){

    }
    else{
        B2BNToN();
        B2BNToOne();
        B2BOneToN();
        B2BOneToOne();
        B2BOneToZero();
        B2BZeroToOne();
        gatherMissedTrackSet();
        t2b.dumpReDetection();
        dumpBlobToTrack(currB2T);
        T2BReDetection();
        dumpBlobToTrack(currB2T);
        update();
    }

    dumpToTxt();
    check();
    return true;
}

void DataDrive::BlobToBlobAssignment_SplitMerge::dump()
{

}

void DataDrive::BlobToBlobAssignment_SplitMerge::B2BDiscuss()
{
    auto &b2b=data->BlobToBlob;

    //1-1,N-1,1-N,N-N
    for(auto ia=b2b.AToB.begin();ia!=b2b.AToB.end();ia++){
        Index_t AIdx=ia->first;;
        std::set<Index_t> &Bset=ia->second;
        if(Bset.size()==1){
            Index_t BIdx=*(Bset.begin());
            if(b2b.BToA[BIdx].size()==1){
                b2b.OneToOneMap[AIdx]=BIdx;
            }
            else{
                b2b.NToOneMap[BIdx].insert(AIdx);
            }
        }
        else if(Bset.size()>1){
            for(auto ib=Bset.begin();ib!=Bset.end();ib++){
                Index_t BIdx=*ib;
                if(b2b.BToA[BIdx].size()==1){
                    b2b.OneToNMap[AIdx].insert(BIdx);
                }
                else{
                    b2b.NToNMap[AIdx].insert(BIdx);
                }
            }
        }
    }

    //1-0,0-1
    auto currFvit=std::prev(data->fvlist.end(),1);
    auto prevFvit=std::prev(data->fvlist.end(),2);
    vector<trackingObjectFeature> &currFv=*currFvit;
    vector<trackingObjectFeature> &prevFv=*prevFvit;

    for(int i=0;i<currFv.size();i++){
        if(b2b.matchedBSet.find(i)==b2b.matchedBSet.end()){
            b2b.ZeroToOneSet.insert(i);
        }
    }
    for(int i=0;i<prevFv.size();i++){
        if(b2b.matchedASet.find(i)==b2b.matchedASet.end()){
            b2b.OneToZeroSet.insert(i);
        }
    }

    assert(prevFv.size()==b2b.matchedASet.size()+b2b.OneToZeroSet.size());
    assert(currFv.size()==b2b.matchedBSet.size()+b2b.ZeroToOneSet.size());
}

void DataDrive::BlobToBlobAssignment_SplitMerge::B2BNToOne()
{
    auto &b2b=data->BlobToBlob;
    for(auto it=b2b.NToOneMap.begin();it!=b2b.NToOneMap.end();it++){
        Index_t currIdx=it->first;
        std::set<Index_t> &prevSet=it->second;
        std::set<Index_t> trackset;
        for(auto ip=prevSet.begin();ip!=prevSet.end();ip++){
            Index_t prevIdx=*ip;
            //prev track set.
            const std::set<Id_t> &s=prevB2T[prevIdx];
            std::set<Index_t> idxset;
            id2index(s,idxset);
            trackset.insert(idxset.begin(),idxset.end());
        }
        T2BNToOne(trackset,currIdx);
    }
}

void DataDrive::BlobToBlobAssignment_SplitMerge::B2BNToN()
{
    if(!data->BlobToBlob.NToNMap.empty()){
        //        qDebug()<<"NOTE: B2BNToN is ignored!!!";
        qDebug()<<"do with B2BNToN";
        for(auto it=data->BlobToBlob.NToNMap.begin();it!=data->BlobToBlob.NToNMap.end();it++){
            Index_t prevIdx=it->first;
            const std::set<Index_t> &currSet=it->second;
            assert(data->BlobToBlob.AToB[prevIdx].size()>1);
            if(data->BlobToBlob.OneToNMap.find(prevIdx)==data->BlobToBlob.OneToNMap.end()){
                data->BlobToBlob.OneToZeroSet.insert(prevIdx);
            }
            for(auto it=currSet.begin();it!=currSet.end();it++){
                Index_t currIdx=*it;
                assert(data->BlobToBlob.BToA[currIdx].size()>1);
                if(data->BlobToBlob.NToOneMap.find(currIdx)==data->BlobToBlob.NToOneMap.end()){
                    data->BlobToBlob.ZeroToOneSet.insert(currIdx);
                }
            }
        }
    }

}

void DataDrive::BlobToBlobAssignment_SplitMerge::B2BOneToN()
{
    auto &b2b=data->BlobToBlob;
    for(auto it=b2b.OneToNMap.begin();it!=b2b.OneToNMap.end();it++){
        Index_t prevIdx=it->first;
        std::set<Index_t> &currSet=it->second;
        std::set<Index_t> trackset,missset;
        id2index(prevB2T[prevIdx],missset);

        for(auto it=missset.begin();it!=missset.end();it++){
            Index_t trackIdx=*it;
            if(needReDetection(trackIdx)){
                trackset.insert(trackIdx);
            }
            else{
                deleteTSet.insert(data->tracks[trackIdx]->track_id);
            }
        }

        if(trackset.empty()){
            //add to 0-1
            auto &set=data->BlobToBlob.ZeroToOneSet;
            set.insert(currSet.begin(),currSet.end());
        }
        else{
            ReHungarian(trackset,currSet);
        }
    }
}

void DataDrive::BlobToBlobAssignment_SplitMerge::B2BOneToOne()
{
    auto &set=data->BlobToBlob.OneToOneMap;
    for(auto it=set.begin();it!=set.end();it++){
        Index_t prevIdx=it->first;
        Index_t currIdx=it->second;
        if(prevB2T[prevIdx].size()==1){
            Id_t id=*prevB2T[prevIdx].begin();
            Index_t trackIdx=id2index(id);
            T2BOneToOne(trackIdx,currIdx);
        }
        else if(prevB2T[prevIdx].size()>1){
            std::set<Index_t> idxset;
            id2index(prevB2T[prevIdx],idxset);
            T2BNToOne(idxset,currIdx);
        }
    }
}

void DataDrive::BlobToBlobAssignment_SplitMerge::B2BOneToZero()
{
    auto &set=data->BlobToBlob.OneToZeroSet;
    for(auto it=set.begin();it!=set.end();it++){
        Index_t prevIdx=*it;
        auto &trackSet=prevB2T[prevIdx];
        for(auto trackIt=trackSet.begin();trackIt!=trackSet.end();trackIt++){
            Id_t id=*trackIt;
            Index_t trackIdx=id2index(id);
            T2BOneToZero_Append(trackIdx);
        }
    }
}

void DataDrive::BlobToBlobAssignment_SplitMerge::B2BZeroToOne()
{
    auto &set=data->BlobToBlob.ZeroToOneSet;
    for(auto it=set.begin();it!=set.end();it++){
        Index_t currIdx=*it;
        T2BZeroToOne_Append(currIdx);
    }
}

void DataDrive::BlobToBlobAssignment_SplitMerge::ReHungarian(const std::set<Index_t> trackSet, const std::set<Index_t> &currSet,bool isB2BOneToN)
{
    int N=trackSet.size(),M=currSet.size();
    distMatrix_t Cost(N*M);
    for(int i=0;i<N;i++){
        Index_t trackIdx=*(std::next(trackSet.begin(),i));
        for(int j=0;j<M;j++){
            Index_t currIdx=*std::next(currSet.begin(),j);
            Cost[i+j*N]=getDistCost(trackIdx,currIdx);
        }
    }

    AssignmentProblemSolver APS;
    assignments_t unMatchedAssignment;
    APS.Solve(Cost, N, M, unMatchedAssignment, AssignmentProblemSolver::optimal);

    //TODO reassignment!!!
    std::set<Index_t> unmatched=currSet;
    assert((&unmatched)!=(&currSet));

    for(uint i=0;i<unMatchedAssignment.size();i++){
        Index_t trackIdx=*std::next(trackSet.begin(),i);
        if(unMatchedAssignment[i]!=-1){
            Index_t currIdx=*std::next(currSet.begin(),unMatchedAssignment[i]);
            if(unmatched.find(currIdx)!=unmatched.end()){
                unmatched.erase(currIdx);
                T2BOneToOne(trackIdx,currIdx);
            }
            else{
                //Do OneToOne update only once.
                qWarning()<<"need reaasignment!!!";
            }
        }
        else{
            if(isB2BOneToN)
                T2BOneToZero_Append(trackIdx);
            else
                T2BOneToZero_Append(trackIdx);
        }
    }

    //BUG
    for(auto it=unmatched.begin();it!=unmatched.end();it++){
        Index_t currIdx=*it;
        if(isB2BOneToN)
            T2BZeroToOne_Append(currIdx);
        else
            T2BZeroToOne_Append(currIdx);
    }
}

double DataDrive::BlobToBlobAssignment_SplitMerge::getDistCost(Index_t trackIdx, Index_t currIdx)
{
    vector<trackingObjectFeature> &fv=data->fvlist.back();
    cv::Mat newBlobImg=data->img_input(fv[currIdx].rect);
    cv::Mat newBlobMask=fv[currIdx].mask(fv[currIdx].rect);
    assert(!newBlobImg.empty()&&!newBlobMask.empty());

    Rect_t trackRect=data->tracks[trackIdx]->rect_last_normal;
    cv::Mat trackImg=data->tracks[trackIdx]->img_last_normal(trackRect);
    cv::Mat trackMask=data->tracks[trackIdx]->mask_last_normal(trackRect);

    ReDetection RR;
    double hog=RR.myCompareHog(trackImg,newBlobImg);
    double color=RR.myCompareColor(trackImg,newBlobImg);

    return hog*weight_hog+color*weight_color;
}

bool DataDrive::BlobToBlobAssignment_SplitMerge::needReDetection(Index_t trackIdx)
{
//    int lifetime=data->tracks[trackIdx]->get_lifetime();
    int cacheTime=data->tracks[trackIdx]->get_catch_frames();
    bool onBoundary=data->tracks[trackIdx]->feature->onBoundary;
    if(cacheTime>(int)data->param.MaxFreshObjectLifeTime&&!onBoundary){
        return true;
    }

    return false;
}

bool DataDrive::BlobToBlobAssignment_SplitMerge::needRemove(Index_t trackIdx)
{
    int misstime=data->tracks[trackIdx]->get_skipped_frames();
    if(misstime>(int)data->param.maximum_allowed_skipped_frames){
        return true;
    }
    else{
        return false;
    }
}

void DataDrive::BlobToBlobAssignment_SplitMerge::T2BNToOne(const std::set<Index_t> &trackSet, Index_t currIdx)
{
    const vector<trackingObjectFeature> &fv=data->fvlist.back();
    Rect_t rb=fv[currIdx].rect;
    for(auto it=trackSet.begin();it!=trackSet.end();it++){
        Index_t trackIdx=*it;
        Point_t pos=data->tracks[trackIdx]->feature->pos;
        Rect_t rect=data->tracks[trackIdx]->feature->rect;
        Point_t predict_pos=data->tracks[trackIdx]->KF.GetPrediction();
        Rect_t predict_rect=data->tracks[trackIdx]->GetPredictRect();
        Rect_t subRect=yzbxlib::getSubRect(rb,predict_rect);
        //kalman speed v1
        Point_t speed=predict_pos-pos;
        //in rect adjust speed v2
        Point_t adjustSpeed=subRect.tl()-rect.tl();

        //v1 and v2 is close!!!
        double vec_norm=cv::norm(speed)*cv::norm(adjustSpeed);
        double cos_theta;
        if(vec_norm==0) cos_theta=0;
        else cos_theta=(speed.x*adjustSpeed.x+speed.y*adjustSpeed.y)/vec_norm;

        trackingObjectFeature of;
        of.copy(*(data->tracks[trackIdx]->feature));
        if(cos_theta>0.7){
            of.rect=subRect;
            of.pos+=(subRect.tl()-rect.tl());
        }
        else{
            of.rect=predict_rect;
            of.pos+=(predict_rect.tl()-rect.tl());
        }
        //B2B_NToOneUpdate increate skip_frames but NToOneUpdate not.
        //So to avoid delete object merged, we use the latter.
        //But in B2B, we only delete missed/unmatched object
        //the object merged will never be delete!!!
        data->tracks[trackIdx]->B2B_NToOneUpdate(of,data->frameNum);
    }

    for(auto it=trackSet.begin();it!=trackSet.end();it++){
        Index_t trackIdx=*it;
        Id_t id=data->tracks[trackIdx]->track_id;
        currB2T[currIdx].insert(id);
    }

}

void DataDrive::BlobToBlobAssignment_SplitMerge::T2BOneToOne(Index_t trackIdx, Index_t currIdx)
{
    const vector<trackingObjectFeature> &fv=data->fvlist.back();
    assert(fv[currIdx].rect.area()>data->param.MinBlobArea);
    data->tracks[trackIdx]->NormalUpdate(fv[currIdx],data->frameNum,data->img_input);
    Id_t id=data->tracks[trackIdx]->track_id;
    currB2T[currIdx].insert(id);
    assert(currB2T[currIdx].size()==1);
}

//void DataDrive::BlobToBlobAssignment_SplitMerge::T2BOneToZero(Index_t trackIdx)
//{
//    if(needReDetection(trackIdx)){
//        std::set<Index_t> &set=data->BlobToBlob.ZeroToOneSet;
//        for(int it=set.begin();it!=set.end();it++){
//            Index_t currIdx=*it;
//            double dist=getDistCost(trackIdx,currIdx);
//            if(dist<threshold){
//                T2BOneToOne(trackIdx,currIdx);
//                return;
//            }
//        }
//    }

//    if(!needRemove(trackIdx)){
//        data->tracks[trackIdx]->MissUpdate(data->frameNum);
//        return;
//    }
//    deleteTSet.insert(trackIdx);
//}

void DataDrive::BlobToBlobAssignment_SplitMerge::T2BOneToZero_Append(Index_t trackIdx)
{
    t2b.OneToZeroSet.insert(trackIdx);
}

//void DataDrive::BlobToBlobAssignment_SplitMerge::T2BZeroToOne(Index_t currIdx)
//{
//data->tracks.push_back(std::make_unique<singleObjectTracker>(fv[currIdx],
//                       data->param.dt, data->param.Accel_noise_mag, data->NextTrackID++,data->frameNum,data->img_input.cols,data->img_input.rows));
//}

void DataDrive::BlobToBlobAssignment_SplitMerge::T2BZeroToOne_Append(Index_t currIdx)
{
    t2b.ZeroToOneSet.insert(currIdx);
}

void DataDrive::BlobToBlobAssignment_SplitMerge::T2BReDetection()
{
    const vector<trackingObjectFeature> &fv=data->fvlist.back();
    auto missSet=t2b.OneToZeroSet;
    auto currSet=t2b.ZeroToOneSet;
    std::set<Index_t> trackSet;
    std::set<Index_t> newSet;
    for(auto it=missSet.begin();it!=missSet.end();it++){
        Index_t trackIdx=*it;
        if(needReDetection(trackIdx)){
            trackSet.insert(trackIdx);
        }
        else{
            deleteTSet.insert(data->tracks[trackIdx]->track_id);
        }
    }

    if(trackSet.empty()){
        missSet.clear();
        newSet=currSet;
    }
    else if(currSet.empty()){
        missSet=trackSet;
    }
    else{
        t2b.OneToZeroSet.clear();
        t2b.ZeroToOneSet.clear();
        assert(!trackSet.empty());
        assert(!currSet.empty());
        ReHungarian(trackSet,currSet);
        missSet=t2b.OneToZeroSet;
        newSet=t2b.ZeroToOneSet;
    }

    for(auto it=missSet.begin();it!=missSet.end();it++){
        Index_t trackIdx=*it;
        data->tracks[trackIdx]->MissUpdate(data->frameNum);
    }

    for(auto it=newSet.begin();it!=newSet.end();it++){
        Index_t currIdx=*it;
        assert(fv[currIdx].rect.area()>data->param.MinBlobArea);
        data->tracks.push_back(std::make_unique<singleObjectTracker>(fv[currIdx],
                                                                     data->param.dt, data->param.Accel_noise_mag, data->NextTrackID++,data->frameNum,data->img_input.cols,data->img_input.rows));

        currB2T[currIdx].insert(data->NextTrackID-1);
        assert(currB2T[currIdx].size()==1);
    }

    t2b.OneToZeroSet.clear();
    t2b.ZeroToOneSet.clear();

    for(auto it=deleteTSet.begin();it!=deleteTSet.end();it++){
        Id_t id=*it;
        Index_t trackIdx=id2index(id);
        data->tracks.erase(data->tracks.begin()+trackIdx);
    }

    deleteTSet.clear();
}

void DataDrive::BlobToBlobAssignment_SplitMerge::dumpBlobToTrack(const std::map<Index_t, std::set<Id_t> > &b2t)
{
    cout<<"BlobToBlobAssignment_SplitMerge::dumpBlobToTrack ********"<<endl;
    for(auto it=b2t.begin();it!=b2t.end();it++){
        Index_t blobIdx=it->first;
        const std::set<Id_t> &trackIds=it->second;
        std::set<Index_t> trackIdxs;
        id2index(trackIds,trackIdxs);
        cout<<blobIdx<<"(blob)->(track id)";
        for(auto it=trackIds.begin();it!=trackIds.end();it++){
            cout<<*it<<",";
        }
        cout<<endl;
        cout<<blobIdx<<"(blob)->(track idx)";
        for(auto it=trackIdxs.begin();it!=trackIdxs.end();it++){
            cout<<*it<<",";
        }
        cout<<endl;
    }
}

void DataDrive::BlobToBlobAssignment_SplitMerge::gatherMissedTrackSet()
{
    std::set<Index_t> missSet;
    for(int i=0;i<data->tracks.size();i++){
        missSet.insert(i);
    }

    for(auto it=prevB2T.begin();it!=prevB2T.end();it++){
        std::set<Id_t> &set=it->second;
        for(auto it=set.begin();it!=set.end();it++){
            Index_t idx=id2index(*it);
            missSet.erase(idx);
        }
    }

    //add miss set to t2b
    t2b.OneToZeroSet.insert(missSet.begin(),missSet.end());
}

Index_t DataDrive::BlobToBlobAssignment_SplitMerge::id2index(Id_t id)
{
    for(uint i=0;i<data->tracks.size();i++){
        if(data->tracks[i]->track_id==(int)id){
            return i;
        }
    }
    assert(false);
    return 0;
}

void DataDrive::BlobToBlobAssignment_SplitMerge::id2index(const std::set<Id_t> &idset, std::set<Index_t> &idxset)
{
    assert(idxset.empty());
    for(auto it=idset.begin();it!=idset.end();it++){
        Id_t id=*it;
        for(uint i=0;i<data->tracks.size();i++){
            if(data->tracks[i]->track_id==(int)id){
                idxset.insert(i);
                break;
            }
        }
    }
    assert(idxset.size()==idset.size());
}

void DataDrive::BlobToBlobAssignment_SplitMerge::update()
{
    prevB2T.clear();
    prevB2T.swap(currB2T);
    assert(currB2T.empty());
}

void DataDrive::BlobToBlobAssignment_SplitMerge::dumpToTxt()
{
    for(int i=0;i<data->tracks.size();i++){
        Index_t trackIdx=i;
        if(data->tracks[trackIdx]->get_lifetime()<=data->param.MinDumpLifeTime){
            continue;
        }

        vector<Rect_t> &rects=data->tracks[trackIdx]->rects;
        vector<Point_t> &trace=data->tracks[trackIdx]->trace;
        vector<int> &frames=data->tracks[trackIdx]->frames;
        vector<STATUS> &vec_status=data->tracks[trackIdx]->vec_status;

        int n=rects.size();
        assert(n==trace.size());
        assert(n==frames.size());
        assert(n==vec_status.size());

        QString idstr=QString::number(data->tracks[trackIdx]->track_id);

        assert(!data->recordFile.isEmpty());
        QFile file(data->recordFile);
        if(!file.open(QIODevice::WriteOnly|QIODevice::Append)){
            assert(false);
        }
        QTextStream out(&file);
        if(vec_status[n-1]!=MISSING_STATUS){
            QStringList line;
            int x=(int)rects[n-1].x;
            int y=(int)rects[n-1].y;
            int width=(int)rects[n-1].width;
            int height=(int)rects[n-1].height;
            line<<QString::number(frames[n-1])<<idstr<<QString::number(x)<<QString::number(y)
               <<QString::number(width)<<QString::number(height);

            out<<line.join(",")<<"\n";
        }
        file.close();
    }
}

bool DataDrive::BlobToBlobAssignment_SplitMerge::check()
{
//    data->BlobToBlob.checkAssignment();
    assert(t2b.OneToZeroSet.empty());
    assert(t2b.ZeroToOneSet.empty());
    t2b.checkAssignment();
    assert(deleteTSet.empty());
    assert(currB2T.empty());

    //for every track, we must update once and only once.
    for(int i=0;i<data->tracks.size();i++){
        assert(data->frameNum>=data->tracks[i]->frames.back());
        assert(data->frameNum<=data->tracks[i]->frames.back());
//        int lifetime=data->tracks[i]->get_lifetime();
        int cacheTime=data->tracks[i]->get_catch_frames();
        if(cacheTime>(int)data->param.MaxFreshObjectLifeTime){
            assert(data->tracks[i]->status!=NEW_STATUS);
            assert(data->tracks[i]->rect_last_normal.area()>=data->param.MinBlobArea);
        }
    }
    return true;
}



bool DataDrive::TrackingDump::run()
{
    //output assignment result
    vector<trackingObjectFeature> &fv=data->fvlist.back();
    auto imgIt=std::prev(data->imglist.end(),1);
    auto prev_imgIt=std::prev(data->imglist.end(),2);

    Mat input,mask,prev_input,prev_mask;
    input=imgIt->first.clone();
    mask=imgIt->second.clone();
    prev_input=prev_imgIt->first.clone();
    prev_mask=prev_imgIt->second.clone();
    //blob
    for(uint i=0;i<fv.size();i++){
        Rect_t r=fv[i].rect;
        string idxstr=boost::lexical_cast<string>(i);
        //        string title="index="+idxstr;
        string title=idxstr;
        yzbxlib::annotation(input,r,title);
        rectangle(input,r,Scalar(0,0,255),3,8);
        rectangle(mask,r,Scalar::all(255),3,8);
    }

    //object
    for(uint i=0;i<data->tracks.size();i++){
        Rect_t r=data->tracks[i]->feature->rect;
        Index_t trackIdx=data->tracks[i]->track_id;
        string idxstr=boost::lexical_cast<string>(trackIdx);
        string title="id="+idxstr;
        yzbxlib::annotation(prev_input,r,title);
        rectangle(prev_input,r,Scalar(0,0,255),3,8);
        rectangle(prev_mask,r,Scalar::all(255),3,8);
    }

    Mat blob,object;
    cvtColor(mask,mask,CV_GRAY2BGR);
    hconcat(input,mask,blob);
    cvtColor(prev_mask,prev_mask,CV_GRAY2BGR);
    hconcat(prev_input,prev_mask,object);

    Mat klt;
    vconcat(object,blob,klt);
    string title=boost::lexical_cast<string>(data->frameNum);
    yzbxlib::annotation(klt,Point(20,20),title);

    namedWindow("TrackingDump",WINDOW_NORMAL);
    imshow("TrackingDump",klt);

    return true;
}
