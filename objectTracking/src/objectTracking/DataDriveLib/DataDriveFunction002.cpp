#include "DataDriveFunction002.h"

bool DataDrive::BlobToBlobAssignment_KLT::run()
{
    loadConfig();

    if(data->fvlist.size()<2) return false;
    AssignmentVecSetMap &b2b=data->BlobToBlob;
    b2b.clear();


    vector<trackingObjectFeature> &fv=data->fvlist.back();
    auto fvlistIter=std::prev(data->fvlist.end(),1);
    vector<trackingObjectFeature> &prev_fv=*fvlistIter;

    if(fv.empty()||prev_fv.empty()){
        if(!data->KLTMatchMat.empty()) data->KLTMatchMat.release();

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


    for( i = k = 0; i < points[1].size(); i++ )
    {
        circle( image, points[1][i], 3, Scalar(255,0,0), -1, 8);
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
}


void DataDrive::BlobToBlobAssignment_KLT::dump()
{

}

void DataDrive::BlobToBlobAssignment_KLT::matchToMat(const Mat &matchMat)
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

    vector<trackingObjectFeature> &fv=data->fvlist.back();
    auto fvlistIter=std::prev(data->fvlist.end(),1);
    vector<trackingObjectFeature> &prev_fv=*fvlistIter;

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

        for(uint i=0;i<m;i++){
            if(mMatchedPrevSet.find(i)==mMatchedPrevSet.end()){
                unMatchedPrev.push_back(i);
            }
        }
        for(uint j=0;j<n;j++){
            if(mMatchedCurrSet.find(j)==mMatchedCurrSet.end()){
                unMatchedCurr.push_back(j);
            }
        }

        size_t M=unMatchedCurr.size(),N=unMatchedPrev.size();
        assert(M>0&&N>0);

        distMatrix_t Cost(N * M);
        for(uint i=0;i<N;i++){
            for(uint j=0;j<M;j++){
                Cost[i+j*N]=calcDist(std::make_shared<trackingObjectFeature>(prev_fv[unMatchedPrev[i]]),
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

    return true;
}

void DataDrive::BlobToBlobAssignment_Hungarian::dump()
{

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

    vector<trackingObjectFeature> &fv=data->fvlist.back();
    auto fvlistIter=std::prev(data->fvlist.end(),1);
    vector<trackingObjectFeature> &prev_fv=*fvlistIter;
    bool emptyPrevBlobs=prev_fv.empty();
    bool emptyCurrBlobs=fv.empty();
    bool emptyObjects=data->tracks.empty();

    if(emptyPrevBlobs&&emptyCurrBlobs&&emptyObjects){

    }
    else if(emptyCurrBlobs){
        //handle missed blob
        for(uint i=0;i<prev_fv.size();i++){
            data->BlobToBlob.OneToZeroSet.insert(i);
        }
        B2BOneToZero();
    }
    else if(emptyPrevBlobs){
        //handle new blob
        for(uint i=0;i<fv.size();i++){
            data->BlobToBlob.ZeroToOneSet.insert(i);
        }
        B2BZeroToOne();
    }
    else{
        B2BDiscuss();
        B2BNToN();
        B2BNToOne();
        B2BOneToN();
        B2BOneToZero();
        B2BZeroToOne();
    }
    return true;
}

void DataDrive::BlobToBlobAssignment_SplitMerge::dump()
{

}

void DataDrive::BlobToBlobAssignment_SplitMerge::B2BDiscuss()
{
    auto &b2b=data->BlobToBlob;

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
        else if(BSet.size()>1){
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
}

void DataDrive::BlobToBlobAssignment_SplitMerge::B2BNToOne()
{
    auto &b2b=data->BlobToBlob;
    for(auto it=b2b.NToOneMap.begin();it!=b2b.NToOneMap.end();it++){
        Index_t currIdx=it->first;
        std::set<Index_t> &prevSet=it->second;
        for(auto ip=prevSet.begin();ip!=prevSet.end();ip++){
            Index_t prevIdx=*ip;
            //prev track set.
            std::set<Index_t> &s=prevB2T[prevIdx];
            currB2T[currIdx].insert(s.begin(),s.end());
            for(auto it=s.begin();it!=s.end();it++){
                Index_t currTrack=*it;
                currT2B[currTrack].insert(currIdx);
            }

        }

        T2BNToOne(currB2T[currIdx],currIdx);
    }
}

void DataDrive::BlobToBlobAssignment_SplitMerge::B2BNToN()
{

}

void DataDrive::BlobToBlobAssignment_SplitMerge::B2BOneToN()
{
    auto &b2b=data->BlobToBlob;
    for(auto it=b2b.OneToNMap.begin();it!=b2b.OneToNMap.end();it++){
        Index_t prevIdx=it->first;
        std::set<Index_t> &currSet=it->second;
        ReHungarian(prevB2T[prevIdx],currSet);
    }
}

void DataDrive::BlobToBlobAssignment_SplitMerge::B2BOneToOne()
{
    auto &set=data->BlobToBlob.OneToOneMap;
    for(auto it=set.begin();it!=set.end();it++){
        Index_t prevIdx=it->first;
        Index_t currIdx=it->second;
        if(prevB2T[prevIdx].size()==1){
            Index_t trackIdx=*prevB2T[prevIdx].begin();
            T2BOneToOne(trackIdx,currIdx);
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
            Index_t trackIdx=*trackIt;
            T2BOneToZero(trackIdx);
        }
    }
}

void DataDrive::BlobToBlobAssignment_SplitMerge::B2BZeroToOne()
{
    auto &set=data->BlobToBlob.ZeroToOneSet;
    for(auto it=set.begin();it!=set.end();it++){
        Index_t currIdx=*it;
        T2BZeroToOne(currIdx);
    }
}

void DataDrive::BlobToBlobAssignment_SplitMerge::ReHungarian(const std::set<Index_t> trackSet, const std::set<Index_t> &currSet)
{
    int N=trackSet.size(),M=currSet.size();
    distMatrix_t Cost(N*M);
    for(uint i=0;i<N;i++){
        Index_t trackIdx=*(trackSet.begin()+i);
        for(uint j=0;j<M;j++){
            Index_t currIdx=*(currSet.begin()+j);
            Cost[i+j*N]=getDistCost(trackIdx,currIdx);
        }
    }

    AssignmentProblemSolver APS;
    assignments_t unMatchedAssignment;
    APS.Solve(Cost, N, M, unMatchedAssignment, AssignmentProblemSolver::optimal);

}

double DataDrive::BlobToBlobAssignment_SplitMerge::getDistCost(Index_t trackIdx, Index_t currIdx)
{
    vector<trackingObjectFeature> &fv=data->fvlist.back();
    cv::Mat newBlobImg=data->img_input(fv[currIdx].rect);
    cv::Mat newBlobMask=fv[currIdx].mask(fv[currIdx].rect);
    assert(!newBlobImg.empty()&&!newBlobMask.empty());

    cv::Rect_t trackRect=data->tracks[trackIdx]->rect_last_normal;
    cv::Mat trackImg=data->tracks[trackIdx]->img_last_normal(trackRect);
    cv::Mat trackMask=data->tracks[trackIdx]->mask_last_normal(trackRect);

    ReDetection RR;
    double hog=RR.myCompareHog(trackImg,newBlobImg);
    double color=RR.myCompareColor(trackImg,newBlobImg);

    return hog*weight_hog+color*weight*weight_color;
}
