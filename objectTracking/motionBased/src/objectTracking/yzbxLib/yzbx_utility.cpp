#include "yzbx_utility.h"

unsigned yzbx_hamdist(unsigned x, unsigned y)
{
    //  11111 11100
    unsigned dist = 0, val = x ^ y; // XOR

    // Count the number of set bits
    while(val)
    {
        ++dist;
        val &= val - 1;
    }

    return dist;
}

unsigned yzbx_d1(unsigned x,unsigned y){
    if(x>y) return x-y;
    else return y-x;
}

unsigned yzbx_d2(unsigned x,unsigned y){
    if(x>y) return (x-y)*(x-y);
    else return (y-x)*(y-x);
}

unsigned yzbx_distance (unsigned x, unsigned y, int distance_type){
    CV_Assert(distance_type<=2&&distance_type>=0);
    if(distance_type==0) return yzbx_hamdist (x,y);
    if(distance_type==1) return yzbx_d1 (x,y);
    if(distance_type==2) return yzbx_d2(x,y);

    CV_Assert(false);
    return 0;
}

void showImgInLoop (const cv::Mat img, int i, string title){
    stringstream ss;
    ss<<i;
    string name;
    ss>>name;
    if(title.empty())   cv::imshow(name,img);
    else cv::imshow (title+" "+name,img);
}

void yzbx_imfill(Mat &input){
    //assume input is uint8 B & W (0 or FG)
    //this function imitates imfill(image,'hole')

    //    const int FG=255;
    const int FLOODED=100;
    cv::Mat holes=input.clone();

    size_t x=0,y=0;
    size_t img_rows=input.rows,img_cols=input.cols;
    for(x=0;x<img_rows;x++){
        //holes.at<uchar>(x,y)==FLOODED or FG, will not be flooded again or flooded.
        //NOTE: Mat(x,y) vs Point(y,x)
        y=0;
        if(holes.at<uchar>(x,y)==0){
            cv::floodFill(holes,cv::Point2i(y,x),cv::Scalar(FLOODED));
        }
        y=img_cols-1;
        if(holes.at<uchar>(x,y)==0){
            cv::floodFill(holes,cv::Point2i(y,x),cv::Scalar(FLOODED));
        }
    }

    for(y=0;y<img_cols;y++){
        //holes.at<uchar>(x,y)==FLOODED or FG, will not be flooded again or flooded.
        x=0;
        if(holes.at<uchar>(x,y)==0){
            cv::floodFill(holes,cv::Point2i(y,x),cv::Scalar(FLOODED));
        }
        x=img_rows-1;
        if(holes.at<uchar>(x,y)==0){
            cv::floodFill(holes,cv::Point2i(y,x),cv::Scalar(FLOODED));
        }
    }

    input=input|(holes==0);
    //    for(int i=0; i<input.rows*input.cols; i++)
    //    {
    //        if(holes.data[i]==0)
    //            input.data[i]=255;
    //    }
}

void yzbx_lbp(const Mat &input, Mat &output,int lbp_operator){
    int radius=3;
    int neighbors=32;
    Mat dst;
    CV_Assert(input.type()==CV_8UC3);
    cvtColor(input, dst, CV_BGR2GRAY);
    GaussianBlur(dst, dst, Size(7,7), 5, 3, BORDER_CONSTANT); // tiny bit of smoothing is always a good idea
    // comment the following lines for original size
    //    resize(input, input, Size(), 0.5, 0.5);
    //    resize(dst,dst,Size(), 0.5, 0.5);
    //    int lbp_operator=1;
    Mat lbpMat;
    switch(lbp_operator) {
    case LBP8UC1:
        //TODO other LBP
        radius=1;
        neighbors=8;
        lbp::OLBP(dst, lbpMat); // use the original operator

        output=Mat::zeros(input.size(),CV_8UC1);
        for(int i=0;i<lbpMat.rows;i++){
            for(int j=0;j<lbpMat.cols;j++){
                output.at<uchar>(i+radius,j+radius)=lbpMat.at<uchar>(i,j);
            }
        }
        break;
    case LBP16UC1:
        radius=2;
        neighbors=16;
        lbp::ELBP(dst, lbpMat, 2, 16); // use the extended operator
        output=Mat::zeros(input.size(),CV_16UC1);
        for(int i=0;i<lbpMat.rows;i++){
            for(int j=0;j<lbpMat.cols;j++){
                output.at<unsigned short>(i+radius,j+radius)=lbpMat.at<unsigned short>(i,j);
            }
        }
        break;
    case LBP32UC1:
        lbp::ELBP(dst, lbpMat, radius, neighbors); // use the extended operator
        output=Mat::zeros(input.size(),CV_32SC1);
        for(int i=0;i<lbpMat.rows;i++){
            for(int j=0;j<lbpMat.cols;j++){
                output.at<unsigned int>(i+radius,j+radius)=lbpMat.at<unsigned int>(i,j);
            }
        }
        break;
    case LBP32FC1:
        //NOTE: this one is count for variance for LBP.
        CV_Assert(false);
        lbp::VARLBP(dst, lbpMat, radius, neighbors);
        break;
    default:
        CV_Assert(false);
    }


    // now to show the patterns a normalization is necessary
    // a simple min-max norm will do the job...
    //    normalize(lbpMat, lbpMat, 0, 255, NORM_MINMAX, CV_8UC1);
    //    imshow("original", input);
    //    imshow("lbp", lbpMat);
}

void yzbx_match(Mat &descriptor1, Mat &descriptor2, vector<DMatch> &matches, int matchType){

    if(!descriptor1.empty ()&&!descriptor2.empty ()){
        CV_Assert(descriptor1.type ()==descriptor2.type ());
        CV_Assert(descriptor1.cols==descriptor2.cols);

        if(matchType==yzbx_match_BF){
            BFMatcher matcher(NORM_L2,true);
            matcher.match (descriptor1,descriptor2,matches);
        }
        else if(matchType==yzbx_match_KNN){
            BFMatcher matcher(NORM_L2);
            const float minRatio=0.6;
            const int k=2;
            vector<vector<DMatch>> knnMatches;
            matcher.knnMatch(descriptor1,descriptor2,knnMatches,k);

            for(size_t i=0;i<knnMatches.size();i++){
                const DMatch &bestMatch=knnMatches[i][0];
                const DMatch &secondMatch=knnMatches[i][1];
                float distanceRatio=bestMatch.distance/secondMatch.distance;
                if(distanceRatio<minRatio){
                    matches.push_back(bestMatch);
                }
            }
        }
    }
}

unsigned yzbx_distance_Vec3b(Vec3b x, Vec3b y, int distance_type, int color_space){
    unsigned distance=0;
    if(color_space==yzbx_colorSpace_BGR||color_space==yzbx_colorSpace_LAB){
        for(int i=0;i<3;i++){
            distance+=yzbx_distance(x[i],y[i],distance_type);
        }
    }
    else if(color_space==yzbx_colorSpace_AB){
        for(int i=1;i<3;i++){
            distance+=yzbx_distance(x[i],y[i],distance_type);
        }
    }
    else if(color_space==yzbx_colorSpace_LABPlus){
        //for shadow remove
        distance=(unsigned)(yzbx_distance(x[0],y[0],distance_type));
        if(x[0]>y[0]+5){
            distance=(int)(distance*0.7);
        }

        for(int i=1;i<3;i++){
            distance+=yzbx_distance(x[i],y[i],distance_type);
        }
    }
    else{
        CV_Assert(false);
    }

    return distance;
}

void icvprCcaByTwoPass(const cv::Mat& _binImg, cv::Mat& _lableImg, std::vector<int> *labelVector)
{
    // connected component analysis (4-component)
    // use two-pass algorithm
    // 1. first pass: label each foreground pixel with a label
    // 2. second pass: visit each labeled pixel and merge neighbor labels
    //
    // foreground pixel: _binImg(x,y) = 1
    // background pixel: _binImg(x,y) = 0


    if (_binImg.empty() ||
            _binImg.type() != CV_8UC1)
    {
        if(_binImg.empty())
            qDebug()<<"empty _binImg";
        else{
            int imgTypeInt=_binImg.type();
            string ImageTypeStr=getImgType(imgTypeInt);
            std::cout<<"not CV_8UC1, but "<<ImageTypeStr<<std::endl;
        }
        return ;
    }

    // 1. first pass

    _lableImg.release() ;
    _binImg.convertTo(_lableImg, CV_32SC1) ;

    int label = 1 ;  // start by 2
    std::vector<int> labelSet ;
    labelSet.push_back(0) ;   // background: 0
    labelSet.push_back(1) ;   // foreground: 1

    int rows = _binImg.rows - 1 ;
    int cols = _binImg.cols - 1 ;
    for (int i = 1; i < rows; i++)
    {
        int* data_preRow = _lableImg.ptr<int>(i-1) ;
        int* data_curRow = _lableImg.ptr<int>(i) ;
        for (int j = 1; j < cols; j++)
        {
            if (data_curRow[j] == 1)
            {
                std::vector<int> neighborLabels ;
                neighborLabels.reserve(2) ;
                int leftPixel = data_curRow[j-1] ;
                int upPixel = data_preRow[j] ;
                if ( leftPixel > 1)
                {
                    neighborLabels.push_back(leftPixel) ;
                }
                if (upPixel > 1)
                {
                    neighborLabels.push_back(upPixel) ;
                }

                if (neighborLabels.empty())
                {
                    labelSet.push_back(++label) ;  // assign to a new label
                    data_curRow[j] = label ;
                    labelSet[label] = label ;
                }
                else
                {
                    std::sort(neighborLabels.begin(), neighborLabels.end()) ;
                    int smallestLabel = neighborLabels[0] ;
                    data_curRow[j] = smallestLabel ;

                    // save equivalence
                    for (size_t k = 1; k < neighborLabels.size(); k++)
                    {
                        int tempLabel = neighborLabels[k] ;
                        int& oldSmallestLabel = labelSet[tempLabel] ;
                        if (oldSmallestLabel > smallestLabel)
                        {
                            labelSet[oldSmallestLabel] = smallestLabel ;
                            oldSmallestLabel = smallestLabel ;
                        }
                        else if (oldSmallestLabel < smallestLabel)
                        {
                            labelSet[smallestLabel] = oldSmallestLabel ;
                        }
                    }
                }
            }
        }
    }

    // update equivalent labels
    // assigned with the smallest label in each equivalent label set
    for (size_t i = 2; i < labelSet.size(); i++)
    {
        int curLabel = labelSet[i] ;
        int preLabel = labelSet[curLabel] ;
        while (preLabel != curLabel)
        {
            curLabel = preLabel ;
            preLabel = labelSet[preLabel] ;
        }
        labelSet[i] = curLabel ;
    }


    // 2. second pass
    for (int i = 0; i < rows; i++)
    {
        int* data = _lableImg.ptr<int>(i) ;
        for (int j = 0; j < cols; j++)
        {
            int& pixelLabel = data[j] ;
            pixelLabel = labelSet[pixelLabel] ;
        }
    }

    // return labelSet
    if(labelVector!=NULL){
        labelSet.swap(*labelVector);
    }
}

void compressMat(Mat &mCV32SC1,Mat &mCV8UC1){
    mCV8UC1.release ();
    mCV8UC1.create (mCV32SC1.size(),CV_8UC1);
    mCV8UC1=Scalar::all (0);

    //push the number in mCV32SC1 to vector by sort
    //then compress it.
    //for example:srcVec 1 100 232 400 ...
    //desVec: 1,2,3,...
    //then get pair(1,1),(100,2),(232,3)...
    //convert the pixel in mCV32SC1 to mCV8UC1 according to this!
    vector<int> srcVec;
    size_t img_cols=mCV32SC1.cols,img_rows=mCV32SC1.rows;
    for(size_t i=0;i<img_rows;i++){
        for(size_t j=0;j<img_cols;j++){
            int v=mCV32SC1.at<int>(i,j);
            //find before sort
            //search after sort
            //            std::vector<int>::iterator it;
            //            it = find (srcVec.begin(), srcVec.end(), v);
            //            if (it == srcVec.end()){
            //                //not find it
            //                srcVec.push_back (v);
            //            }

            //no background
            if(v!=0&&!binary_search(srcVec.begin (),srcVec.end (),v)){
                srcVec.push_back (v);
                sort(srcVec.begin (),srcVec.end ());
            }
        }
    }

    //compress to CV8UC1
    for(size_t i=0;i<img_rows;i++){
        for(size_t j=0;j<img_cols;j++){
            int v=mCV32SC1.at<int>(i,j);
            if(v!=0){
                std::vector<int>::iterator it;
                it=search_n(srcVec.begin (),srcVec.end (),1,v);
                int des=1+(it-srcVec.begin ());
                mCV8UC1.at<uchar>(i,j)=des;
            }
        }
    }
}

cv::Scalar icvprGetRandomColor(){
    uchar r = 255 * (rand()/(1.0 + RAND_MAX));
    uchar g = 255 * (rand()/(1.0 + RAND_MAX));
    uchar b = 255 * (rand()/(1.0 + RAND_MAX));
    return cv::Scalar(b,g,r) ;
}

void icvprLabelColor(const cv::Mat& _labelImg, cv::Mat& _colorLabelImg){
    if (_labelImg.empty() ||
            _labelImg.type() != CV_32SC1)
    {
        return ;
    }

    std::map<int, cv::Scalar> colors ;

    int rows = _labelImg.rows ;
    int cols = _labelImg.cols ;

    _colorLabelImg.release() ;
    _colorLabelImg.create(rows, cols, CV_8UC3) ;
    _colorLabelImg = cv::Scalar::all(0) ;

    for (int i = 0; i < rows; i++)
    {
        const int* data_src = (int*)_labelImg.ptr<int>(i) ;
        uchar* data_dst = _colorLabelImg.ptr<uchar>(i) ;
        for (int j = 0; j < cols; j++)
        {
            int pixelValue = data_src[j] ;
            if (pixelValue > 1)
            {
                if (colors.count(pixelValue) <= 0)
                {
                    colors[pixelValue] = icvprGetRandomColor() ;
                }
                cv::Scalar color = colors[pixelValue] ;
                *data_dst++   = color[0] ;
                *data_dst++ = color[1] ;
                *data_dst++ = color[2] ;
            }
            else
            {
                data_dst++ ;
                data_dst++ ;
                data_dst++ ;
            }
        }
    }
}

void connectedComponentSplit (Mat &nextFGMask, Mat &labelImg8UC1){
    Mat binImage;
    cv::threshold (nextFGMask,binImage,1,1,CV_THRESH_BINARY);
    Mat labelImg32SC1;
    icvprCcaByTwoPass (binImage,labelImg32SC1);
    compressMat (labelImg32SC1,labelImg8UC1);
    CV_Assert(!labelImg8UC1.empty ());
    //    imshow("label image",labelImg8UC1);

    //    Mat colorLabelImg;
    //input mat must be 32SC1 in this label function
    //    icvprLabelColor (labelImg32SC1,colorLabelImg);
    //BUG empty mat!!
    //    imshow("color label img",colorLabelImg);
}

string getImgType(int imgTypeInt)
{
    int numImgTypes = 35; // 7 base types, with five channel options each (none or C1, ..., C4)

    int enum_ints[] =       {CV_8U,  CV_8UC1,  CV_8UC2,  CV_8UC3,  CV_8UC4,
                             CV_8S,  CV_8SC1,  CV_8SC2,  CV_8SC3,  CV_8SC4,
                             CV_16U, CV_16UC1, CV_16UC2, CV_16UC3, CV_16UC4,
                             CV_16S, CV_16SC1, CV_16SC2, CV_16SC3, CV_16SC4,
                             CV_32S, CV_32SC1, CV_32SC2, CV_32SC3, CV_32SC4,
                             CV_32F, CV_32FC1, CV_32FC2, CV_32FC3, CV_32FC4,
                             CV_64F, CV_64FC1, CV_64FC2, CV_64FC3, CV_64FC4};

    string enum_strings[] = {"CV_8U",  "CV_8UC1",  "CV_8UC2",  "CV_8UC3",  "CV_8UC4",
                             "CV_8S",  "CV_8SC1",  "CV_8SC2",  "CV_8SC3",  "CV_8SC4",
                             "CV_16U", "CV_16UC1", "CV_16UC2", "CV_16UC3", "CV_16UC4",
                             "CV_16S", "CV_16SC1", "CV_16SC2", "CV_16SC3", "CV_16SC4",
                             "CV_32S", "CV_32SC1", "CV_32SC2", "CV_32SC3", "CV_32SC4",
                             "CV_32F", "CV_32FC1", "CV_32FC2", "CV_32FC3", "CV_32FC4",
                             "CV_64F", "CV_64FC1", "CV_64FC2", "CV_64FC3", "CV_64FC4"};

    for(int i=0; i<numImgTypes; i++)
    {
        if(imgTypeInt == enum_ints[i]) return enum_strings[i];
    }
    return "unknown image type";
}

void CDNet_GetRoiNum(QString filename, int &a, int &b){
    string fname=filename.toStdString();
    ifstream rfile;
    const char *cname=fname.c_str();
    rfile.open(cname);
    CV_Assert(rfile.is_open());
    char str[100];
    rfile.getline(str,100);
    QString qstr(str);
    qstr.trimmed();
    QStringList numstr=qstr.split(" ",QString::SkipEmptyParts);
    CV_Assert(numstr.size()==2);
    bool ok;
    a=numstr.at(0).toInt(&ok);
    CV_Assert(ok);
    b=numstr.at(1).toInt(&ok);
    CV_Assert(ok);
    rfile.close();
}

void cvt2CV_8U(const cv::Mat input,cv::Mat &output){
    CV_Assert(!input.empty());
    if(input.type()==CV_8UC3||input.type()==CV_8UC1||input.type()==CV_8U){
        output=input.clone();
    }
    else if(input.type()==CV_32FC1){
        output.release();
        output.create(input.size(),CV_8UC3);
        int img_rows=input.rows;
        int img_cols=input.cols;
        for(int i=0;i<img_rows;i++){
            for(int j=0;j<img_cols;j++){
                float f=input.at<float>(i,j);
                int iv=(int)f;
                int R=iv/(256*256);
                iv-=(R*256*256);
                int G=iv/256;
                int B=iv%256;

                Vec3b v(B,G,R);
                output.at<Vec3b>(i,j)=v;
            }
        }
    }
    else if(input.type()==CV_64FC1){
        output.release();
        output.create(input.size(),CV_8UC3);
        int img_rows=input.rows;
        int img_cols=input.cols;
        for(int i=0;i<img_rows;i++){
            for(int j=0;j<img_cols;j++){
                double f=input.at<double>(i,j);
                int iv=(int)f;
                int R=iv/(256*256);
                iv-=(R*256*256);
                int G=iv/256;
                int B=iv%256;

                Vec3b v(B,G,R);
                output.at<Vec3b>(i,j)=v;
            }
        }
    }
    else if(input.channels()==3){
        input.convertTo(output,CV_8UC3);
    }
    else if(input.channels()==1){
        input.convertTo(output,CV_8UC1);
    }
    else{
        cout<<"undefined convert"<<endl;
        CV_Assert(false);
    }

}

size_t getDirections(float dx, float dy){
    if(dx<0.1&&dy<0.1) return 8;

    if(dx>=0){
        if(dy>=0){
            if(dx-dy>=0){
                //                directions8[0]++;
                return 0;
            }
            else{
                //                directions8[1]++;
                return 1;
            }
        }
        else{
            if(dx+dy>=0){
                //                directions8[7]++;
                return 7;
            }
            else{
                //                directions8[6]++;
                return 6;
            }
        }
    }
    else{
        if(dy>=0){
            if(dx+dy>=0){
                //                directions8[2]++;
                return 2;
            }
            else{
                //                directions8[3]++;
                return 3;
            }
        }
        else{
            if(dx-dy>=0){
                //                directions8[5]++;
                return 5;
            }
            else{
                //                directions8[4]++;
                return 4;
            }
        }
    }
}

void filterBinaryImageByArea(Mat &binImage,double minarea){
    //the elements hierarchy[i][0] , hiearchy[i][1] , hiearchy[i][2] , and hiearchy[i][3] are set to 0-based indices in contours of the next and previous contours at the same hierarchical level, the first child contour and the parent contour, respectively. If for the contour i there are no next, previous, parent, or nested contours, the corresponding elements of hierarchy[i] will be negative.
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

//    Mat temp;
//    int niters = 3;
//    dilate(binImage, temp, Mat(), Point(-1,-1), niters);
//    erode(temp, temp, Mat(), Point(-1,-1), niters*2);
//    dilate(temp, temp, Mat(), Point(-1,-1), niters);

    //CV_RETR_EXTERNAL retrieves only the extreme outer contours. It sets hierarchy[i][2]=hierarchy[i][3]=-1 for all the contours.
    //CV_CHAIN_APPROX_NONE stores absolutely all the contour points. That is, any 2 subsequent points (x1,y1) and (x2,y2) of the contour will be either horizontal, vertical or diagonal neighbors, that is, max(abs(x1-x2),abs(y2-y1))==1
    findContours( binImage, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE );

    Mat dst = Mat::zeros(binImage.size(), CV_8UC3);

    if( contours.size() == 0 )
        return;

    // iterate through all the top-level contours,
    // draw each connected component with its own random color
    int idx = 0;
    Scalar color( 0, 0, 255 );
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
        const vector<Point>& c = contours[idx];
        //the area is computed using the Green formula. Thus, the returned area and the number of non-zero pixels
        double area = fabs(contourArea(Mat(c)));
        if( area > minarea )
        {
           drawContours( dst, contours, idx, color, CV_FILLED, 8, hierarchy );
        }
    }

    Mat gray;
    cvtColor(dst,gray,CV_BGR2GRAY);
    binImage=gray>0;
}

void yzbx_generateErrorMap(const Mat& truthGround,const Mat & FGMask,Mat &errorMat){
    CV_Assert(!truthGround.empty());
    CV_Assert(!FGMask.empty());
    CV_Assert(truthGround.type()==FGMask.type());
    CV_Assert(truthGround.type()==CV_8UC1);
    CV_Assert(truthGround.rows==FGMask.rows);
    CV_Assert(truthGround.cols==FGMask.cols);
    errorMat.release();
    errorMat.create(truthGround.size(),CV_8UC3);

    Vec3b TN(0,0,0);
    Vec3b TP(200,200,200);
    Vec3b FP_shadow(255,0,0);
    Vec3b FP_other(0,255,0);
    Vec3b FN(0,0,255);
    Vec3b OUTROI(85,85,85);

//    Mat grayErrorMat(truthGround.size(),CV_8UC1);
    size_t img_rows=truthGround.rows;
    size_t img_cols=truthGround.cols;
    for(size_t i=0;i<img_rows;i++){
        for(size_t j=0;j<img_cols;j++){
            uchar tg=truthGround.at<uchar>(i,j);
            uchar fg=FGMask.at<uchar>(i,j);
            if(tg==fg||tg==CDNet_GT_Unknown){
                if(tg==CDNet_GT_Moving) errorMat.at<Vec3b>(i,j)=TP;
                else errorMat.at<Vec3b>(i,j)=TN;
            }
            else if(tg==CDNet_GT_OutRoi){
                errorMat.at<Vec3b>(i,j)=OUTROI;
            }
            else if(tg==CDNet_GT_Shadow){
                if(fg==CDNet_GT_BG){
                    errorMat.at<Vec3b>(i,j)=TN;
                }
                else{
                     errorMat.at<Vec3b>(i,j)=FP_shadow;
                }
            }
            else if(tg==CDNet_GT_BG){
                 errorMat.at<Vec3b>(i,j)=FP_other;
            }
            else if(tg==CDNet_GT_Moving){
                 errorMat.at<Vec3b>(i,j)=FN;
            }
            else{
                CV_Assert(false);
            }
        }
    }
    dilate(errorMat,errorMat,Mat(),Point(-1,-1),3);
}

void showMat8U(const string windowName,const cv::Mat input){
    Mat mat8U;
    cvt2CV_8U(input,mat8U);
    imshow(windowName,mat8U);
}

void doubleThresholdConnect(const Mat &highThresholdMask,const Mat &lowThresholdMask,Mat &outputMask){
    outputMask=Mat::zeros(highThresholdMask.rows+2,highThresholdMask.cols+2,CV_8UC1);
    int img_rows=highThresholdMask.rows,img_cols=highThresholdMask.cols;
    int count=0;
    Rect rect;
    for(int i=0;i<img_rows;i++){
        for(int j=0;j<img_cols;j++){
            Point p(j,i);
            if(highThresholdMask.at<uchar>(i,j)>0&&outputMask.at<uchar>(i+1,j+1)==0){
                count++;
                Mat mask=Mat::zeros(img_rows+2,img_cols+2,CV_8UC1);
                floodFill(lowThresholdMask,mask,p,Scalar(255),&rect,Scalar(1),Scalar(1),8|FLOODFILL_FIXED_RANGE);
                CV_Assert(mask.at<uchar>(i+1,j+1)!=0);
                outputMask|=mask;

//                imshow("lowThresholdMask",lowThresholdMask);
//                imshow("highThresholdMask",highThresholdMask);
//                imshow("tmpMask",mask>0);
//                imshow("outputMask",outputMask>0);

//                int key=waitKey(30);
//                if(key=='s'){
//                    waitKey(0);
//                }
            }
        }
    }

    outputMask=outputMask>0;
    qDebug()<<"count="<<count;
}

namespace  yzbxlib {
track_t getRectGap(Rect_t ra,Rect_t rb){
    track_t width=(ra.width+rb.width)*0.5f;
    track_t height=(ra.height+rb.height)*0.5f;
    Point_t p=(ra.tl()+ra.br()-rb.tl()-rb.br())*0.5f;
    if(p.x<0) p.x=-p.x;
    if(p.y<0) p.y=-p.y;

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

bool isPointInRect(cv::Point2f p, Rect_t rect)
{
    if(p.x>rect.x&&p.x>rect.y&&p.x<rect.x+rect.width&&p.y<rect.y+rect.height){
        return true;
    }
    else{
        return false;
    }
}


bool isRectAInRectB(Rect_t A,Rect_t B){
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

QString getAbsoluteFilePath(QString currentPathOrFile, QString fileName)
{
    QFileInfo pinfo(currentPathOrFile);
    QString currentPath=pinfo.absolutePath();

    QFileInfo info;
    info.setFile(QDir(currentPath),fileName);
    if(info.isAbsolute()){
        return info.absoluteFilePath();
    }
    else{
        if(info.isDir()){
            QDir dir(currentPath);
            dir.cd(fileName);
            return dir.absolutePath();
        }
        else if(info.isFile()){
            QDir dir(currentPath);
            return dir.absoluteFilePath(fileName);
        }
        else{
            qDebug()<<"error in absoluteFilePath: "<<currentPathOrFile<<fileName;
            return "error";
        }
    }
}

bool isSameImage(const Mat &A_8U, const Mat &B_8U)
{
    cv::Mat C;
    cv::absdiff(A_8U,B_8U,C);
    cv::Scalar s=cv::sum(C);
    if(C.channels()==3){
        if(s[0]==0&&s[1]==0&&s[2]==0){
            return true;
        }
    }
    else{
        if(s[0]==0){
            return true;
        }
    }

    return false;
}

void showImageInWindow(string windowName, const Mat &img)
{
    cv::namedWindow(windowName,WINDOW_NORMAL);
    cv::imshow(windowName,img);
}

void moveWindows(std::vector<string> &windowNames, int colNum)
{
    int n=windowNames.size();
    int width=100,height=100;
    for(int i=0;i<n;i++){
        int row=n/colNum;
        int col=n%colNum;
        int y=row*height;
        int x=col*width;
        cv::namedWindow(windowNames[i],cv::WINDOW_NORMAL);
        cv::moveWindow(windowNames[i],x,y);
    }
}

void dumpVector(std::vector<int> &v)
{
    for(auto it=v.begin();it!=v.end();it++){
//        string numstr=boost::lexical_cast<std::string>(*it);
        std::cout<<*it<<", ";
    }
    std::cout<<std::endl;
}

void dumpVector(std::vector<float> &v)
{
    for(auto it=v.begin();it!=v.end();it++){
//        string numstr=boost::lexical_cast<std::string>(*it);
        std::cout<<*it<<", ";
    }
    std::cout<<std::endl;
}

void drawMatch(Mat &img, Point_t p1, Point_t p2)
{
    int width=img.cols/2;
    Point_t newP2=p2;
    newP2.x +=width;
    cv::line(img,p1,newP2,cv::Scalar(0,0,255),3);
}

}
