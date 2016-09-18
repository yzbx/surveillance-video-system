#include "BlobDetector.h"

BlobDetector::BlobDetector()
{
    // Change thresholds
    //pixel < minThreshold := 0
    //pixel > maxThreshold := 0
    //detector do with dark area by default
    params.minThreshold = 100;
    params.maxThreshold = 200;
    params.thresholdStep = 255;
    params.minRepeatability = 0;

    //whether to set two blob as one?
    params.minDistBetweenBlobs = 10;

    params.filterByColor = true;
    params.blobColor=255;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 100;
    params.maxArea = std::numeric_limits<float>::max();

    params.filterByCircularity = false;
    params.minCircularity = 0.8f;
    params.maxCircularity = std::numeric_limits<float>::max();

    params.filterByInertia = false;
    //minInertiaRatio = 0.6;
    params.minInertiaRatio = 0.1f;
    params.maxInertiaRatio = std::numeric_limits<float>::max();

    params.filterByConvexity = false;
    //minConvexity = 0.8;
    params.minConvexity = 0.95f;
    params.maxConvexity = std::numeric_limits<float>::max();


    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    //    Mat im_with_keypoints;
    //    drawKeypoints( im, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
}

BlobDetector::~BlobDetector()
{

}

void BlobDetector::process(InputArray _image, InputArray _binaryImage, std::vector<trackingObjectFeature> &features)
{
    getBlobFeature(_image,_binaryImage,features);
}

void BlobDetector::getBlobFeature(InputArray _image, InputArray _binaryImage, std::vector<trackingObjectFeature> &features)
{
    if(!features.empty()) features.clear();
    Mat image = _image.getMat(), binaryImage0 = _binaryImage.getMat();

    //check and init blobDetector
    if(rows!=binaryImage0.rows||cols!=binaryImage0.cols){
        rows=binaryImage0.rows;
        cols=binaryImage0.cols;
        maxObjectSize=0;
        params.minArea=rows*cols/5000.0;
    }

    //postprocessing for binaryImage
    Mat binaryImage;
    if(binaryImage0.channels()==3){
        cv::cvtColor(binaryImage0,binaryImage,CV_BGR2GRAY);
    }
    else{
        binaryImage=binaryImage0.clone();
    }

    std::vector < std::vector<Point> > contours;
    Mat tmpBinaryImage = binaryImage.clone();
    findContours(tmpBinaryImage, contours, CV_RETR_EXTERNAL, CHAIN_APPROX_NONE);

    vector<Point2f> KLTPoints;
    if(UseKLT){
        TermCriteria termcrit(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03);
        Size subPixWinSize(10,10), winSize(31,31);
        const int MAX_COUNT = 500;
        cv::Mat gray;
        if(image.channels()==3)
            cv::cvtColor(image,gray,CV_BGR2GRAY);
        else
            gray=image.clone();

        //    goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
        //    cornerSubPix(gray, points[1], subPixWinSize, Size(-1,-1), termcrit);
        goodFeaturesToTrack(gray, KLTPoints, MAX_COUNT, 0.01, 10, Mat(), 3, 0, 0.04);
        cornerSubPix(gray, KLTPoints, subPixWinSize, Size(-1,-1), termcrit);
    }
    for (size_t contourIdx = 0; contourIdx < contours.size(); contourIdx++)
    {
        trackingObjectFeature of;
        Moments moms = moments(Mat(contours[contourIdx]));

        double area = moms.m00;
        of.size=area;
//BUG bad for bgs error.
//        if(area>maxObjectSize){
//            maxObjectSize=area;
//            params.minArea=std::max(params.minArea,(float)maxObjectSize/20);
//        }
        if (params.filterByArea&&(area < params.minArea || area >= params.maxArea)){
            continue;
        }

        double perimeter = arcLength(Mat(contours[contourIdx]), true);
        double ratio = 4 * CV_PI * area / (perimeter * perimeter);
        of.Circularity=ratio;
        if (params.filterByCircularity&&(ratio < params.minCircularity || ratio >= params.maxCircularity))
        {
            continue;
        }

        double denominator = std::sqrt(std::pow(2 * moms.mu11, 2) + std::pow(moms.mu20 - moms.mu02, 2));
        const double eps = 1e-2;
        if (denominator > eps)
        {
            double cosmin = (moms.mu20 - moms.mu02) / denominator;
            double sinmin = 2 * moms.mu11 / denominator;
            double cosmax = -cosmin;
            double sinmax = -sinmin;

            double imin = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmin - moms.mu11 * sinmin;
            double imax = 0.5 * (moms.mu20 + moms.mu02) - 0.5 * (moms.mu20 - moms.mu02) * cosmax - moms.mu11 * sinmax;
            ratio = imin / imax;
        }
        else
        {
            ratio = 1;
        }
        of.Inertia=ratio;

        if (params.filterByInertia)
        {
            if (ratio < params.minInertiaRatio || ratio >= params.maxInertiaRatio)
                continue;
        }

        std::vector < Point > hull;
        convexHull(Mat(contours[contourIdx]), hull);
        area = contourArea(Mat(contours[contourIdx]));
        double hullArea = contourArea(Mat(hull));
        ratio = area / hullArea;
        of.Convexity=ratio;

        if (params.filterByConvexity)
        {
            if (ratio < params.minConvexity || ratio >= params.maxConvexity)
                continue;
        }

        if(moms.m00 == 0.0)
            continue;


        of.pos = Point_t(moms.m10 / moms.m00, moms.m01 / moms.m00);

        //compute blob radius
        {
            std::vector<double> dists;
            for (size_t pointIdx = 0; pointIdx < contours[contourIdx].size(); pointIdx++)
            {
                Point_t pt = contours[contourIdx][pointIdx];
                dists.push_back(norm(of.pos - pt));
            }
            std::sort(dists.begin(), dists.end());
            of.radius = (dists[(dists.size() - 1) / 2] + dists[dists.size() / 2]) / 2.;
        }

        of.rect=cv::boundingRect(contours[contourIdx]);
        Point_t tl=of.rect.tl();
        Point_t br=of.rect.br();
        if(tl.x<5||tl.y<5||br.x+5>binaryImage.cols||br.y+5>binaryImage.rows){
            of.onBoundary=true;
        }
        else{
            of.onBoundary=false;
        }

        cv::Mat mask_i(image.size(),CV_8UC1);
        mask_i=Scalar::all(0);
        cv::drawContours(mask_i,contours,contourIdx,cv::Scalar::all(255),CV_FILLED);

        of.mask=mask_i;
        if(UseLIF){

            ObjectLocalFeatureMatch match;
            match.getLIFMat(of.LIFMat,of.LIFPos,image,mask_i);
            for(int i=0;i<of.LIFPos.size();i++){
                Point_t p=of.LIFPos[i];
                int x=(int)p.x;
                int y=(int)p.y;
                Point3i color;
                color.x=image.at<Vec3b>(y,x)[0];
                color.y=image.at<Vec3b>(y,x)[1];
                color.z=image.at<Vec3b>(y,x)[2];
                of.LIFColor.push_back(color);
            }
        }

        if(UseKLT){
            for(int i=0;i<KLTPoints.size();i++){
                Point p=KLTPoints[i];
                if(mask_i.at<uchar>(p.y,p.x)==255){
                    of.KLTPos.push_back(p);
                }
            }
        }


//        showBlobFeature(image,mask_i,of);

        //NOTE, avoid auto LIFMat not empty.
//        if(of.LIFPos.empty()) of.LIFMat.release();
//        if(of.LIFMat.empty()){
//            qDebug()<<"empty";
//        }

        features.push_back(of);
    }
}

void BlobDetector::showBlobFeature(const Mat &input, const Mat &mask, const trackingObjectFeature &of)
{
    Mat showIn=input.clone();
    Mat showFg=mask.clone();

    namedWindow("showIn",WINDOW_NORMAL);
    namedWindow("showFg",WINDOW_NORMAL);

    const vector<Point_t> &ps=of.LIFPos;
    rectangle(showIn,of.rect,Scalar(0,0,255),3);
    rectangle(showFg,of.rect,Scalar(255),3);
    for(int i=0;i<ps.size();i++){
        circle(showIn,ps[i],3,Scalar(255,0,0),3);
    }

    imshow("showIn",showIn);
    imshow("showFg",showFg);
    waitKey(0);
}
