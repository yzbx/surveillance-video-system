#include "cvhogdetector.h"

CVHOGDetector::CVHOGDetector(int argc, char *argv[])
{
    main(argc,argv);
}


// static void help()
// {
//     printf(
//             "\nDemonstrate the use of the HoG descriptor using\n"
//             "  HOGDescriptor::hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());\n"
//             "Usage:\n"
//             "./peopledetect (<image_filename> | <image_list>.txt)\n\n");
// }

int CVHOGDetector::main(int argc, char** argv)
{
    string filename;
    if( argc == 1 )
    {
        printf("Usage: peopledetect video_filename\n");
        filename="/mnt/hgfs/I/Win7/dataset/大王卡口全景东向西_大王卡口全景东向西/大王卡口全景东向西_大王卡口全景东向西_20150318090325.mp4";
    }
    else {
        filename=argv[1];
    }
    VideoCapture cap(filename);
    if(!cap.isOpened()){
        std::cout<<"cannot open video file "<<filename<<endl;
    }

    HOGDescriptor hog;
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    namedWindow("people detector", 1);

    for(;;)
    {
        Mat img;
        cap>>img;

        if(!img.data){
            std::cout<<"empty image"<<endl;
        }

        vector<Rect> found, found_filtered;
        double t = (double)getTickCount();
        // run the detector with default parameters. to get a higher hit-rate
        // (and more false alarms, respectively), decrease the hitThreshold and
        // groupThreshold (set groupThreshold to 0 to turn off the grouping completely).
        hog.detectMultiScale(img, found, 0, Size(8,8), Size(32,32), 1.05, 2);
        t = (double)getTickCount() - t;
        printf("tdetection time = %gms\n", t*1000./cv::getTickFrequency());
        size_t i, j;
        for( i = 0; i < found.size(); i++ )
        {
            Rect r = found[i];
            for( j = 0; j < found.size(); j++ )
                if( j != i && (r & found[j]) == r)
                    break;
            if( j == found.size() )
                found_filtered.push_back(r);
        }
        for( i = 0; i < found_filtered.size(); i++ )
        {
            Rect r = found_filtered[i];
            // the HOG detector returns slightly larger rectangles than the real objects.
            // so we slightly shrink the rectangles to get a nicer output.
            r.x += cvRound(r.width*0.1);
            r.width = cvRound(r.width*0.8);
            r.y += cvRound(r.height*0.07);
            r.height = cvRound(r.height*0.8);
            rectangle(img, r.tl(), r.br(), cv::Scalar(0,255,0), 3);
        }
        imshow("people detector", img);
        int c = waitKey(30) & 255;
        if( c == 'q' || c == 'Q')
            break;
    }

    return 0;
}
