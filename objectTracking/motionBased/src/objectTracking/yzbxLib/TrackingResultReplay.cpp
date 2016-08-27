#include "TrackingResultReplay.h"

TrackingResultReplay::TrackingResultReplay()
{

}


void TrackingResultReplay::process(QString videoFilePath, QString recordFilePath)
{

    std::vector<object> objects;

    //     For example, the line “0 0 92 12 22 53 ” means:
    //            Frame Number: 0, must start from 0, not 1, and the line should order by frame number
    //                          but the minimal frame number in file can be any number >=0
    //            Label of this object: 0, the minimal object label in file can be any number >=0
    //            Upper left point (x, y) of the bounding box: (92, 12)
    //            (Width, Height) of the bounding box: (22, 53)

    QFile file(recordFilePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)){
        qDebug()<<"cannot open file "<<recordFilePath;
        return;
    }
    FrameInput frameInput;
    int frameNum=0;
    cv::Mat img_input;

    QTextStream in(&file);
    std::set<int> idSet={};
    while (!in.atEnd()) {
        QString line = in.readLine();
        QStringList strlist=line.split(",",QString::SkipEmptyParts);
        float numbers[6];
        for(int i=0;i<strlist.size();i++){
            QString numstr=strlist.at(i);
            bool okay;
            numbers[i]=numstr.toFloat(&okay);
            if(!okay){
                qDebug()<<"error convert to float "<<numstr;
                exit(-1);
            }
        }

        int readToFrameNum=boost::lexical_cast<int>(numbers[0]);
        while(readToFrameNum>=frameNum){
            frameInput.getNextFrame(videoFilePath,img_input);
            frameNum++;
        }
        //NOTE must show here, otherwise there will have some error!
        cv::imshow("img_input",img_input);
//        cv::waitKey(30);
        Rect_t r(numbers[2],numbers[3],numbers[4],numbers[5]);

        //NOTE objectID should be continuously
        int readToObjectID=boost::lexical_cast<int>(numbers[1]);
        if(idSet.find(readToObjectID)==idSet.end()){
            object ob(readToFrameNum,r,readToObjectID);
            objects.push_back(ob);
            idSet.insert(readToObjectID);
        }
        else{
            for(int i=0;i<objects.size();i++){
                if(readToObjectID==objects[i].ID){
                    objects[i].add(readToFrameNum,r);
                    break;
                }
            }
        }

        removeOldObjects(objects,readToFrameNum,idSet);
        replay(img_input,objects,readToFrameNum);
    }

}

void TrackingResultReplay::removeOldObjects(std::vector<object> &objects,int readToFrameNum,std::set<int> &idSet){
    for(int i=objects.size()-1;i>=0;i--){
        if(readToFrameNum>objects[i].frameNum+MaxSkipFrame){
            idSet.erase(objects[i].ID);
            objects.erase(objects.begin()+i);
        }
    }
}

void TrackingResultReplay::replay(const cv::Mat &img_input,std::vector<object> &objects,int readToFrameNum){
    cv::Scalar Colors[] = { cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255), cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255), cv::Scalar(255, 0, 255), cv::Scalar(255, 127, 255), cv::Scalar(127, 0, 255), cv::Scalar(127, 0, 127) };

    cv::Mat img_show;

    if(img_input.channels()==1){
        cvtColor(img_input,img_show,CV_GRAY2BGR);
    }
    else{
        img_show=img_input.clone();
    }
    if(img_input.channels()!=globalChannel){
//        cv::destroyAllWindows();
        globalChannel=img_input.channels();
    }

    for(int i=0;i<objects.size();i++){
        if(readToFrameNum==objects[i].frameNum){
            //void rectangle(Mat& img, Rect rec, const Scalar& color, int thickness=1, int lineType=LINE_8, int shift=0 )
            Rect_t r=objects[i].trace.back();
//            std::cout<<"r=["<<r.x<<","<<r.y<<","<<r.width<<","<<r.height<<"]"<<std::endl;
            cv::rectangle(img_show,r,cv::Scalar(0,0,255),3);
            string text=boost::lexical_cast<std::string>(objects[i].ID);
            cv::putText(img_show, text, r.tl(), cv::FONT_HERSHEY_COMPLEX, 1.0,
                        cv::Scalar(0,0,255), 2, 8);
        }

        for (uint j = 0; j < objects[i].trace.size() - 1; j++)
        {
            Rect_t r1=objects[i].trace[j],r2=objects[i].trace[j+1];
            cv::Point c1(r1.x+r1.width/2,r1.y+r1.height/2),c2(r2.x+r2.width/2,r2.y+r2.height/2);
            cv::line(img_show, c1,c2, Colors[objects[i].ID % 9], 2, CV_AA);
        }
    }


    cv::imshow("img_new_input",img_input);
    cv::imshow(winname,img_show);
    cv::waitKey(30);
}