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
        QStringList strlist=line.split(" ",QString::SkipEmptyParts);
        float numbers[6];
        for(int i=0;i<strlist.size();i++){
            QString numstr=strlist.at(i);
            bool okay;
            numbers[i]=numstr.toFloat(&okay);
            if(!okay){
                qDebug()<<"error convert to int";
                exit(-1);
            }
        }

        int readToFrameNum=boost::lexical_cast<int>(numbers[0]);
        while(readToFrameNum>=frameNum){
            frameInput.getNextFrame(videoFilePath,img_input);
            frameNum++;
        }

        cv::Rect r(numbers[3],numbers[4],numbers[5],numbers[6]);

        //NOTE objectID should be continuously
        int readToObjectID=boost::lexical_cast<int>(numbers[1]);
        if(idSet.find(readToObjectID)!=idSet.end()){
            object ob(readToFrameNum,r,readToObjectID);
            objects.push_back(ob);
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

void TrackingResultReplay::replay(cv::Mat &img_input,std::vector<object> &objects,int readToFrameNum){
    cv::Mat img_show=img_input.clone();
    for(int i=0;i<objects.size();i++){
        if(readToFrameNum==objects[i].frameNum){
            //void rectangle(Mat& img, Rect rec, const Scalar& color, int thickness=1, int lineType=LINE_8, int shift=0 )
            cv::rectangle(img_show,objects[i].trace.back(),cv::Scalar(0,0,255),3);
            string text=boost::lexical_cast<std::string>(objects[i].ID);
            cv::Rect r=objects[i].trace.back();
            cv::putText(img_show, text, r.tl(), cv::FONT_HERSHEY_COMPLEX, 0.5,
                        cv::Scalar(0,0,255), 2, 8);
        }
    }

    cv::imshow(winname,img_show);
    cv::waitKey(30);
}
