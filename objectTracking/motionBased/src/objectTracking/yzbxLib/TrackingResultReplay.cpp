#include "TrackingResultReplay.h"

TrackingResultReplay::TrackingResultReplay(QString datasetName)
{
    globalInit=false;
    datasetNameList<<"Multi-CameraTracking";
    if(!datasetName.isEmpty()){
        Init(datasetName);
    }
}
void TrackingResultReplay::Init(QString datasetName){
    if(datasetNameList.contains(datasetName,Qt::CaseInsensitive)){
        globalInit=true;
    }
    this->datasetName=datasetName;
}

void TrackingResultReplay::process(QString videoFilePath, QString recordFilePath)
{
    if(!globalInit){
        qDebug()<<"please init first!";
        return;
    }

    std::vector<object> objects;
    if(datasetName.compare("Multi-CameraTracking",Qt::CaseInsensitive)==0){
//     For example, the line “1 0 0 92 12 22 53 ” means:
//            Camera Number: 1
//            Frame Number: 0
//            Label of this object: 0
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
        int objectID=0;

        QTextStream in(&file);
        while (!in.atEnd()) {
            QString line = in.readLine();
            QStringList strlist=line.split(" ",QString::SkipEmptyParts);
            int numbers[7];
            for(int i=0;i<strlist.size();i++){
                QString numstr=strlist.at(i);
                bool okay;
                numbers[i]=numstr.toInt(&okay);
                if(!okay){
                    qDebug()<<"error convert to int";
                    exit(-1);
                }
            }

            int readToFrameNum=numbers[1];
            while(readToFrameNum>=frameNum){
                frameInput.getNextFrame(videoFilePath,img_input);
                frameNum++;
            }

            cv::Rect r(numbers[3],numbers[4],numbers[5],numbers[6]);

            //NOTE objectID should be continuously
            if(numbers[2]==objectID){
                object ob(readToFrameNum,r,objectID++);
                objects.push_back(ob);
            }
            else if(numbers[2]<objectID){
                for(int i=0;i<objects.size();i++){
                    if(numbers[2]==objects[i].ID){
                        objects[i].add(readToFrameNum,r);
                        break;
                    }
                }
            }
            else{
                qDebug()<<"unexpected thing.....................";
            }

            removeOldObjects(objects,readToFrameNum);
            replay(img_input,objects,readToFrameNum);
        }

    }
}

void TrackingResultReplay::removeOldObjects(std::vector<object> &objects,int readToFrameNum){
    for(int i=objects.size()-1;i>=0;i--){
        if(readToFrameNum>objects[i].frameNum+MaxSkipFrame){
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
        }
    }

    cv::imshow(winname,img_show);
    cv::waitKey(30);
}
