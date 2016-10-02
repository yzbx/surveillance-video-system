#include "benchmark.h"
#include "ui_benchmark.h"

Benchmark::Benchmark(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::Benchmark)
{
    ui->setupUi(this);
    QString defaultConfigFile="/home/yzbx/config/surveillance-video-system.ini";
    loadConfig(defaultConfigFile);
}

Benchmark::~Benchmark()
{
    delete ui;
}

void Benchmark::on_pushButton_analyse_clicked()
{
    QString video=ui->comboBox_video->currentText();
    if(video.compare("all")==0){
        for(int i=0;i<globalVideosList.size();i++){
            QString videoFile=globalVideosList[i];
            processOne(videoFile);
        }
    }
    else{
        QString videoFile=video;
        processOne(videoFile);
    }
}

void Benchmark::processOne(QString videoFile){
    QString outputFileName=videoFile;

    videoFile=globalVideoHome+"/"+videoFile;
    QFileInfo info(videoFile);
    if(info.isDir()){
        outputFileName.replace("/","_");
        outputFileName+=".txt";
    }
    else if(info.isFile()){
        outputFileName.replace("/","_");
        outputFileName.replace(info.suffix(),"txt");
    }
    qDebug()<<"output filename: "<<outputFileName;
    tracker.init(outputFileName);

    globalVideoFile=videoFile;

    while(1){
        cv::Mat img_fg,img_input;
        frameInput.getNextFrame(videoFile,img_input);
        if(img_input.empty()||globalStop){
            break;
        }

        if(img_input.channels()==3){
            cv::cvtColor(img_input,img_fg,CV_BGR2GRAY);
        }
        img_fg=(img_fg==255);

        //2. generate feature and tracking.
        tracker.tracking(img_input,img_fg);
        cv::waitKey(30);

        //3. process event of stop
        QApplication::processEvents();
    }
    globalStop=false;
}

void Benchmark::loadConfig(QString configFilePath){
    boost::property_tree::ini_parser::read_ini(configFilePath.toStdString(),globalPt);
    QString VideoHome=QString::fromStdString(globalPt.get<std::string>("Benchmark.VideoHome"));
    VideoHome=yzbxlib::getAbsoluteFilePath(configFilePath,VideoHome);

    QString VideoTxt=QString::fromStdString(globalPt.get<std::string>("Benchmark.VideoTxt"));
    VideoTxt=yzbxlib::getAbsoluteFilePath(configFilePath,VideoTxt);

    QString filedata;
    QFile file;
    file.setFileName(VideoTxt);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        qDebug()<<"cannot open file "<<VideoTxt;
        exit(-1);
    } else {
        QTextStream in(&file);
        filedata=in.readAll();
        globalVideosList=filedata.split("\n",QString::SkipEmptyParts);
        globalVideoHome=VideoHome;
        file.close();
    }

    ui->lineEdit_configFile->setText(configFilePath);
    ui->comboBox_video->clear();
    ui->comboBox_video->addItems(globalVideosList);
    ui->comboBox_video->addItem("all");
}

void Benchmark::on_pushButton_stop_clicked()
{
    if(ui->pushButton_stop->text()=="stop"){
        ui->pushButton_stop->setText("continue");
        globalStop=true;
    }
    else{
        ui->pushButton_stop->setText("stop");
        QString videoFile=globalVideoFile;
        while(1){
            cv::Mat img_fg,img_input;
            frameInput.getNextFrame(videoFile,img_input);
            if(img_input.empty()||globalStop){
                break;
            }

            if(img_input.channels()==3){
                cv::cvtColor(img_input,img_fg,CV_BGR2GRAY);
            }
            img_fg=(img_fg==255);

            //2. generate feature and tracking.
            tracker.tracking(img_input,img_fg);
            cv::waitKey(30);

            //3. process event of stop
            QApplication::processEvents();
        }
        globalStop=false;
    }
}
