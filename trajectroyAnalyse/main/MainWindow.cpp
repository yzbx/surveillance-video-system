#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QString configFile="/home/yzbx/config/surveillance-video-system.ini";
    loadconfig(configFile);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::loadconfig(QString configFile)
{
    QFileInfo info(configFile);
    if(info.isFile()){
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini(configFile.toStdString(),pt);
        QString VideoTxt=QString::fromStdString(pt.get<std::string>("TrajectoryAnalyse.VideoTxt"));
        VideoTxt=yzbxlib::getAbsoluteFilePath(configFile,VideoTxt);
        globalImageDatabaseDir=QString::fromStdString(pt.get<std::string>("TrajectoryAnalyse.ImageDatabaseDir"));
        globalImageDatabaseDir=yzbxlib::getAbsoluteFilePath(configFile,globalImageDatabaseDir);
        QString CSVTxt=QString::fromStdString(pt.get<std::string>("TrajectoryAnalyse.CSVTxt"));
        CSVTxt=yzbxlib::getAbsoluteFilePath(configFile,CSVTxt);

        QString filedata;
        QFile file;
        file.setFileName(VideoTxt);
        if (!file.open(QFile::ReadOnly | QFile::Text)) {
            qDebug()<<"cannot open file "<<VideoTxt;
            assert(false);
        } else {
            QTextStream in(&file);
            filedata=in.readAll();
            globalVideoList=filedata.split("\n",QString::SkipEmptyParts);
            globalVideoHome=QString::fromStdString(pt.get<std::string>("TrajectoryAnalyse.VideoHome"));
            globalVideoHome=yzbxlib::getAbsoluteFilePath(configFile,globalVideoHome);
            file.close();
        }

        file.setFileName(CSVTxt);
        if (!file.open(QFile::ReadOnly | QFile::Text)) {
            qDebug()<<"cannot open file "<<CSVTxt;
            assert(false);
        } else {
            QTextStream in(&file);
            filedata=in.readAll();
            globalCSVList=filedata.split("\n",QString::SkipEmptyParts);
            globalCSVHome=QString::fromStdString(pt.get<std::string>("TrajectoryAnalyse.CSVHome"));
            globalCSVHome=yzbxlib::getAbsoluteFilePath(configFile,globalCSVHome);
            file.close();
        }

        ui->comboBox_trajectory->addItems(globalCSVList);
        ui->comboBox_video->addItems(globalVideoList);
    }
}

void MainWindow::on_pushButton_loadTrajectory_clicked()
{
    QString videoFile=ui->comboBox_video->currentText();
    QString csvFile=ui->comboBox_trajectory->currentText();
    QFileInfo info(csvFile);
    QString baseName=info.baseName();
    assert(!baseName.isEmpty());

    QFile file(csvFile);
    QTextStream in(&file);
    QString filedata=in.readAll();
    QStringList csvFileData=filedata.split("\n",QString::SkipEmptyParts);
    int id,frameNum;
    Rect r;

    FrameInput cap;
    int lineNum=0;
    int input_frameNum=0;
    int obj_frameNum=0;
    Mat img;

    //load trajectory to objectRecordMap and save best file.
    while(1){

        if(lineNum>=csvFileData.size()){
            break;
        }

        QString line=csvFileData[lineNum++];
        yzbxlib::csvToTrajectory(line,obj_frameNum,id,r);
        while(obj_frameNum>=input_frameNum){
            cap.getNextFrame(videoFile,img);
            input_frameNum++;
            assert(!img.empty());
        }
        assert(obj_frameNum==input_frameNum-1);

        if(objectRecordMap.find(id)==objectRecordMap.end()){
            ObjectRecord record;
            record.id=id;
            record.frameNum=obj_frameNum;


            //create dir to save object image
            QFileInfo dirInfo(globalImageDatabaseDir+"/"+baseName);
            if(!dirInfo.isDir()){
                QDir dir;
                bool flag=dir.mkdir(globalImageDatabaseDir+"/"+baseName);
                assert(flag);
            }

            //if object image file exist at first time, this mean it's the best! mark bestFileSaved=true.
            //if object image file not exist at first time, we save it without check onBoundary
            //for the rest loop, if onBoundary==false, which mean bestFile, we resave the file and mark bestFileSaved=true
            record.saveFilePath=globalImageDatabaseDir+"/"+baseName+QString::number(id)+".jpg";
            QFileInfo info(record.saveFilePath);
            if(info.isFile()){
                record.bestFileSaved=true;
            }
            else{
                Mat firstImageFile=img(r);
                bool flag=imwrite(record.saveFilePath.toStdString(),firstImageFile);
                assert(flag);
            }

            objectRecordMap[id]=record;
        }

        ObjectRecord & record=objectRecordMap[id];
        record.rectTrace.push_back(r);
        if(!record.bestFileSaved){
            bool onBoundary=yzbxlib::rectOnBoundary(r,img);
            if(!onBoundary){
                Mat firstImageFile=img(r);
                bool flag=imwrite(record.saveFilePath.toStdString(),firstImageFile);
                record.bestFileSaved=true;
                assert(flag);
            }
        }
    }

    //show best file on listwidget
//    m_listeWidget->addItem(new QListWidgetItem(QIcon("../ics.jpg"),"Wallpaper"));
    ui->listWidget->clear();
    for(auto it=objectRecordMap.begin();it!=objectRecordMap.end();it++){
        int id=it->first;
        ObjectRecord &r=it->second;
        QString text="id="+QString::number(id);
        ui->listWidget->addItem(new QListWidgetItem(QIcon(r.saveFilePath),text));
    }

}
