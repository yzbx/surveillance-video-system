#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    globalInited=false;
    ui->setupUi(this);
    bgsFactory_yzbx bgsFac;
    QStringList list=bgsFac.getBgsTypeList();
    list.sort();
    ui->comboBox_bgsType->addItems(list);

    trackingFactory_yzbx trackingFac;
    QStringList tlist=trackingFac.getTrackingTypeList();
    tlist.sort();
    ui->comboBox_trackingType->addItems(tlist);

    ui->lineEdit_inputPath->setReadOnly(true);
    ui->lineEdit_inputPath->setEnabled(false);

    globalDatasetList<<"Multi-CameraTracking"<<"MOT2D2015-Train"<<"MOT2D2015-Test"<<"VisualTrackerBenchmark";
    globalDatasetList.sort();
    ui->comboBox_dataset->addItems(globalDatasetList);

    QString configFilePath="/home/yzbx/config/surveillance-video-system.ini";
    ui->lineEdit_inputPath->setText(configFilePath);
    loadIni(configFilePath);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pushButton_inputPath_clicked()
{
        QString fileName = QFileDialog::getOpenFileName(this,
                                                        tr("Open config file"), "",
                                                        tr("ini (*.ini)"));

//    QString fileName = QFileDialog::getExistingDirectory(this, tr("Open Input Directory"),
//                                                         ".",
//                                                         QFileDialog::ShowDirsOnly
//                                                         | QFileDialog::DontResolveSymlinks);
    if(!fileName.isEmpty()){
        ui->lineEdit_inputPath->setText(fileName);

        QFileInfo finfo(fileName);
        if(finfo.exists()){
            globalDatasetChanged=true;
            loadIni(fileName);
            globalDatasetChanged=false;
        }
        else{
            qDebug()<<"cannot find file "<<fileName;
            exit(-1);
        }

    }
}

void MainWindow::on_pushButton_bgs_clicked()
{
    cv::destroyAllWindows();

    QString bgsType=ui->comboBox_bgsType->currentText();
    bgsFactory_yzbx bgsFac;
    QString videoFilePath=ui->comboBox_video->currentText();
    if(videoFilePath.compare("all")!=0){
        QString inputPath=globalVideoHome+"/"+videoFilePath;
        bgsFac.process(bgsType,inputPath);
    }
    else{
        int n=globalVideosList.length();

        for(int i=0;i<n;i++){
            QString inputPath=globalVideoHome+"/"+globalVideosList.at(i);
            bgsFac.process(bgsType,inputPath);
        }
    }

}

void MainWindow::loadIni(QString filepath)
{
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(filepath.toStdString(),pt);


    QString RecordHome=QString::fromStdString(pt.get<std::string>("General.RecordHome"));
    QString Dataset=QString::fromStdString(pt.get<std::string>("General.Dataset"));
    if(globalInited){
        Dataset=ui->comboBox_dataset->currentText();
    }

    QString VideoHome=QString::fromStdString(pt.get<std::string>(Dataset.toStdString()+".VideoHome"));
    QString VideoTxt=QString::fromStdString(pt.get<std::string>(Dataset.toStdString()+".VideoTxt"));
    QString AnnotationHome=QString::fromStdString(pt.get<std::string>(Dataset.toStdString()+".AnnotationHome"));
    QString AnnotationTxt=QString::fromStdString(pt.get<std::string>(Dataset.toStdString()+".AnnotationTxt"));

    VideoHome=absoluteFilePath(filepath,VideoHome);
    VideoTxt=absoluteFilePath(filepath,VideoTxt);
    AnnotationHome=absoluteFilePath(filepath,AnnotationHome);
    AnnotationTxt=absoluteFilePath(filepath,AnnotationTxt);
    RecordHome=absoluteFilePath(filepath,RecordHome);

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


    file.setFileName(AnnotationTxt);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        qDebug()<<"cannot open file "<<AnnotationTxt;
        exit(-1);
    } else {
        QTextStream in(&file);
        filedata=in.readAll();
        globalAnnotationList=filedata.split("\n",QString::SkipEmptyParts);
        globalAnnotationHome=AnnotationHome;
        file.close();
    }

    if(!globalInited||globalDatasetChanged){
        ui->comboBox_video->clear();
        ui->comboBox_video->addItem("all");
        ui->comboBox_video->addItems(globalVideosList);
    }
    ui->lineEdit_recordPath->setText(RecordHome);

    if(ui->comboBox_replay->currentText()=="record"){
        ui->lineEdit_recordPath->setText(RecordHome);
    }
    else{
        ui->lineEdit_recordPath->setText(AnnotationHome);
    }

    if(!globalInited){
        ui->comboBox_dataset->setCurrentText(Dataset);
        globalInited=true;
    }
}

//BUG how to deal with fileName=../output ?
QString MainWindow::absoluteFilePath(QString currentPathOrFile, QString fileName)
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

void MainWindow::on_pushButton_tracking_clicked()
{
    cv::destroyAllWindows();
    QString trackType=ui->comboBox_trackingType->currentText();
    QString configFile=ui->lineEdit_inputPath->text();


    trackingFactory_yzbx trackerFac;
    Tracking_yzbx *tracker=trackerFac.getTrackingAlgorithm(trackType);
    QString videoFilePath=ui->comboBox_video->currentText();
    if(videoFilePath.compare("all")!=0){
        QString inputPath=globalVideoHome+"/"+videoFilePath;
        tracker->process(configFile,inputPath);
    }
    else{
        int n=globalVideosList.length();

        for(int i=0;i<n;i++){
            QString inputPath=globalVideoHome+"/"+globalVideosList.at(i);
            tracker->process(configFile,inputPath);
        }
    }
}

void MainWindow::on_pushButton_recordReplay_clicked()
{
    QString Dataset=ui->comboBox_dataset->currentText();
    QString recordFormat;
    //for urbantracker, find *.sqlite.
    if(Dataset=="Multi-CameraTracking"){
        recordFormat="txt";
    }
    else if(Dataset=="MOT2D2015-Train"||Dataset=="MOT2D2015-Test"){
        recordFormat="dat";
    }
    else if(Dataset=="VisualTrackerBenchmark"){
        recordFormat="txt";
    }
    else{
        qDebug()<<"undefine Dataset in replay: "<<Dataset;
        exit(-1);
    }

    QString RecordHome=ui->lineEdit_recordPath->text();

    QDir RecordDir(RecordHome);
    QDir VideoDir(globalVideoHome);
    QString videoText=ui->comboBox_video->currentText();
    if(videoText=="all"){
        int n=globalVideosList.length();

        for(int i=0;i<n;i++){
            QString videoFile=globalVideosList.at(i);
            QString videoFilePath=VideoDir.absoluteFilePath(videoFile);
            QString recordFile=globalAnnotationList.at(i);
            QString recordFilePath=RecordDir.absoluteFilePath(recordFile);

            QFileInfo info(recordFilePath);
            if(info.exists()){
                qDebug()<<i<<" record file path is "<<recordFilePath<<videoFilePath;
            }
            else{
                qDebug()<<"cannot find file"<<recordFilePath<<"under RecordHome:"<<RecordHome;
                qDebug()<<"please make sure RecordHome is right";
                qDebug()<<"and make sure record file have the same base name with video file";
            }
        }
    }
    else{
        QString videoFile=videoText;
        int i=globalVideosList.indexOf(videoFile);
        QString videoFilePath=VideoDir.absoluteFilePath(videoFile);
        QString recordFile=globalAnnotationList.at(i);
        QString recordFilePath=RecordDir.absoluteFilePath(recordFile);

        QFileInfo info(recordFilePath);
        if(info.exists()){
            qDebug()<<i<<" record file path is "<<recordFilePath<<videoFilePath;
        }
        else{
            qDebug()<<"cannot find file"<<recordFilePath<<"under RecordHome:"<<RecordHome;
            qDebug()<<"please make sure RecordHome is right";
            qDebug()<<"and make sure record file have the same base name with video file";
        }
    }
}

void MainWindow::on_comboBox_dataset_currentIndexChanged(const QString &arg1)
{
    if(globalInited){
        qDebug()<<"load dataset "<<arg1;
        globalDatasetChanged=true;
        loadIni(ui->lineEdit_inputPath->text());
        globalDatasetChanged=false;
    }
}

void MainWindow::on_comboBox_replay_currentIndexChanged(const QString &arg1)
{
    if(globalInited){
        qDebug()<<"replay type: "<<arg1;
        loadIni(ui->lineEdit_inputPath->text());
    }
}
