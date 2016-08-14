#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    bgsFactory_yzbx bgsFac;
    QStringList list=bgsFac.getBgsTypeList();
    ui->comboBox_bgsType->addItems(list);

    trackingFactory_yzbx trackingFac;
    QStringList tlist=trackingFac.getTrackingTypeList();
    ui->comboBox_trackingType->addItems(tlist);

    ui->lineEdit_inputPath->setReadOnly(true);
    ui->lineEdit_inputPath->setEnabled(false);
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
            loadIni(fileName);
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

    QString VideoHome=QString::fromStdString(pt.get<std::string>("General.VideoHome"));
    QString VideoTxt=QString::fromStdString(pt.get<std::string>("General.VideoTxt"));
    QString SqliteHome=QString::fromStdString(pt.get<std::string>("General.SqliteHome"));
    VideoHome=absoluteFilePath(filepath,VideoHome);
    VideoTxt=absoluteFilePath(filepath,VideoTxt);
    SqliteHome=absoluteFilePath(filepath,SqliteHome);

    QString filedata;
    QFile file;
    file.setFileName(VideoTxt);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        qDebug()<<"cannot open file "<<VideoTxt;
        exit(-1);
    } else {
        QTextStream in(&file);
        filedata=in.readAll();
        file.close();
    }
    QStringList inputVideoList=filedata.split("\n");
    globalVideosList=inputVideoList;
    globalVideoHome=VideoHome;

    ui->comboBox_video->addItem("all");
    ui->comboBox_video->addItems(globalVideosList);
    ui->lineEdit_recordPath->setText(SqliteHome);
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
    //for urbantracker, find *.sqlite.
    QString SqliteHome=ui->lineEdit_recordPath->text();
    QStringList sqliteFileList;
    QDir dir(SqliteHome);
    QStringList filters;
    filters<<"*.sqlite";
    foreach(QString file, dir.entryList(filters,QDir::Files)){
        sqliteFileList<<file;
    }

    QString videoFilePath=ui->comboBox_video->currentText();
    if(videoFilePath=="all"){
        int n=globalVideosList.length();

        for(int i=0;i<n;i++){
            QFileInfo info(globalVideosList.at(i));
            QString sqliteFilePath=info.baseName()+".sqlite";


            if(sqliteFileList.contains(sqliteFilePath)){
                sqliteFilePath=absoluteFilePath(SqliteHome,sqliteFilePath);
                qDebug()<<i<<" sqlite file path is "<<sqliteFilePath;
            }
            else{
                qDebug()<<"cannot find file"<<sqliteFilePath<<"under SqliteHome:"<<SqliteHome;
                qDebug()<<"please make sure SqliteHome is right";
                qDebug()<<"and make sure sqlite file have the same base name with video file";
            }
        }
    }
    else{
        QFileInfo info(videoFilePath);
        QString sqliteFilePath=info.baseName()+".sqlite";
        if(sqliteFileList.contains(sqliteFilePath)){
            sqliteFilePath=absoluteFilePath(SqliteHome,sqliteFilePath);
            qDebug()<<" sqlite file path is "<<sqliteFilePath;
        }
        else{
            qDebug()<<"cannot find file"<<sqliteFilePath<<"under SqliteHome:"<<SqliteHome;
            qDebug()<<"please make sure SqliteHome is right";
            qDebug()<<"and make sure sqlite file have the same base name with video file";
        }
    }

    qDebug()<<"replay record";
}
