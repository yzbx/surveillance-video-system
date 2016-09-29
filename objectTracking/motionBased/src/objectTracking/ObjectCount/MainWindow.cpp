#include "MainWindow.h"
#include "ui_MainWindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QString configFilePath="/home/yzbx/config/surveillance-video-system.ini";
    ui->lineEdit_inputPath->setText(configFilePath);

    globalInited=false;
    loadIni(configFilePath);
    globalInited=true;
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::loadIni(QString filepath){
    boost::property_tree::ptree globalPt;
    boost::property_tree::ini_parser::read_ini(filepath.toStdString(),globalPt);

    QString Dataset=QString::fromStdString(globalPt.get<std::string>("General.Dataset"));
    if(globalInited){
        Dataset=ui->comboBox_dataset->currentText();
        globalPt.put("General.Dataset",Dataset.toStdString());
        boost::property_tree::ini_parser::write_ini(filepath.toStdString(),globalPt);
    }
    else{
        QString DatasetTxt=QString::fromStdString(globalPt.get<std::string>("General.DatasetTxt"));
        if(!DatasetTxt.isEmpty()){
            QString filedata;
            QFile file;
            file.setFileName(DatasetTxt);
            if (!file.open(QFile::ReadOnly | QFile::Text)) {
                qDebug()<<"cannot open file "<<DatasetTxt;
                exit(-1);
            } else {
                QTextStream in(&file);
                filedata=in.readAll();
                QStringList globalDatasetList=filedata.split("\n",QString::SkipEmptyParts);

                ui->comboBox_dataset->clear();
                ui->comboBox_dataset->addItems(globalDatasetList);
                file.close();
            }
        }
    }

    assert(Dataset=="Multi-CameraTracking"||Dataset=="UrbanTrackerDataset"||Dataset=="UserDefine");

    QString VideoHome=QString::fromStdString(globalPt.get<std::string>(Dataset.toStdString()+".VideoHome"));
    QString VideoTxt=QString::fromStdString(globalPt.get<std::string>(Dataset.toStdString()+".VideoTxt"));

    VideoHome=yzbxlib::getAbsoluteFilePath(filepath,VideoHome);
    VideoTxt=yzbxlib::getAbsoluteFilePath(filepath,VideoTxt);

    QString filedata;
    QFile file;
    file.setFileName(VideoTxt);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        qDebug()<<"cannot open file "<<VideoTxt;
        exit(-1);
    } else {
        QTextStream in(&file);
        filedata=in.readAll();
        QStringList globalVideosList=filedata.split("\n",QString::SkipEmptyParts);
        ui->comboBox_video->clear();
        ui->comboBox_video->addItems(globalVideosList);
        ui->comboBox_video->addItem("all");
        file.close();
    }

}

void MainWindow::on_pushButton_clicked()
{
    QString videoFile=ui->comboBox_video->currentText();
    QString configFile=ui->lineEdit_inputPath->text();
    tracker=std::make_unique<DataDriveTracker>(DataDriveTracker(configFile));

    if(videoFile=="all"){
        tracker->runAll();
    }
    else{
        tracker->mainData->setCurrentVideo(videoFile);
        tracker->run();
    }
}

void MainWindow::on_comboBox_dataset_currentTextChanged(const QString &arg1)
{
    qDebug()<<"change to dataset "<<arg1;
    QString filepath=ui->lineEdit_inputPath->text();
    loadIni(filepath);
}
