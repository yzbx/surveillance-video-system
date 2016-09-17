#include "DataDriveMain.h"

DataDriveMain::DataDriveMain(QString configFile)
{
    boost::property_tree::ini_parser::read_ini(configFile.toStdString(),globalPt);
    QString Dataset=QString::fromStdString(globalPt.get<std::string>("General.Dataset"));
    QString VideoHome=QString::fromStdString(globalPt.get<std::string>(Dataset.toStdString()+".VideoHome"));
    QString VideoTxt=QString::fromStdString(globalPt.get<std::string>(Dataset.toStdString()+".VideoTxt"));

    VideoHome=yzbxlib::getAbsoluteFilePath(configFile,VideoHome);
    VideoTxt=yzbxlib::getAbsoluteFilePath(configFile,VideoTxt);

    QString filedata;
    QFile file;
    file.setFileName(VideoTxt);
    if (!file.open(QFile::ReadOnly | QFile::Text)) {
        qDebug()<<"cannot open file "<<VideoTxt;
        exit(-1);
    } else {
        QTextStream in(&file);
        filedata=in.readAll();
        globalVideoList=filedata.split("\n",QString::SkipEmptyParts);
        globalVideoHome=VideoHome;
        file.close();
    }

    bgsFactory_yzbx fac;
    bgs=fac.getBgsAlgorithm(bgsType);
}
