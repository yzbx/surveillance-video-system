#include "urbantracker_tracking.h"

UrbanTracker_tracking::UrbanTracker_tracking()
{

}

void UrbanTracker_tracking::process(QString configFile,QString videoFile)
{
    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(configFile.toStdString(),pt);
    QString cmd;

    QString OutputPath=QString::fromStdString(pt.get<std::string>("General.OutputPath"));
    OutputPath=absoluteFilePath(configFile,OutputPath);

    QString App=QString::fromStdString(pt.get<std::string>("UrbanTracker.App"));
    App=absoluteFilePath(configFile,App);
    QString maincfg=QString::fromStdString(pt.get<std::string>("UrbanTracker.maincfg"));
    maincfg=absoluteFilePath(configFile,maincfg);
    QString filecfg=absoluteFilePath(maincfg,"filepath.cfg");


    boost::property_tree::ptree filept;
    boost::property_tree::ini_parser::read_ini(filecfg.toStdString(),filept);

    QFileInfo info(videoFile);
    QString sqlFile=OutputPath+"/"+info.baseName()+".sqlite";

    filept.put("video-filename",videoFile.toStdString());
    filept.put("object-sqlite-filename",sqlFile.toStdString());
    boost::property_tree::ini_parser::write_ini(filecfg.toStdString(),filept);

    cmd=App+" "+maincfg;
//    QProcess::startDetached(cmd);
    QProcess::execute(cmd);
}
