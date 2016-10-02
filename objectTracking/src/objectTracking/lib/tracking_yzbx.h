#ifndef TRACKING_YZBX_H
#define TRACKING_YZBX_H
#include <iostream>
#include <QtCore>
#include "bgsfactory_yzbx.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>
#include <../yzbxLib/trackingStatus.h>
#include <../yzbxLib/frameinput.h>
#include <../yzbxLib/qyzbxTrackingFeatures.h>

class Tracking_yzbx : public QThread
{
    Q_OBJECT
public:
    virtual void process(QString configFile, QString videoFile, TrackingStatus *status=NULL) = 0;
    virtual void run()=0;
    virtual void stop()=0;
    QString absoluteFilePath(QString currentPathOrFile, QString fileName)
    {
        QFileInfo pinfo(currentPathOrFile);
        QString currentPath=pinfo.absolutePath();


        QFileInfo info;
        info.setFile(QDir(currentPath),fileName);
        if(info.isAbsolute()){
            return fileName;
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
    virtual ~Tracking_yzbx(){}
};
#endif // TRACKING_YZBX_H
