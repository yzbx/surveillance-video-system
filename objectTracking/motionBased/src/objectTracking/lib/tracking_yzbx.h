#ifndef TRACKING_YZBX_H
#define TRACKING_YZBX_H
#include <iostream>
#include <QtCore>
#include "bgsfactory_yzbx.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

class Tracking_yzbx
{
public:
    virtual void process(QString configFile, QString videoFile) = 0;
    QString absoluteFilePath(QString currentPathOrFile, QString fileName)
    {
        QFileInfo pinfo(currentPathOrFile);
        QString currentPath=pinfo.absolutePath();


        QFileInfo info(fileName);
        if(info.isAbsolute()){
            return fileName;
        }
        else{
            if(info.isDir()){
                QDir dir(currentPath);
                dir.cd(fileName);
                return dir.absolutePath();
            }
            else{
                QDir dir(currentPath);
                return dir.absoluteFilePath(fileName);
            }
        }
    }
    virtual ~Tracking_yzbx(){}
};
#endif // TRACKING_YZBX_H
