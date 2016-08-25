#include "qyzbxlib.h"


QYzbxLib::QYzbxLib()
{
}

namespace  yzbxlib {

QString getAbsoluteFilePath(QString currentPathOrFile, QString fileName)
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

bool isSameImage(const Mat &A_8U, const Mat &B_8U)
{
    cv::Mat C;
    cv::absdiff(A_8U,B_8U,C);
    cv::Scalar s=cv::sum(C);
    if(C.channels()==3){
        if(s[0]==0&&s[1]==0&&s[2]==0){
            return true;
        }
    }
    else{
        if(s[0]==0){
            return true;
        }
    }

    return false;
}

}
