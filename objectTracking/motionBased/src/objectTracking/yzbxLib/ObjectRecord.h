#ifndef OBJECTRECORD_H
#define OBJECTRECORD_H
#include <QtCore>
#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
class ObjectRecord
{
public:
    ObjectRecord();
    QString videoPath;
    QString imagePath;
    QString objectType;
    QString saveFilePath;
    int id;
    vector<Rect> rectTrace;
    vector<int> frameTrace;
    int frameNum;
    bool bestFileSaved=false;
};

#endif // OBJECTRECORD_H
