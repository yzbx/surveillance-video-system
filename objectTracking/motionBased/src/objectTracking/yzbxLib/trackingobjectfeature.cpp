#include "trackingobjectfeature.h"

trackingObjectFeature::trackingObjectFeature()
{

}

QString trackingObjectFeature::dump()
{
    QStringList strs;
    strs<<QString::number(pos.x)<<QString::number(pos.y)<<
        QString::number(rect.x)<<QString::number(rect.y)<<
        QString::number(rect.width)<<QString::number(rect.height)<<
        QString::number(size)<<QString::number(radius)<<
        QString::number(Convexity)<<QString::number(Circularity)<<QString::number(Inertia);

    return strs.join("\t");
}

void trackingObjectFeature::fromString(QString str){
    QStringList strs=str.split("\t",QString::SkipEmptyParts);
    pos.x=strs[0].toFloat();
    pos.y=strs[1].toFloat();
    rect.x=strs[2].toInt();
    rect.x=strs[3].toInt();
    rect.width=strs[4].toFloat();
    rect.height=strs[5].toFloat();
    size=strs[6].toFloat();
    radius=strs[7].toFloat();
    Convexity=strs[8].toFloat();
    Circularity=strs[9].toFloat();
    Inertia=strs[10].toFloat();
}
