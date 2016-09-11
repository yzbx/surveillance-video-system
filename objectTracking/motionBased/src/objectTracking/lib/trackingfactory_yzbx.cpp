#include "trackingfactory_yzbx.h"

trackingFactory_yzbx::trackingFactory_yzbx()
{

}

Tracking_yzbx *trackingFactory_yzbx::getTrackingAlgorithm(QString trackingType)
{
    if(trackingType.compare("UrbanTracker",Qt::CaseInsensitive)==0){
        UrbanTracker_tracking *t=new UrbanTracker_tracking;
        return t;
    }
    else if(trackingType.compare("default",Qt::CaseInsensitive)==0){
        UrbanTracker_tracking *t=new UrbanTracker_tracking;
        return t;
    }
    else if(trackingType.compare("MotionBasedTracker",Qt::CaseInsensitive)==0){
        MotionBasedTracker *t=new MotionBasedTracker;
        return t;
    }
    else{
        qDebug()<<"unexpected trackingType:"<<trackingType;
        return NULL;
    }
}

QStringList trackingFactory_yzbx::getTrackingTypeList()
{
    QStringList list;
    list<<"BlobBasedTracker"<<"UrbanTracker"<<"default"<<"MotionBasedTracker"<<"MultiObjectTracking"<<"BlobBasedTracker";
    return list;
}
