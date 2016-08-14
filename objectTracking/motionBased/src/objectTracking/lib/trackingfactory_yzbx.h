#ifndef TRACKINGFACTORY_YZBX_H
#define TRACKINGFACTORY_YZBX_H
#include "tracking_yzbx.h"
#include "urbantracker_tracking.h"

class trackingFactory_yzbx
{
public:
    trackingFactory_yzbx();
    Tracking_yzbx *getTrackingAlgorithm(QString trackingType);
    QStringList getTrackingTypeList();
};

#endif // TRACKINGFACTORY_YZBX_H
