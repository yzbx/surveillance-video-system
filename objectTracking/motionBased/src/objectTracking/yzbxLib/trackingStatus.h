#ifndef TRACKINGSTATUS_H
#define TRACKINGSTATUS_H
#include <QtCore>
#include "frameinput.h"
#include "qyzbxTrackingFeatures.h"


class TrackingStatus
{
public:
    TrackingStatus();

    IBGS *ibgs=NULL;
    QString BGSType;
    FrameInput frameinput;
    list<FrameFeature> frameFeatureList;
    int trackingWindowSize=20;
    int initFrameNum=100;
    bool bgsInited=false;
};

#endif // TRACKINGSTATUS_H
