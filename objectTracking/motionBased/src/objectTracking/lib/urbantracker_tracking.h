#ifndef URBANTRACKER_TRACKING_H
#define URBANTRACKER_TRACKING_H
#include "tracking_yzbx.h"
#include <QtCore>

class UrbanTracker_tracking : public Tracking_yzbx
{
public:
    UrbanTracker_tracking();
    void process(QString configFile, QString videoFile, TrackingStatus *status=NULL);
    void run();
    void stop();

    QProcess *globalProcess;
};

#endif // URBANTRACKER_TRACKING_H
