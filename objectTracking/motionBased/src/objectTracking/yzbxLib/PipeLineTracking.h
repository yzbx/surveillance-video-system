#ifndef PIPELINETRACKING_H
#define PIPELINETRACKING_H
#include <QtCore>
#include "frameinput.h"
#include "../lib/bgsfactory_yzbx.h"
#include "BlobDetector.h"
#include "RectFloatTracker.h"
#include "TrackingBlobsMatchAnnotation.h"
#include "TrackingResultReplay.h"

class PipeLineTracking
{
public:
    PipeLineTracking();
    void process(QString sourceData);
//    void PipeLine_Input();
//    void PipeLine_Bgs();
//    void PipeLine_Features();
//    void PipeLine_Tracking();
    void PipeLine_DumpFV(int frameNum, const vector<int> &ids, vector<trackingObjectFeature> &fv);

private:
    bool firstDump=true;
    track_t dist_thres=200;
    void PipeLine_Replay(QString dataSource, QString replaySource);
    QString bgsType="SJN_MultiCueBGS";
};

#endif // PIPELINETRACKING_H
