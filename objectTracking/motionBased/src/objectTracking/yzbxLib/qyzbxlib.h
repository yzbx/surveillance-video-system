#ifndef QYZBXLIB_H
#define QYZBXLIB_H

#include "yzbxlib_global.h"
#include "yzbx_config.h"
#include "yzbx_utility.h"
#include "qyzbxTrackingFeatures.h"
#include "../lib/trackingfactory_yzbx.h"
#include "../extern/qt-json/json.h"
#include "frameinput.h"
#include "TrackingResultReplay.h"
#include "HungarianBasedTracking.h"
#include <assert.h>
#include "../extern/csv.h"
#include <boost/lexical_cast.hpp>

namespace yzbxlib
{
QString getAbsoluteFilePath(QString currentPathOrFile, QString fileName);
bool isSameImage(const cv::Mat &A_8U,const cv::Mat &B_8U);
}

class YZBXLIBSHARED_EXPORT QYzbxLib
{

public:
    QYzbxLib();
};

#endif // QYZBXLIB_H
