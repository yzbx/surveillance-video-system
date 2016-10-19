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
#include "GraphBasedTracker.h"
#include "RectFloatTracker.h"
#include <assert.h>
#include "../extern/csv.h"
#include <boost/lexical_cast.hpp>
#include <boost/any.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "../extern/vibe/vibe.hpp"
#include "../extern/vibe/PBAS.h"
#include "PipeLineTracking.h"
#include "ObjectRecord.h"

class YZBXLIBSHARED_EXPORT QYzbxLib
{

public:
    QYzbxLib();
};

#endif // QYZBXLIB_H
