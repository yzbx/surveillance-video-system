#ifndef YZBX_CONFIG
#define YZBX_CONFIG

//separate FG strategy
//A-B,B-A,A&B
#define Generate3FG 0
//A-B-C,B-A-C,C-A-B, else
#define Generate4FG 1
//A-B-C,B-A-C,C-A-B,A&B-A&B&C,A&C-A&B&C,B&C-A&B&C,A&B&C
#define Generate7FG 2
//simple mix, A|B|C
#define Generate1FG 3

//distance type
#define yzbx_distance_Hamming 0
#define yzbx_distance_L1 1
#define yzbx_distance_L2 2

//color space
#define yzbx_colorSpace_BGR 0
#define yzbx_colorSpace_LAB 1
#define yzbx_colorSpace_AB 2
#define yzbx_colorSpace_LABPlus 3

//point type
#define STATIC_POINT 0
#define UNKNOWN_POINT 1
#define MOVING_POINT 2
#define TEMPORAL_STOP_POINT 3
#define MAX_POINT_TYPE 4


//algorithm
#define LBP_BGS 0
#define NPE_BGS 1
#define ORG_BGS 2

//blob type defination strategy
#define BlobTypeRatio 0
#define BlobTypePreferMoving 1

//statistic
#define StatisticInputMean 0
#define StatisticInputVariance 1
#define StatisticFGMean 2
#define StatisticFGVariance 3
#define StatisticFGCount 4

//cache information
#define CacheOneFirst 0
#define CacheTwoFirst 1
#define CacheOneSize 2
#define CacheTwoSize 3

//LBP
#define LBP8UC1 0
#define LBP16UC1 1
#define LBP32UC1 2
#define LBP32FC1 3

//CDNet groundtruth
#define CDNet_GT_BG 0
#define CDNet_GT_Shadow 50
#define CDNet_GT_OutRoi 85
#define CDNet_GT_Unknown 170
#define CDNet_GT_Moving 255

//modelUpdateStrategy
enum ModelUpdateStrategy{
    REPLACE_ALL,
    REPLACE_LAB_ONLY,
    REPLACE_L_ONLY,
    UPDATE_ALL,
    UPDATE_LAB_ONLY,
    UPDATE_L_ONLY,
    UNCHANGE
};

enum StatisticDistanceMat{
    LAB_DISTANCE_MEAN,
    LAB_DISTANCE_STD,
    LBP_DISTANCE_MEAN,
    LBP_DISTANCE_STD
};

//#define REPLACE_ALL 0
//#define REPLACE_LAB_ONLY 1
//#define UPDATE_ALL 2
//#define UPDATE_LAB_ONLY 3
//#define UPDATE_L_ONLY 4

namespace yzbx_config {

namespace shadowRemove{
namespace Lab_la{
//use only l and a, but the effect is very good.
const size_t _value_size=2;
const size_t _cache_level_one_size=20;
const size_t _cache_level_two_size=25;
const size_t _cache_one_minFrequency=5;
const int _cache_hit_distance=100;
}

namespace Lab_ab{
const size_t _value_size=2;
const size_t _cache_level_one_size=20;
const size_t _cache_level_two_size=25;
const size_t _cache_one_minFrequency=5;
const int _cache_hit_distance=100;
}

}

namespace  npe_bgs {

namespace rgb{
const size_t _value_size=3;
const size_t _cache_level_one_size=20;
const size_t _cache_level_two_size=25;
const size_t _cache_one_minFrequency=5;
const int _cache_hit_distance=50;
}

namespace lab_l1{
const size_t _value_size=3;
const size_t _cache_level_one_size=20;
const size_t _cache_level_two_size=25;
const size_t _cache_one_minFrequency=5;
const int _cache_hit_distance=50;
}

namespace lab_l2{
const size_t _value_size=3;
const size_t _cache_level_one_size=20;
const size_t _cache_level_two_size=25;
const size_t _cache_one_minFrequency=5;
const int _cache_hit_distance=50;
}
}

}
#endif // YZBX_CONFIG

