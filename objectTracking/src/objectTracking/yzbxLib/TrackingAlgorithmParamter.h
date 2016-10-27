#ifndef TRACKINGALGORITHMPARAMTER_H
#define TRACKINGALGORITHMPARAMTER_H
#include <iostream>
#include "yzbx_config.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

class TrackingAlgorithmParamter
{
public:
    TrackingAlgorithmParamter();
    void loadConfig(std::string ParamConfigFile){
        boost::property_tree::ptree pt;
        boost::property_tree::ini_parser::read_ini(ParamConfigFile,pt);
        dt=pt.get("DataDrive.dt",0.2f);
        Accel_noise_mag=pt.get("DataDrive.Accel_noise_mag",0.1f);
        dist_thres=pt.get("DataDrive.dist_thres",100);
        maximum_allowed_skipped_frames=pt.get("DataDrive.maximum_allowed_skipped_frames",100);
        max_trace_length=pt.get("DataDrive.max_trace_length",100);
        MatchNumThreshold=pt.get("DataDrive.MatchNumThreshold",3);
        MaxFreshObjectLifeTime=pt.get("DataDrive.MaxFreshObjectLifeTime",3);
        MinDumpLifeTime=pt.get("DataDrive.MinDumpLifeTime",10);
        whog=pt.get("ReDetection.HOG_Weight",0.5f);
        wcolor=pt.get("ReDetection.Color_Weight",0.5f);
        rr_threshold=pt.get("ReDetection.Threshold",5.0f);
        MinBlobArea=pt.get("BlobDetector.MinBlobArea",200);
        ReSize=pt.get("BlobDetector.ReSize",false);
    }

    track_t dt=0.2f;
    track_t Accel_noise_mag=0.1f;
    uint dist_thres = 100;
    uint maximum_allowed_skipped_frames = 100;
    uint max_trace_length=100;
    uint MatchNumThreshold=3;
    uint MaxFreshObjectLifeTime=3;
    uint MinDumpLifeTime=10;
    double whog,wcolor,rr_threshold;
    uint MinBlobArea;
    bool ReSize;
};

#endif // TRACKINGALGORITHMPARAMTER_H
