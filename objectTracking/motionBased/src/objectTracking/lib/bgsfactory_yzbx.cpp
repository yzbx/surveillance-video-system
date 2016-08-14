#include "bgsfactory_yzbx.h"

bgsFactory_yzbx::bgsFactory_yzbx()
{
    QDir dir;
    dir.mkdir("config");
}

IBGS *bgsFactory_yzbx::getBgsAlgorithm(QString bgstype)
{
    FrameDifferenceBGS* frameDifference;

    if(bgstype.compare("FrameDifferenceBGS",Qt::CaseInsensitive)==0){
        frameDifference=new FrameDifferenceBGS;
        return frameDifference;
    }

    StaticFrameDifferenceBGS* staticFrameDifference;

    if(bgstype.compare("StaticFrameDifferenceBGS",Qt::CaseInsensitive)==0){
        staticFrameDifference=new StaticFrameDifferenceBGS;
        return staticFrameDifference;
    }

    WeightedMovingMeanBGS* weightedMovingMean;

    if(bgstype.compare("WeightedMovingMeanBGS",Qt::CaseInsensitive)==0){
        weightedMovingMean=new WeightedMovingMeanBGS;
        return weightedMovingMean;
    }

    WeightedMovingVarianceBGS* weightedMovingVariance;
    if(bgstype.compare("WeightedMovingVarianceBGS",Qt::CaseInsensitive)==0){
        weightedMovingVariance=new WeightedMovingVarianceBGS;
        return weightedMovingVariance;
    }

    MixtureOfGaussianV1BGS* mixtureOfGaussianV1BGS;
    if(bgstype.compare("MixtureOfGaussianV1BGS",Qt::CaseInsensitive)==0){
        mixtureOfGaussianV1BGS=new MixtureOfGaussianV1BGS;
        return mixtureOfGaussianV1BGS;
    }

    MixtureOfGaussianV2BGS* mixtureOfGaussianV2BGS;
    if(bgstype.compare("MixtureOfGaussianV2BGS",Qt::CaseInsensitive)==0){
        mixtureOfGaussianV2BGS=new MixtureOfGaussianV2BGS;
        return mixtureOfGaussianV2BGS;
    }

    AdaptiveBackgroundLearning* adaptiveBackgroundLearning;
    if(bgstype.compare("AdaptiveBackgroundLearning",Qt::CaseInsensitive)==0){
        adaptiveBackgroundLearning=new AdaptiveBackgroundLearning;
        return adaptiveBackgroundLearning;
    }

#if CV_MAJOR_VERSION >= 2 && CV_MINOR_VERSION >= 4 && CV_SUBMINOR_VERSION >= 3
    GMG* gmg;
    if(bgstype.compare("GMG",Qt::CaseInsensitive)==0){
        gmg=new GMG;
        return gmg;
    }
#endif

    DPAdaptiveMedianBGS* adaptiveMedian;
    if(bgstype.compare("DPAdaptiveMedianBGS",Qt::CaseInsensitive)==0){
        adaptiveMedian=new DPAdaptiveMedianBGS;
        return adaptiveMedian;
    }

    DPGrimsonGMMBGS* grimsonGMM;
    if(bgstype.compare("DPGrimsonGMMBGS",Qt::CaseInsensitive)==0){
        grimsonGMM=new DPGrimsonGMMBGS;
        return grimsonGMM;
    }

    DPZivkovicAGMMBGS* zivkovicAGMM;
    if(bgstype.compare("DPZivkovicAGMMBGS",Qt::CaseInsensitive)==0){
        zivkovicAGMM=new DPZivkovicAGMMBGS;
        return zivkovicAGMM;
    }

    DPMeanBGS* temporalMean;
    if(bgstype.compare("DPMeanBGS",Qt::CaseInsensitive)==0){
        temporalMean=new DPMeanBGS;
        return temporalMean;
    }

    DPWrenGABGS* wrenGA;
    if(bgstype.compare("DPWrenGABGS",Qt::CaseInsensitive)==0){
        wrenGA=new DPWrenGABGS;
        return wrenGA;
    }

    DPPratiMediodBGS* pratiMediod;
    if(bgstype.compare("DPPratiMediodBGS",Qt::CaseInsensitive)==0){
        pratiMediod=new DPPratiMediodBGS;
        return pratiMediod;
    }

    DPEigenbackgroundBGS* eigenBackground;
    if(bgstype.compare("DPEigenbackgroundBGS",Qt::CaseInsensitive)==0){
        eigenBackground=new DPEigenbackgroundBGS;
        return eigenBackground;
    }

    DPTextureBGS* textureBGS;
    if(bgstype.compare("DPTextureBGS",Qt::CaseInsensitive)==0){
        textureBGS=new DPTextureBGS;
        return textureBGS;
    }

    T2FGMM_UM* type2FuzzyGMM_UM;
    if(bgstype.compare("T2FGMM_UM",Qt::CaseInsensitive)==0){
        type2FuzzyGMM_UM=new T2FGMM_UM;
        return type2FuzzyGMM_UM;
    }

    T2FGMM_UV* type2FuzzyGMM_UV;
    if(bgstype.compare("T2FGMM_UV",Qt::CaseInsensitive)==0){
        type2FuzzyGMM_UV=new T2FGMM_UV;
        return type2FuzzyGMM_UV;
    }

    T2FMRF_UM* type2FuzzyMRF_UM;
    if(bgstype.compare("T2FMRF_UM",Qt::CaseInsensitive)==0){
        type2FuzzyMRF_UM=new T2FMRF_UM;
        return type2FuzzyMRF_UM;
    }

    T2FMRF_UV* type2FuzzyMRF_UV;
    if(bgstype.compare("T2FMRF_UV",Qt::CaseInsensitive)==0){
        type2FuzzyMRF_UV=new T2FMRF_UV;
        return type2FuzzyMRF_UV;
    }

    FuzzySugenoIntegral* fuzzySugenoIntegral;
    if(bgstype.compare("FuzzySugenoIntegral",Qt::CaseInsensitive)==0){
        fuzzySugenoIntegral=new FuzzySugenoIntegral;
        return fuzzySugenoIntegral;
    }

    FuzzyChoquetIntegral* fuzzyChoquetIntegral;
    if(bgstype.compare("FuzzyChoquetIntegral",Qt::CaseInsensitive)==0){
        fuzzyChoquetIntegral=new FuzzyChoquetIntegral;
        return fuzzyChoquetIntegral;
    }

    LBSimpleGaussian* lbSimpleGaussian;
    if(bgstype.compare("LBSimpleGaussian",Qt::CaseInsensitive)==0){
        lbSimpleGaussian=new LBSimpleGaussian;
        return lbSimpleGaussian;
    }

    LBFuzzyGaussian* lbFuzzyGaussian;
    if(bgstype.compare("LBFuzzyGaussian",Qt::CaseInsensitive)==0){
        lbFuzzyGaussian=new LBFuzzyGaussian;
        return lbFuzzyGaussian;
    }

    LBMixtureOfGaussians* lbMixtureOfGaussians;
    if(bgstype.compare("LBMixtureOfGaussians",Qt::CaseInsensitive)==0){
        lbMixtureOfGaussians=new LBMixtureOfGaussians;
        return lbMixtureOfGaussians;
    }

    LBAdaptiveSOM* lbAdaptiveSOM;
    if(bgstype.compare("LBAdaptiveSOM",Qt::CaseInsensitive)==0){
        lbAdaptiveSOM=new LBAdaptiveSOM;
        return lbAdaptiveSOM;
    }

    LBFuzzyAdaptiveSOM* lbFuzzyAdaptiveSOM;
    if(bgstype.compare("LBFuzzyAdaptiveSOM",Qt::CaseInsensitive)==0){
        lbFuzzyAdaptiveSOM=new LBFuzzyAdaptiveSOM;
        return lbFuzzyAdaptiveSOM;
    }

    LbpMrf* lbpMrf;
    if(bgstype.compare("LbpMrf",Qt::CaseInsensitive)==0){
        lbpMrf=new LbpMrf;
        return lbpMrf;
    }

    MultiLayerBGS* multiLayerBGS;
    if(bgstype.compare("MultiLayerBGS",Qt::CaseInsensitive)==0){
        multiLayerBGS=new MultiLayerBGS;
        return multiLayerBGS;
    }

    //cv::Mat img_pt_pbas;
    //PixelBasedAdaptiveSegmenter* pixelBasedAdaptiveSegmenter;
    //bool enablePBAS;

    VuMeter* vuMeter;
    if(bgstype.compare("VuMeter",Qt::CaseInsensitive)==0){
        vuMeter=new VuMeter;
        return vuMeter;
    }

    KDE* kde;
    if(bgstype.compare("KDE",Qt::CaseInsensitive)==0){
        kde=new KDE;
        return kde;
    }

    IndependentMultimodalBGS* imbs;
    if(bgstype.compare("IndependentMultimodalBGS",Qt::CaseInsensitive)==0){
        imbs=new IndependentMultimodalBGS;
        return imbs;
    }

    SJN_MultiCueBGS* mcbgs;
    if(bgstype.compare("SJN_MultiCueBGS",Qt::CaseInsensitive)==0){
        mcbgs=new SJN_MultiCueBGS;
        return mcbgs;
    }

    SigmaDeltaBGS* sdbgs;
    if(bgstype.compare("SigmaDeltaBGS",Qt::CaseInsensitive)==0){
        sdbgs=new SigmaDeltaBGS;
        return sdbgs;
    }

    SuBSENSEBGS* ssbgs;
    if(bgstype.compare("SuBSENSEBGS",Qt::CaseInsensitive)==0){
        ssbgs=new SuBSENSEBGS;
        return ssbgs;
    }

    LOBSTERBGS* lobgs;
    if(bgstype.compare("LOBSTERBGS",Qt::CaseInsensitive)==0){
        lobgs=new LOBSTERBGS;
        return lobgs;
    }

    if(bgstype.compare("default",Qt::CaseInsensitive)==0){
        frameDifference=new FrameDifferenceBGS;
        return frameDifference;
    }

    qDebug()<<"unknow bgs type, please use the follow choice: ";
    qDebug()<<getBgsTypeList();

    return NULL;
}

QStringList bgsFactory_yzbx::getBgsTypeList()
{
    QStringList list;
    list<<"FrameDifferenceBGS"<<"StaticFrameDifferenceBGS"<<"WeightedMovingMeanBGS"<<"WeightedMovingVarianceBGS"<<"MixtureOfGaussianV1BGS"<<"MixtureOfGaussianV2BGS"<<"AdaptiveBackgroundLearning"<<"DPAdaptiveMedianBGS"<<"DPGrimsonGMMBGS"<<"DPZivkovicAGMMBGS"<<"DPMeanBGS"<<"DPWrenGABGS"<<"DPPratiMediodBGS"<<"DPEigenbackgroundBGS"<<"DPTextureBGS"<<"T2FGMM_UM"<<"T2FGMM_UV"<<"T2FMRF_UM"<<"T2FMRF_UV"<<"FuzzySugenoIntegral"<<"FuzzyChoquetIntegral"<<"LBSimpleGaussian"<<"LBFuzzyGaussian"<<"LBMixtureOfGaussians"<<"LBAdaptiveSOM"<<"LBFuzzyAdaptiveSOM"<<"LbpMrf"<<"MultiLayerBGS"<<"VuMeter"<<"KDE"<<"IndependentMultimodalBGS"<<"SJN_MultiCueBGS"<<"SigmaDeltaBGS"<<"SuBSENSEBGS"<<"LOBSTERBGS";
#if CV_MAJOR_VERSION >= 2 && CV_MINOR_VERSION >= 4 && CV_SUBMINOR_VERSION >= 3
    list<<"GMG";
#endif

    return list;
}

void bgsFactory_yzbx::process(QString bgstype, QString inputSource)
{
    IBGS *ibgs=getBgsAlgorithm(bgstype);
    QString inputPath=inputSource;
    if(ibgs==NULL||inputPath.isEmpty()){

        qDebug()<<"null ibgs or empty input path. "<<inputPath;
        return;
    }

    cv::VideoCapture cap(inputPath.toStdString());
    if(!cap.isOpened()){
        qDebug()<<"cannot open file "<<inputPath;
        return;
    }

    cv::Mat inputFrame;
    cv::Mat foregroundFrame;
    cv::Mat backgroundFrame;
    cv::namedWindow("input",cv::WINDOW_NORMAL);
//    cv::namedWindow("foreground",cv::WINDOW_NORMAL);
//    cv::namedWindow("background",cv::WINDOW_NORMAL);

    int frameNum=0;
    do{
        qDebug()<<frameNum;
        frameNum++;
        cap>>inputFrame;

        if(!inputFrame.empty()){
            ibgs->process(inputFrame,foregroundFrame,backgroundFrame);
            cv::imshow("input",inputFrame);
//            if(!foregroundFrame.empty()){
//                cv::imshow("foreground",foregroundFrame);
//            }

//            if(!backgroundFrame.empty()){
//                cv::imshow("background",backgroundFrame);
//            }
        }



        int key=cv::waitKey(30);
        if(key=='q'||key=='Q'){
            break;
        }
    }
    while(!inputFrame.empty());
}
