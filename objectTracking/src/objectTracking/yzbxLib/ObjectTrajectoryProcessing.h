#ifndef OBJECTTRAJECTORYPROCESSING_H
#define OBJECTTRAJECTORYPROCESSING_H
#include <QtCore>
#include <iostream>
#include <yzbx_config.h>

class ObjectTrajectoryProcessing
{
public:
    ObjectTrajectoryProcessing(QString inputFile,QString outputFile);
private:
    float distanceThreshold=100;
    class object{
    public:
        int id;
        std::list<std::pair<int,Rect_t>> rects;
    };

    std::list<object> objects;
    void updateObjects(int id, int frameNum, Rect_t rect);
    void loadObjects(QString inputFile);
    void filterObjects();
    void dumpObjects(QString outputFile);
    inline float calculateDistance(object &a, object &b){

        auto ita=a.rects.begin();
        auto itb=b.rects.begin();

        //use kalman to find weather object a in object b
        float frameNumDif=a.rects.size()-b.rects.size();
        int frameNumSame=0;
        float rectsDif=0.0;
        while(ita!=a.rects.end()&&itb!=b.rects.end()){
            int ida=ita->first;
            int idb=ita->first;
            if(ida>idb){
                itb++;
            }
            else if(ida<idb){
                ita++;
            }
            else{
                frameNumSame++;

                Rect_t &ra=(ita->second);
                Rect_t &rb=(itb->second);
                Point_t dbr=ra.br()-rb.br();
                Point_t dtl=ra.tl()-rb.tl();

                float dx=(ra.br().x-rb.br().x)*(ra.tl().x-rb.tl().x);
                float dy=(ra.br().y-rb.br().y)*(ra.tl().y-rb.tl().y);
                if(dx<=0&&dy<=0){
                    //ra in rb or rb in ra

                }
                else{
                    float dist=(cv::norm(dbr)+cv::norm(dtl))/2;
                    rectsDif+=dist;
                    if(rectsDif>distanceThreshold){
                        return std::numeric_limits<float>::max();
                    }
                }
            }
        }

        if(frameNumSame==0) return std::numeric_limits<float>::max();
        else return rectsDif/frameNumSame;
    }
    void calculateDistanceMat(cv::Mat &distMat);
};

#endif // OBJECTTRAJECTORYPROCESSING_H
