#include "trackingObjectAssociation.h"

TrackingObjectAssociation::TrackingObjectAssociation()
{

}

void TrackingObjectAssociation::process(TrackingStatus *status)
{
    DirectedGraph g;
    float missCost=0.7;
    int vertexId=0;

    std::list<FrameFeature>::iterator it=status->frameFeatureList.begin();
    std::list<FrameFeature>::iterator next=std::next(it,1);
    for (; next != status->frameFeatureList.end(); ++it,++next){
        adjecentObjectAssociation(it,next,g,vertexId);
        vertexId+=it->features.size();
    }

    qDebug()<<"dump object association graph g: ";
    dumpObjectAssociationGraph(g);
//    boost::write_graphviz("/home/yzbx/output/tracking.dot",g);
}

void TrackingObjectAssociation::adjecentObjectAssociation(FFIter it, FFIter next,DirectedGraph &g, int vertexId)
{
    int m=it->features.size();
    int n=next->features.size();
    for(int i=0;i<it->features.size();i++){

        vector<float> weights;
        for(int j=0;j<next->features.size();j++){
            float weight=getObjectAssociationWeight(it->features[i],next->features[j]);
            weights.push_back(weight);
        }

        vector<bool> filter=weightFilter(weights);
        for(int j=0;j<filter.size();j++){
            if(filter[j]){
                boost::add_edge(vertexId+i,vertexId+m+j,weights[j],g);
            }
        }
    }
}

vector<bool> TrackingObjectAssociation::weightFilter(vector<float> weights){
    //1. min~1.2min
    vector<float> sortedWeights;
    std::copy(weights.begin(),weights.end(),sortedWeights.begin());
    std::sort(weights.begin(),weights.end());

    vector<float>::iterator it;
    std::cout<<"weights: ";
    for(it=weights.begin();it!=weights.end();it++){
        std::cout<<*it;
    }
    std::cout<<std::endl;
    std::cout<<"sortedWeights: ";
    for(it=sortedWeights.begin();it!=sortedWeights.end();it++){
        std::cout<<*it;
    }
    std::cout<<std::endl;

    const int maxEdge=2;
    const float maxRatio=1.2;
    const float maxAssociation=std::numeric_limits<float>::max();
    float weightFilterThreshold=0;
    int i=0;
    for(i=0,it=sortedWeights.begin();it!=sortedWeights.end()&&i<maxEdge;it++,i++){
        if(i==0) weightFilterThreshold=*it;
        else if(*it<weightFilterThreshold*maxRatio){
            weightFilterThreshold=*it;
        }
        else{
            break;
        }
    }

    weightFilterThreshold=std::min(maxAssociation,weightFilterThreshold);

    vector<bool> filter;
    for(it=weights.begin();it!=weights.end();it++){
        if(*it<=weightFilterThreshold){
            filter.push_back(true);
        }
        else{
            filter.push_back(false);
        }
    }

    return filter;
}

float TrackingObjectAssociation::getObjectAssociationWeight(ObjectFeature a,ObjectFeature b){
    float dx=(a.pos.x-b.pos.x)/a.size;
    float dy=(a.pos.y-b.pos.y)/a.size;
    float posDist=sqrt(dx*dx+dy*dy);
    float sizeDist=max(a.size/b.size,b.size/a.size)-1;

    float alpha=0.5;
    float weight=alpha*posDist+(1-alpha)*sizeDist;

    return weight;
}

void TrackingObjectAssociation::dumpObjectAssociationGraph(DirectedGraph &g)
{
    boost::property_map<DirectedGraph, boost::edge_weight_t>::type EdgeWeightMap = get(boost::edge_weight_t(), g);
    std::pair<edge_iterator, edge_iterator> ei = edges(g);

    std::cout << "Number of edges = " << num_edges(g) << "\n";
    std::cout << "Edge list:\n";

    for (edge_iterator it = ei.first; it != ei.second; ++it )
    {
        std::cout << *it <<"[label=\""<<EdgeWeightMap(*it)<<"\"]"<< std::endl;
    }


    std::cout << std::endl;
}
