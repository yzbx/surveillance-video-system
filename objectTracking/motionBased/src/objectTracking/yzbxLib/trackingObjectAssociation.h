#ifndef TRACKINGOBJECTASSOCIATION_H
#define TRACKINGOBJECTASSOCIATION_H
#include "trackingStatus.h"
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
typedef boost::property<boost::edge_weight_t, int> EdgeWeightProperty;
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::directedS, boost::no_property, EdgeWeightProperty > DirectedGraph;
typedef boost::graph_traits<DirectedGraph>::edge_iterator edge_iterator;
typedef std::list<FrameFeature>::iterator FFIter;

class TrackingObjectAssociation
{
public:
    TrackingObjectAssociation();
    void process(TrackingStatus *status);
    void adjecentObjectAssociation(FFIter it, FFIter next, DirectedGraph &g, int vertexId);
    void dumpObjectAssociationGraph(DirectedGraph &g);
    float getObjectAssociationWeight(ObjectFeature a, ObjectFeature b);
    vector<bool> weightFilter(vector<float> weights);
};

#endif // TRACKINGOBJECTASSOCIATION_H
