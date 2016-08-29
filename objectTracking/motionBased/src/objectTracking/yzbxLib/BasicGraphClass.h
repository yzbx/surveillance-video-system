#ifndef BASICGRAPHCLASS_H
#define BASICGRAPHCLASS_H

#include "yzbx_config.h"
#include "trackingobjectfeature.h"
#include <iostream>
#include <memory>
using namespace  std;
class BasicGraphPos{
public:
    BasicGraphPos(int frameNum=-1,int blob_id=-1){
        this->frameNum=frameNum;
        this->blob_id=blob_id;
    }
    int frameNum;
    int blob_id;
};

class BasicGraphNode{
public:
    BasicGraphNode(){

    }
    std::vector<int> track_ids;
    std::vector<std::pair<BasicGraphPos,track_t>> edges;
    BasicGraphPos graph_pos;
    std::shared_ptr<trackingObjectFeature> of;
    std::vector<track_t> pathWeights;
};

class BasicGraphClass
{
public:
    typedef vector<std::shared_ptr<BasicGraphNode>> Path;
    typedef vector<Path> PathsForOneNode;
    typedef vector<PathsForOneNode> PathsForAllNodes;
    BasicGraphClass();
    std::list<std::vector<BasicGraphNode>> graph;
    int NextTrackId=0;
    void getHeadNodes(vector<std::shared_ptr<BasicGraphNode>> &headNodes){
        assert(headNodes.empty());
        std::vector<BasicGraphNode> &nodes=graph.front();
        for(auto node=nodes.begin();node!=nodes.end();node++){
            headNodes.push_back(make_shared<BasicGraphNode>(*node));
        }
    }

    std::shared_ptr<BasicGraphNode> getGraphNode(BasicGraphPos pos){
        for(auto it=graph.begin();it!=graph.end();it++){
            for(auto node=it->begin();node!=it->end();node++){
                if(node->graph_pos.frameNum<pos.frameNum){
                    break;
                }
                else if(node->graph_pos.frameNum==pos.frameNum){
                    if(node->graph_pos.blob_id<pos.blob_id){
                        continue;
                    }
                    else if(node->graph_pos.blob_id==pos.blob_id){
                        //FIXME from iterator to shared_ptr
                        return std::make_shared<BasicGraphNode>(*node);
                    }
                    else{
                        qDebug()<<"blob_id out of range!!!";
                        assert(false);
                    }
                }
                else{
                    qDebug()<<"frameNum out of range!!!";
                    assert(false);
                }
            }
        }

        qDebug()<<"cannot find BasicGraphNode";
        assert(false);
        //use default constructor to return empty result;
        return std::make_shared<BasicGraphNode>();
    }

    void addFeatureVector(std::vector<trackingObjectFeature> &featureVector,int frameNum){

        int blob_id=0;
        std::vector<BasicGraphNode> nodeVector;
        for(auto it=featureVector.begin();it!=featureVector.end();it++){
            BasicGraphPos pos(frameNum,blob_id++);
            BasicGraphNode node;
            node.of=std::make_shared<trackingObjectFeature>(*it);
            node.graph_pos=pos;

            if(frameNum==0){
                node.track_ids.push_back(NextTrackId++);
            }

            nodeVector.push_back(node);
        }

        graph.push_back(nodeVector);
    }

    void maintainGraphSize(int maxLength){
        if(graph.size()>maxLength){
            graph.erase(graph.begin());
        }
    }

    int size(){
        return graph.size();
    }

    void getPaths(PathsForAllNodes &paths){
        //vector<std::shared_ptr<BasicGraphNode>> path
        //vector< vector<std::shared_ptr<BasicGraphNode>> > pathsForOneNode
        //vector< vector< vector<std::shared_ptr<BasicGraphNode>> >> pathsAllNodes
        assert(graph.size()>0);
        for(auto it=graph.begin()->begin();it!=graph.begin()->end();it++){
            std::shared_ptr<BasicGraphNode> node=make_shared<BasicGraphNode>(*it);

            PathsForOneNode subPaths;
            PathsForOneNode childSubPaths;
            getSubPaths(node,childSubPaths);
            for(auto path=childSubPaths.begin();path!=childSubPaths.end();path++){
                vector<std::shared_ptr<BasicGraphNode>> childPath;
                childPath.push_back(node);
                childPath.resize(1+path->size());
                std::copy(path->begin(),path->end(),childPath.begin()+1);
                subPaths.push_back(childPath);
            }

            paths.push_back(subPaths);
        }
    }

    void getSubPaths(const std::shared_ptr<BasicGraphNode> node,\
                     vector< vector<std::shared_ptr<BasicGraphNode>> > &subPaths){

        for(auto edge=node->edges.begin();\
                edge!=node->edges.end();\
                edge++)
        {
            std::shared_ptr<BasicGraphNode> p=getGraphNode(edge->first);
            PathsForOneNode childSubPaths;
            getSubPaths(p,childSubPaths);
            for(auto path=childSubPaths.begin();path!=childSubPaths.end();path++){
                vector<std::shared_ptr<BasicGraphNode>> childPath;
                childPath.push_back(p);
                childPath.resize(1+path->size());
                std::copy(path->begin(),path->end(),childPath.begin()+1);
                subPaths.push_back(childPath);
            }
        }
    }

};

#endif // BASICGRAPHCLASS_H
