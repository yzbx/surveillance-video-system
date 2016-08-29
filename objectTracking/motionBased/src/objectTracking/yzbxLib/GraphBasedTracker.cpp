#include "GraphBasedTracker.h"

void GraphBasedTracker::tracking(const Mat &img_input, const Mat &img_fg)
{
    assert(!img_input.empty());
    assert(!img_fg.empty());
    imageList.push_back(std::make_pair(img_input,img_fg));
    if(imageList.size()>maxlistLength){
        imageList.pop_front();
    }
    this->img_input=img_input;
    this->img_fg=img_fg;
    frameNum++;
    start();
}

void GraphBasedTracker::run()
{
    std::vector<trackingObjectFeature> featureVector;
    blobDetector.getBlobFeature(img_fg,img_fg,featureVector);
    featureGraph.addFeatureVector(featureVector,frameNum);
    featureGraph.maintainGraphSize(maxlistLength);

    assert(imageList.size()==featureGraph.size());
    //if img_input donot contain foreground, empty featureVector need be allowed
    //deal with the miss/disappear
    if(featureGraph.size()>=2){
        GraphBasedTracking();
        showing(img_input,img_fg,featureVector);
    }
    else{
        assert(frameNum==0);
    }
}

void GraphBasedTracker::GraphBasedTracking(){
    assert(featureGraph.size()>=2);
    updateGraphAssociation();
    vector<Path> maxWeightPaths=findMaximumWeightedPaths();
    updateTrajectory(maxWeightPaths);
}

void GraphBasedTracker::updateGraphAssociation(){
    assert(featureGraph.size()>=2);

//    std::list<std::vector<BasicGraphNode>> graph;
    //edge fv2-->fv1
    auto fvEnd=featureGraph.graph.end();
    auto fv2=std::prev(fvEnd,2);
    auto fv1=std::prev(fvEnd,1);

    //edge node2-->node1
    for(auto node2=fv2->begin();node2!=fv2->end();node2++){
        //find two biggest weight
        vector<track_t> weights;
        vector<BasicGraphPos> posVector;
        for(auto node1=fv1->begin();node1!=fv1->end();node1++){
            track_t weight=GetDistance(node1->of,node2->of);
            weights.push_back(weight);
            posVector.push_back(node1->graph_pos);
        }

        if(weights.size()<2){
            if(weights[0]<dist_thres){
                //std::vector<std::pair<BasicGraphPos,track_t>> edges;
                auto edge=std::make_pair(posVector[0],weights[0]);
                node2->edges.push_back(edge);
            }
        }
        else{
            vector<track_t> sortedWeights(weights.size());
            std::copy(weights.begin(),weights.end(),sortedWeights.begin());
            std::sort(sortedWeights.begin(),sortedWeights.end());

            if(sortedWeights[0]<dist_thres){
                int index=std::find(weights.begin(),weights.end(),sortedWeights[0])-weights.begin();
                assert(index<posVector.size());
                auto edge=std::make_pair(posVector[index],sortedWeights[0]);
                node2->edges.push_back(edge);
            }
            if(sortedWeights[1]<dist_thres){
                int index=std::find(weights.begin(),weights.end(),sortedWeights[1])-weights.begin();
                assert(index<posVector.size());
                auto edge=std::make_pair(posVector[index],sortedWeights[1]);
                node2->edges.push_back(edge);
            }
        }
    }
}

track_t GraphBasedTracker::GetDistance(shared_ptr<trackingObjectFeature> of1,shared_ptr<trackingObjectFeature> of2){
    Rect_t rect1=of1->rect;
    Rect_t rect2=of2->rect;
    track_t dist=std::min(cv::norm(rect1.tl()-rect2.tl()),cv::norm(rect1.br()-rect2.br()));
    return dist;
}

vector<vector<std::shared_ptr<BasicGraphNode>>> GraphBasedTracker::findMaximumWeightedPaths(){
    assert(featureGraph.size()>=2);
    //generate paths
    PathsForAllNodes paths;
    featureGraph.getPaths(paths);

    //dump paths
    QString filterName="paths.txt";
    QFile data(filterName);
    if (!data.open(QFile::WriteOnly|QFile::Truncate)) {
        qDebug()<<"cannot open file "<<filterName;
        assert(false);
    }
    QTextStream out(&data);
    for(auto nodePaths=paths.begin();nodePaths!=paths.end();nodePaths++){
        int track_id=nodePaths-paths.begin();
        out<<"################################ track_id="<<track_id<<"\n";
        for(auto path=nodePaths->begin();path!=nodePaths->end();path++){
            for(auto node=path->begin();node!=path->end();node++){
                if(node!=path->begin()) out<<"->";
                out<<"["<<(*node)->graph_pos.frameNum<<","<<(*node)->graph_pos.blob_id<<"]";
            }
            out<<"\n";
        }
    }
    data.close();

    //computer path weight for each node. [in fact ,mainly do local feature match]
    vector<shared_ptr<BasicGraphNode>> headNodes;
    featureGraph.getHeadNodes(headNodes);
    int index=0;
    for(auto nodesAtSameTime=featureGraph.graph.begin();nodesAtSameTime!=featureGraph.graph.end();nodesAtSameTime++){
        auto next=std::next(nodesAtSameTime,1);
        index++;
        if(next==featureGraph.graph.end()){
            break;
        }

        //headNode: shared_ptr<BasicGraphNode>
        //tailNode: vector<BasicGraphNode>::iterator
        ObjectLocalFeatureMatch matcher;
        auto headImagePair=imageList.begin();
        auto currentImagePair=std::next(imageList.begin(),index);
        matcher.getGoodMatches(headImagePair->first,headImagePair->second,currentImagePair->first,currentImagePair->second);
        for(auto tailNode=next->begin();tailNode!=next->end();tailNode++){
            vector<track_t> pathWeights;
            for(auto headNode=headNodes.begin();headNode!=headNodes.end();headNode++){
                track_t pathWeight=calcPathWeight((*headNode)->of,tailNode->of,matcher);
                pathWeights.push_back(pathWeight);
            }
            tailNode->pathWeights.clear();
            std::swap(tailNode->pathWeights,pathWeights);
            assert(pathWeights.empty());
        }
    }


    //find max weight path;
    vector<Path> maxWeightPaths;
    for(auto nodePaths=paths.begin();nodePaths!=paths.end();nodePaths++){
        int startNodeIndex=nodePaths-paths.begin();
        vector<track_t> pathWeights;
        for(auto path=nodePaths->begin();path!=nodePaths->end();path++){
            track_t weight=0;
            for(auto node=path->begin();node!=path->begin();node++){
                if(node!=path->begin()){
                    weight+=(*node)->pathWeights[startNodeIndex];
                }
            }
            pathWeights.push_back(weight);
        }
        auto maxIt=std::max_element(pathWeights.begin(),pathWeights.end());
        maxWeightPaths.push_back((*nodePaths)[maxIt-pathWeights.begin()]);
        assert(maxWeightPaths.back().size()==(*nodePaths)[maxIt-pathWeights.begin()].size());
    }

    return maxWeightPaths;
}

inline bool isPointInRect(cv::Point2f p, Rect_t rect)
{
    if(p.x>rect.x&&p.x>rect.y&&p.x<rect.x+rect.width&&p.y<rect.y+rect.height){
        return true;
    }
    else{
        return false;
    }
}

track_t GraphBasedTracker::calcPathWeight(std::shared_ptr<trackingObjectFeature> of1,std::shared_ptr<trackingObjectFeature> of2,ObjectLocalFeatureMatch &matcher){
    track_t pathWeight=0.0;
    vector<DMatch> &good_matches=matcher.global_good_matches;
    for(auto match=good_matches.begin();match!=good_matches.end();match++){
        assert(match->queryIdx<matcher.global_keypoints_1.size());
        assert(match->trainIdx<matcher.global_keypoints_2.size());
        KeyPoint &kp1=matcher.global_keypoints_1[match->queryIdx];
        KeyPoint &kp2=matcher.global_keypoints_2[match->trainIdx];

        //test KeyPoint's position in trackingObjectFeature's rect
        if(isPointInRect(kp1.pt,of1->rect)&&isPointInRect(kp2.pt,of2->rect)){
            pathWeight+=1.0;
        }
    }
    return pathWeight;
}

void GraphBasedTracker::updateTrajectory(vector<Path> &maxWeightPaths){
    //update all the node in path according to headNode if the headNode has only one track_id/multiple track_id
    //I have to do split or new track_id for second node vector.
    //I have to record split, merge information for trajectory analysis.

    assert(featureGraph.size()>=2);
    //use maxWeightPath assign track_id
    for(auto path=maxWeightPaths.begin();path!=maxWeightPaths.end();path++){
        shared_ptr<BasicGraphNode> headNode;

        //add headNode's ids to path's ids without repetition
        for(auto node=path->begin();node!=path->end();node++){
            if(node==path->begin()){
                headNode=*node;
            }
            else{
                //add headNode's ids to nextIds's ids without repetition
                vector<int> &ids=headNode->track_ids;
                vector<int> &nextIds=(*node)->track_ids;
                for(auto id=ids.begin();id!=ids.end();id++){
                    if(std::find(nextIds.begin(),nextIds.end(),*id)==nextIds.end()){
                        nextIds.push_back(*id);
                    }
                }
            }
        }
    }

    //find empty ids in next head node, use something to decide new or split
    auto nextHeadNodes=std::next(featureGraph.graph.begin(),1);
    for(auto nextHeadNode=nextHeadNodes->begin();nextHeadNode!=nextHeadNodes->end();nextHeadNode++){
        if(nextHeadNode->track_ids.empty()){
            //splitFlag=true, split it; else new it.
            bool splitFlag=splitOrNewTrackId(make_shared<BasicGraphNode>(*nextHeadNode));
            //increase track_id
            int track_id=featureGraph.NextTrackId-1;
            if(splitFlag){
                qDebug()<<"split object(track_id="<<track_id<<")";
            }
            else{
                qDebug()<<"new object(track_id="<<track_id<<")";
            }
        }
    }
}

bool GraphBasedTracker::splitOrNewTrackId(pNode nextHeadNode){
    BasicGraphPos &nextPos=nextHeadNode->graph_pos;
    auto headNodes=featureGraph.graph.begin();
    for(auto headNode=headNodes->begin();headNode!=headNodes->end();headNode++){
        int index=headNode-headNodes->begin();
        std::vector<std::pair<BasicGraphPos,track_t>> &edges=headNode->edges;
        for(auto edge=edges.begin();edge!=edges.end();edge++){
            BasicGraphPos pos=edge->first;
            assert(pos.frameNum==nextPos.frameNum);
            //if have edge in pos range, then check pathWeight
            if(pos.blob_id==nextPos.blob_id){
                if(nextHeadNode->pathWeights[index]>split_threshold){
                    int track_id=featureGraph.NextTrackId;
                    nextHeadNode->track_ids.push_back(featureGraph.NextTrackId++);
                    assert(featureGraph.NextTrackId==track_id+1);

                    dumpSplitInformation(make_shared<BasicGraphNode>(*headNode),nextHeadNode);
                    return true;
                }
            }
        }
    }

    int track_id=featureGraph.NextTrackId;
    nextHeadNode->track_ids.push_back(featureGraph.NextTrackId++);
    assert(track_id+1==featureGraph.NextTrackId);
    return false;
}

void GraphBasedTracker::dumpSplitInformation(pNode headNode,pNode nextNode){

}

