#ifndef ASSIGNMENTVECSETMAP_H
#define ASSIGNMENTVECSETMAP_H
#include <iostream>
#include <map>
#include <vector>
#include <set>
#include <assert.h>

using namespace std;
using Index_t=unsigned int;
class AssignmentVecSetMap
{
public:
    AssignmentVecSetMap(std::string a,std::string b);
    std::string nameA;
    std::string nameB;
    std::map<Index_t,std::set<Index_t>> AToB;
    std::map<Index_t,std::set<Index_t>> BToA;
    std::set<Index_t> matchedASet;
    std::set<Index_t> matchedBSet;

    ///Hungarian Assignment
//    std::set<Index_t> unmatchedASet;
//    std::set<Index_t> unmatchedBSet;

    ///Split and Merge
    std::set<Index_t> ZeroToOneSet;	 // 0-1
    std::set<Index_t> OneToZeroSet; // 1-0
    std::map<Index_t, Index_t> OneToOneMap; //1-1
    std::map<Index_t, std::set<Index_t>> OneToNMap; //1-N
    std::map<Index_t,std::set<Index_t>> NToOneMap; //N-1
    std::map<Index_t,std::set<Index_t>> NToNMap;

    void clear(){
        AToB.clear();
        BToA.clear();
        matchedASet.clear();
        matchedBSet.clear();
//        unmatchedASet.clear();
//        unmatchedBSet.clear();
        ZeroToOneSet.clear();
        OneToZeroSet.clear();
        OneToOneMap.clear();
        OneToNMap.clear();
        NToOneMap.clear();
        NToNMap.clear();
    }

    void dumpMatch(){
        cout<<"AssignmentVecSetMap::dumpMatch: "<<nameA<<"->"<<nameB<<endl;
        for(auto it=AToB.begin();it!=AToB.end();it++){
            Index_t a=it->first;
            set<Index_t> &set=it->second;
            cout<<a<<"->";
            for(auto it=set.begin();it!=set.end();it++){
                cout<<*it<<",";
            }
            cout<<endl;
        }
        cout<<nameB<<"->"<<nameA<<endl;
        for(auto it=BToA.begin();it!=BToA.end();it++){
            Index_t a=it->first;
            set<Index_t> &set=it->second;
            cout<<a<<"->";
            for(auto it=set.begin();it!=set.end();it++){
                cout<<*it<<",";
            }
            cout<<endl;
        }
    }

    void dumpReDetection(){
        cout<<"AssignmentVecSetMap::dumpReDetection: "<<nameA<<"->"<<nameB<<endl;
        cout<<"0-1(idx): "<<endl;
        for(auto it=ZeroToOneSet.begin();it!=ZeroToOneSet.end();it++){
            cout<<*it<<",";
        }
        cout<<endl;

        cout<<"1-0(idx): "<<endl;
        for(auto it=OneToZeroSet.begin();it!=OneToZeroSet.end();it++){
            cout<<*it<<",";
        }
        cout<<endl;
    }

    void dumpAll(){
        cout<<"AssignmentVecSetMap::dumpAll *****************{"<<endl;
        dumpMatch();
        cout<<"AssignmentVecSetMap::matched "<<nameA<<endl;
        for(auto it=matchedASet.begin();it!=matchedASet.end();it++){
            cout<<*it<<",";
        }
        cout<<endl;
        cout<<"AssignmentVecSetMap::matched "<<nameB<<endl;
        for(auto it=matchedBSet.begin();it!=matchedBSet.end();it++){
            cout<<*it<<",";
        }
        cout<<endl;
        cout<<"AssignmentVecSetMap::ZeroToOne "<<nameB<<endl;
        for(auto it=ZeroToOneSet.begin();it!=ZeroToOneSet.end();it++){
            cout<<*it<<",";
        }
        cout<<endl;
        cout<<"AssignmentVecSetMap::OneToZero "<<nameA<<endl;
        for(auto it=OneToZeroSet.begin();it!=OneToZeroSet.end();it++){
            cout<<*it<<",";
        }
        cout<<endl;

        dumpReDetection();
        cout<<"AssignmentVecSetMap::OneToOne "<<nameA<<"->"<<nameB<<endl;
        for(auto it=OneToOneMap.begin();it!=OneToOneMap.end();it++){
            cout<<it->first<<"->"<<it->second<<",";
        }
        cout<<endl;

        cout<<"AssignmentVecSetMap::OneToN "<<nameA<<"->"<<nameB<<endl;
        for(auto it=OneToNMap.begin();it!=OneToNMap.end();it++){
            cout<<it->first<<"->"<<"(";
            const auto &nset=it->second;
            for(auto it=nset.begin();it!=nset.end();it++){
                cout<<*it<<",";
            }
            cout<<")"<<endl;
        }
        cout<<endl;
        cout<<"AssignmentVecSetMap::NToOne "<<nameA<<"->"<<nameB<<endl;
        for(auto it=OneToNMap.begin();it!=OneToNMap.end();it++){
            cout<<"(";
            const auto &nset=it->second;
            for(auto it=nset.begin();it!=nset.end();it++){
                cout<<*it<<",";
            }
            cout<<")->"<<it->first<<endl;
        }
        cout<<endl;
        cout<<"AssignmentVecSetMap::NToN "<<nameA<<"->"<<nameB<<endl;
        for(auto it=OneToNMap.begin();it!=OneToNMap.end();it++){
            cout<<it->first<<"->"<<"(";
            const auto &nset=it->second;
            for(auto it=nset.begin();it!=nset.end();it++){
                cout<<*it<<",";
            }
            cout<<")"<<endl;
        }
        cout<<endl;

        cout<<"AssignmentVecSetMap::dumpAll *****************}"<<endl;
    }

    void checkAssignment(){
//        assert(b2b.unmatchedASet.empty());
//        assert(b2b.unmatchedBSet.empty());
        assert(OneToNMap.empty());
        assert(NToNMap.empty());
        assert(NToOneMap.empty());
        assert(OneToOneMap.empty());
        assert(OneToZeroSet.empty());
        assert(ZeroToOneSet.empty());
    }
};

#endif // ASSIGNMENTVECSETMAP_H
