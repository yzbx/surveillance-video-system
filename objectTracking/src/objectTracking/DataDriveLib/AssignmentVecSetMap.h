#ifndef ASSIGNMENTVECSETMAP_H
#define ASSIGNMENTVECSETMAP_H
#include <iostream>
#include <map>
#include <vector>
#include <set>

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
    std::set<Index_t> unmatchedASet;
    std::set<Index_t> unmatchedBSet;

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
        unmatchedASet.clear();
        unmatchedBSet.clear();
        ZeroToOneSet.clear();
        OneToZeroSet.clear();
        OneToOneMap.clear();
        OneToNMap.clear();
        NToOneMap.clear();
        NToNMap.clear();
    }
};

#endif // ASSIGNMENTVECSETMAP_H
