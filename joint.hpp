#pragma once

#include "cgp/cgp.hpp"
#include "stdlib.h"
#include "kinematicChain.hpp"


using namespace cgp;

class HumanSkeleton;
class Bone;

class Joint{
    public:
        std::string name;
        Bone *boneFather;
        Bone *boneChild;
        //constructor
        Joint(std::string name, Bone* boneFather, Bone* boneChild);
        virtual void applyConstraintOnFather();
        virtual void applyConstraintOnChild();
};


class ParentRotule : public Joint {
    public:
        // Constructor 
        ParentRotule(std::string name, Bone* boneFather, Bone* boneChild, float maxAngle);
        
        float maxAngle; //in radian
        void setAngle(float maxAngle);
        void applyConstraintOnFather() override;
        void applyConstraintOnChild() override;
};

class Hinge : public Joint {
    public:
        // Constructor 
        Hinge(std::string name, Bone* boneFather, Bone* boneChild, float maxAngle);
        
        float maxAngle; //in radian
        void setAngle(float maxAngle);
        void applyConstraintOnFather() override;
        void applyConstraintOnChild() override;
};

