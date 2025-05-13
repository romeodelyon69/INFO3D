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
        virtual void applyConstraintOnFather(HumanSkeleton *skeleton);
        virtual void applyConstraintOnChild(HumanSkeleton *skeleton);
};

class GeneralRotule : public Joint{
    public:
        vec3 axis;      //the axis of the cone of admission
        float maxAngle; //the maximal angle of the cone of admission

        // Constructor explicitly initializes the base class Joint
        GeneralRotule(std::string name, Bone* boneFather, Bone* boneChild, vec3 axis, float maxAngle);

        void set(vec3 axis, float maxAngle);
        virtual void applyConstraintOnFather(HumanSkeleton *skeleton) override;
        virtual void applyConstraintOnChild(HumanSkeleton *skeleton) override;
};

class ParentRotule : public GeneralRotule {
    public:
        // Constructor 
        ParentRotule(std::string name, Bone* boneFather, Bone* boneChild, float maxAngle);
        
        void setAngle(float maxAngle);
        void update();
        void applyConstraintOnFather(HumanSkeleton *skeleton) override;
        void applyConstraintOnChild(HumanSkeleton *skeleton) override;
};

