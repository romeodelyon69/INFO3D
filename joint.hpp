#pragma once

#include "cgp/cgp.hpp"
#include "stdlib.h"
#include "skeleton.hpp"

using namespace cgp;

class Bone;
class Skeleton;

class Joint{
    public:
        std::string name;
        Bone *boneFather;
        Bone *boneChild;
        //constructor
        Joint(std::string name, Bone* boneFather, Bone* boneChild)
        : name(name), boneFather(boneFather), boneChild(boneChild) {}
        virtual void applyConstraint(Skeleton *skeleton);
};

class Rotule : public Joint{
    void applyConstraint(Skeleton *skeleton) override;
};

