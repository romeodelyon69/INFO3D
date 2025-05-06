#pragma once

#include "cgp/cgp.hpp"
#include "stdlib.h"
#include "joint.hpp"
#include "environment.hpp"


//human propotion
#define HEAD 0.13f
#define NECK 0.05f
#define TORSO 0.29f
#define SHOULDER 0.1f
#define UPPER_ARM 0.188f
#define FOREARM 0.145f
#define HAND 0.1f
#define HIP 0.12f
#define THIGH 0.2f
#define TIBIA 0.24f
#define FOOT 0.1f
#define LEGANGLE 0.1f
#define ARMANGLE 0.1f
#define FEETSPACING 0.25f



class Joint;

class Bone{
    public:
        std::string name;
        cgp::vec3 start;
        cgp::vec3 end;
        cgp::vec3 direction;
        
        float length;
        //constructor
        Bone(std::string name, cgp::vec3 start, cgp::vec3 end): name(name), start(start), end(end){
            direction = end - start;
            length = cgp::norm(direction);
        };
        cgp::curve_drawable bone;

        void initialize();

        void draw(environment_structure& environment);
        
        //pour le moment chaque os n'a qu'un seul parent direct et qu'un seul enfant direct
        Joint *jointFather = nullptr;
        Joint *jointChild = nullptr;

        void setStart(cgp::vec3 start);
        void setEnd(cgp::vec3 end);

        
};

class Skeleton{
    public:
         //on preset une taille de 32 pour les vecteurs pour éviter de faire des reallocations
        std::vector<Bone> bones;
        std::vector<Joint*> joints;

        Skeleton() {
            // Initialize the bones and joints vectors with a size of 32
            bones.reserve(32);
            joints.reserve(32);
        }
       
        //a sphere to display the articulation
        cgp::mesh_drawable articulation;
        
        void addBone(Bone bone);
        void addJoint(Joint* joint);

        virtual void initialize();

        void draw(environment_structure& environment);

        // Apply the FABRIK algorithm to the skeleton
        // the function will take to bones, the target position and the tolerance
        //it will try to move the end of the first bone to the target position 
        //and will not move the start of the second
        void forwardFabrik(Bone* targetBone, Bone* fixedBone, vec3 target, float tolerance = 0.01f);
        void forwardApplyConstraints(Bone* targetBone, Bone* fixedBone);
        void backwardFabrik(Bone* targetBone, Bone* fixedBone, vec3 target, float tolerance = 0.01f);
        void backwardApplyConstraints(Bone* targetBone, Bone* fixedBone);
        
        void fabrik(Bone* targetBone, Bone* fixedBone, vec3 target, float tolerance = 0.01f, int max_iter = 10);
        void fabrik(std::string targetBoneName, std::string fixedBoneName, vec3 target, float tolerance = 0.01f, int max_iter = 10);
        
        Bone* getBone(std::string name);
};

class HumanSkeleton : public Skeleton {
public:
    float scale;
    void initialize() override;
    void addJoint(std::string jointName, std::string boneFatherName, std::string boneChildName);
    void addParentRotule(std::string jointName, std::string boneFatherName, std::string boneChildName, float maxAngle);
    HumanSkeleton() : scale(1.0f) {} // Constructeur par défaut avec une valeur par défaut pour scale
    HumanSkeleton(float scale) : scale(scale) {}

};