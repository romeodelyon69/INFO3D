#pragma once

#include "cgp/cgp.hpp"
#include "stdlib.h"
#include "joint.hpp"
#include "environment.hpp"


class Joint;
class HumanSkeleton;

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
            direction = (end - start) / length;
        };
        cgp::curve_drawable bone;

        void initialize();

        void draw(environment_structure& environment, cgp::vec3 poseReference = cgp::vec3(0,0,0));
        
        //pour le moment chaque os n'a qu'un seul parent direct et qu'un seul enfant direct (si il y en a un)
        Joint *jointFather = nullptr;
        Joint *jointChild = nullptr;

        void setStart(cgp::vec3 start);
        void setEnd(cgp::vec3 end);
        void translate(cgp::vec3 translation);
};

class KinematicChain{
    public:
         
        std::vector<Bone*> bones;
        std::vector<Joint*> joints;

        KinematicChain() {
            //on preset une taille de 32 pour les vecteurs pour éviter de faire des reallocations
            // Initialize the bones and joints vectors with a size of 128
            bones.reserve(128);
            joints.reserve(128);
        }

        cgp::vec3 position; //the position of the kinematic chain in the world
        cgp::vec3 endEffectorPos;

        HumanSkeleton *skeleton = nullptr;
        
        //a sphere to display the articulation
        cgp::mesh_drawable articulation;
        
        void addBone(Bone* bone);
        void addJoint(Joint* joint);

        virtual void initialize();

        void draw(environment_structure& environment);

        void setReferencePos(cgp::vec3 position);
        void translate(cgp::vec3 translation);
        bool translateFromEndEffectorToBone(std::string name, cgp::vec3 translation);
        void translateBones(cgp::vec3 translation);

        void setEndEffectorWorldPos(cgp::vec3 target);
        void setEndEffectorRelativePos(cgp::vec3 target);
        void moveEndEffector(cgp::vec3 move);
        void setStartEffector(cgp::vec3 target); 
        

        void setBoneStartWorldPos(std::string name, cgp::vec3 target);
        void setBoneStartRelativePos(std::string name, cgp::vec3 target);
        
        Bone* getBone(std::string name);
        Joint* getJoint(std::string name);
        
        void fabrik(vec3 target, float tolerance = 0.01f, int max_iter = 500);
        void fabrik(std::vector<Bone*> bonesOrder, vec3 target, float tolerance = 0.01f, int max_iter = 500);
        void fabrik(Bone* targetBone, Bone* fixedBone, vec3 target, float tolerance = 0.01f, int max_iter = 500);
        void fabrik(std::string targetBoneName, std::string fixedBoneName, vec3 target, float tolerance = 0.01f, int max_iter = 500);

        void fabrikAscendingTree(std::vector<Bone*> bonesOrder, vec3 target, float tolerance = 0.01f, int max_iter = 500);
        void fabrikAscendingTree(vec3 target, float tolerance = 0.01f, int max_iter = 500);
        void fabrikAscendingTree(std::string targetBoneName, std::string fixedBoneName, vec3 target, float tolerance = 0.01f, int max_iter = 500);
        void fabrikAscendingTree(Bone* targetBone, Bone* fixedBone, vec3 target, float tolerance = 0.01f, int max_iter = 500);

    private:
        // Apply the FABRIK algorithm to the KinematicChain
        // the function will take to bones, the target position and the tolerance
        //it will try to move the end of the first bone to the target position 
        //and will not move the start of the second
        void forwardFabrik(std::vector<Bone*> bonesOrder, vec3 target, float tolerance = 0.01f);
        void forwardApplyConstraints(std::vector<Bone*> bonesOrder);
        void backwardFabrik(std::vector<Bone*> bonesOrder, vec3 target, float tolerance = 0.01f);
        void backwardApplyConstraints(std::vector<Bone*> bonesOrder);
};