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
        
        //pour le moment chaque os n'a qu'un seul parent direct et qu'un seul enfant direct
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

        void translate(cgp::vec3 translation);

        void setEndEffectorWorldPos(cgp::vec3 target);
        void setEndEffectorRelativePos(cgp::vec3 target);
        void moveEndEffector(cgp::vec3 move);
        void setStartEffector(cgp::vec3 target); 
        
        Bone* getBone(std::string name);
        
        void fabrik(vec3 target, float tolerance = 0.01f, int max_iter = 50);
        void fabrik(std::vector<Bone*> bonesOrder, vec3 target, float tolerance = 0.01f, int max_iter = 50);
        void fabrik(Bone* targetBone, Bone* fixedBone, vec3 target, float tolerance = 0.01f, int max_iter = 50);
        void fabrik(std::string targetBoneName, std::string fixedBoneName, vec3 target, float tolerance = 0.01f, int max_iter = 50);

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

class Arm : public KinematicChain{
    public:
        Arm(float scale = 1.0f, bool isLeft = false);
        void initialize() override;

        cgp::vec3 getShoulderPos();
        void moveFromHandToShoulder(cgp::vec3 handTarget);
};

class Leg : public KinematicChain{
    public:
        Leg(float scale = 1.0f, bool isLeft = false);
        void initialize() override;

        cgp::vec3 getHipPos();
        void moveFromFootToHip(cgp::vec3 footTarget);
};

class IsoceleTriangle{
    public:
        cgp::vec3 a;
        cgp::vec3 b;
        cgp::vec3 c;
        cgp::vec3 normal;
        float height; //height of the triangle
        cgp::vec3 positionRef; //technically this is the position of C in the real world

        IsoceleTriangle(cgp::vec3 a, cgp::vec3 b, cgp::vec3 c): a(a), b(b), c(c){
            normal = cgp::cross(b - a, c - a);
            normal = normal / cgp::norm(normal);
            height = cgp::norm(c - (a+b)/2);
        }

        void moveAandB(cgp::vec3 newA, cgp::vec3 newB);
        void moveC(cgp::vec3 newC);
        void setC(cgp::vec3 newC);
};

class HumanSkeleton {
    float scale;
    public:
        Arm leftArm;
        Arm rightArm;
        Leg leftLeg;
        Leg rightLeg;
        IsoceleTriangle torso;
        IsoceleTriangle pelvis;

        cgp::vec3 position;

        HumanSkeleton(float scale = 1.0f, cgp::vec3 position = cgp::vec3(0,0,0));

        void initialize();
        void draw(environment_structure& environment);

        void translate(cgp::vec3 translation);
        void movePelvis(cgp::vec3 translation);
        void setPelvisPos(cgp::vec3 position);
        void setPosLegs(cgp::vec3 leftFootPos, cgp::vec3 rightFootPos);
};