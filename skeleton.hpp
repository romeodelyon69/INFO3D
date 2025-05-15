#pragma once

#include "cgp/cgp.hpp"
#include "stdlib.h"
#include "joint.hpp"
#include "environment.hpp"
#include "kinematicChain.hpp"
#include "segment.hpp"


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


class Arm : public KinematicChain{
    public:
        Arm(float scale = 1.0f, bool isLeft = false);
        void initialize() override;

        cgp::vec3 getShoulderPos();
        void moveFromHandToShoulder(cgp::vec3 handTarget);
        void moveFromShoulderToHand(cgp::vec3 shoulderTarget);
        void moveHand(cgp::vec3 handTarget);
};

class Leg : public KinematicChain{
    public:
        Leg(float scale = 1.0f, bool isLeft = false);
        void initialize() override;

        cgp::vec3 getHipPos();
        void moveFromFootToHip(cgp::vec3 footTarget);
        void moveFromHipToFoot(cgp::vec3 hipTarget);
};

class IsoceleTriangle{
    public:
        cgp::vec3 a;
        cgp::vec3 b;
        cgp::vec3 c;        //supposed to be 0 all the time
        cgp::vec3 normal;
        float height; //height of the triangle
        float base; //length of the base
        cgp::vec3 positionRef; //technically this is the position of C in the real world
        cgp::vec3 heightDirection; //direction of the height from C to the middle of AB
        Segment segment;

        IsoceleTriangle();
        IsoceleTriangle(cgp::vec3 a, cgp::vec3 b, cgp::vec3 c);

        void initialize();
        void moveAandB(cgp::vec3 newA, cgp::vec3 newB);
        void moveC(cgp::vec3 translationC);
        void setC(cgp::vec3 newC);
        void translate(cgp::vec3 translation);
        void draw(environment_structure& environment);
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

        cgp::vec3 lumbarSpineDirection;
        cgp::vec3 thoracicSpineDirection;


        HumanSkeleton(float scale = 1.0f, cgp::vec3 position = cgp::vec3(0,0,0));

        void initialize();
        void draw(environment_structure& environment);

        void translate(cgp::vec3 translation);
        void movePelvis(cgp::vec3 translation);
        void setPelvisPos(cgp::vec3 position);
        void setPosLegs(cgp::vec3 leftFootPos, cgp::vec3 rightFootPos);
};