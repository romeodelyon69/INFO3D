#include "skeleton.hpp"
#include "Nmath.hpp"
#include <cmath>

using namespace cgp;



Arm::Arm(float scale, bool isLeft)
{
    if (isLeft) {
        scale = -scale;
    }
    // Initialize the arm with bones and joints
    float ribLength = std::abs(scale)* 0.27f;
    float bicepLength = std::abs(scale) * 0.188f;
    float forearmLength = std::abs(scale) * 0.145f;
    float handLength = std::abs(scale) * 0.1f;


    vec3 navelPos = cgp::vec3(0, 0, 0);
    vec3 shoulderPos = cgp::vec3(0, ribLength, 0);
    vec3 elbowPos = cgp::vec3(0, (ribLength + bicepLength), 0);
    vec3 wristPos = cgp::vec3( 0, (ribLength + bicepLength + forearmLength), 0);
    vec3 handPos = cgp::vec3(0, (ribLength + bicepLength + forearmLength + handLength), 0);

    cgp::vec3 x = cgp::vec3(1, 0, 0);
    cgp::vec3 y = cgp::vec3(0, 1, 0);
    cgp::vec3 z = cgp::vec3(0, 0, 1);

    cgp::frame ribFrame;
    cgp::frame bicepFrame;
    cgp::frame forearmFrame;
    cgp::frame handFrame;


    if (isLeft) {
        ribFrame = cgp::frame(y, z, navelPos);
        bicepFrame = cgp::frame(y, z, shoulderPos);
        forearmFrame = cgp::frame(y, z, elbowPos);
        handFrame = cgp::frame(y, z, wristPos);
    }
    else {
        ribFrame = cgp::frame(y, z, navelPos);
        bicepFrame = cgp::frame(y, z, shoulderPos);
        forearmFrame = cgp::frame(y, z, elbowPos);
        handFrame = cgp::frame(y, z, wristPos);
    }

    //print rib frame
    std::cout << "rib frame : " << ribFrame.position << std::endl;
    std::cout << "rib ux : " << ribFrame.ux() << std::endl;
    std::cout << "rib uy : " << ribFrame.uy() << std::endl;
    std::cout << "rib uz : " << ribFrame.uz() << std::endl;

    std::cout << "bicep frame : " << bicepFrame.position << std::endl;
    std::cout << "bicep ux : " << bicepFrame.ux() << std::endl;
    std::cout << "bicep uy : " << bicepFrame.uy() << std::endl;
    std::cout << "bicep uz : " << bicepFrame.uz() << std::endl;
    

    Bone* rib = new Bone("rib", ribFrame, ribLength);
    Bone* bicep = new Bone("bicep", bicepFrame, bicepLength);
    Bone* forearm = new Bone("forearm", forearmFrame, forearmLength);
    Bone* hand = new Bone("hand", handFrame, handLength);

    // Initialize the joints
    Joint* shoulder = new Joint("shoulder", rib, bicep);
    Joint* elbow = new Hinge("elbow", bicep, forearm, PI/2);
    Joint* wrist = new ParentRotule("wrist", forearm, hand, PI / 4);

    std::cout << "bones and joints created" << std::endl;
    // Add bones and joints to the arm
    addBone(rib);
    addBone(bicep);
    addBone(forearm);
    addBone(hand);

    addJoint(shoulder);
    addJoint(elbow);
    addJoint(wrist);
    std::cout << "bones and joints added" << std::endl;

    std::cout << "arm created" << std::endl;
    std::cout << " ribs direction : " << rib->getDirection() << std::endl;
}

void Arm::initialize()
{
    //initialize the arm must be done after OpenGl is initialized
    KinematicChain::initialize();
}

cgp::vec3 Arm::getShoulderPos()
{
    return getBone("bicep")->getStart();
}

void Arm::moveFromHandToShoulder(cgp::vec3 handTarget)
{
    //on va déplacer la main à la position cible
    fabrik("hand", "bicep", handTarget);
}
void Arm::moveFromShoulderToHand(cgp::vec3 shoulderTarget)
{
    //on va déplacer l'épaule à la position cible
    fabrikAscendingTree("bicep", "hand", shoulderTarget);
}
void Arm::moveHand(cgp::vec3 handTarget)
{
    //on va déplacer la main à la position cible
    fabrik("hand", "rib", handTarget);
}


Leg::Leg(float scale, bool isLeft)
{
    if (isLeft) {
        scale = -scale;
    }
    /*
    // Initialize the arm with bones and joints
    vec3 navelPos = cgp::vec3(0, 0, 0);
    vec3 hipPos = cgp::vec3(0.5 * FEETSPACING * scale, 0, -std::abs(scale) * 0.1);
    vec3 kneePos = cgp::vec3(0.5 * FEETSPACING * scale, 0, -std::abs(scale) * 0.3);
    vec3 anklePos = cgp::vec3(0.5 * FEETSPACING * scale, 0, -std::abs(scale) * 0.54);
    vec3 footPos = cgp::vec3(0.5 *FEETSPACING * scale, std::abs(scale) * FOOT, -std::abs(scale) * 0.54);

    Bone* pelvis = new Bone("pelvis", navelPos, hipPos);
    Bone* thigh = new Bone("thigh", hipPos, kneePos);
    Bone* calf = new Bone("calf", kneePos, anklePos);
    Bone* foot = new Bone("foot", anklePos, footPos);

    // Initialize the joints
    Joint* hip = new ParentRotule("hip", pelvis, thigh, PI / 2);
    Joint* knee = new GeneralRotule("knee", thigh, calf, cgp::vec3(0,0,-1),PI/2);
    Joint* ankle = new GeneralRotule("ankle", calf, foot, cgp::vec3(0,1,0), PI/6);

    // Add bones and joints to the arm
    addBone(pelvis);
    addBone(thigh);
    addBone(calf);
    addBone(foot);

    addJoint(hip);
    addJoint(knee);
    addJoint(ankle);
    */
}

void Leg::initialize()
{
    //initialize the arm must be done after OpenGl is initialized
    KinematicChain::initialize();
}

cgp::vec3 Leg::getHipPos()
{
    return getBone("thigh")->getStart();
}

void Leg::moveFromFootToHip(cgp::vec3 footTarget)
{
    //on va déplacer le pied à la position cible
    fabrik("foot", "thigh", footTarget);
}
void Leg::moveFromHipToFoot(cgp::vec3 hipTarget)
{
    //on va déplacer la hanche à la position cible
    fabrikAscendingTree("thigh", "foot", hipTarget);
}



IsoceleTriangle::IsoceleTriangle(){}
IsoceleTriangle::IsoceleTriangle(cgp::vec3 a, cgp::vec3 b, cgp::vec3 c)
{
    this->a = a;
    this->b = b;
    this->c = c;
    this->positionRef = c;
    normal = cgp::cross(b - a, c - a);
    normal = normal / cgp::norm(normal);
    height = cgp::norm(c - (a+b)/2);
    base = cgp::norm(b - a);
    heightDirection =cgp::normalize((b+a)/2 - c);
    //update the positionRef and then move the point so C = 0
    positionRef = c;
    this->a -= c;
    this->b -= c;
    this->c -= c;
}

void IsoceleTriangle::moveAandB(cgp::vec3 newA, cgp::vec3 newB)
{
    //translate the triangle so that the base is aligned with the new base
    newA -= positionRef;
    newB -= positionRef;
    
    cgp::vec3 newBaseMiddle = (newB + newA)/2;
    cgp::vec3 oldBaseMiddle = (b + a)/2;

    cgp::vec3 translation = newBaseMiddle - oldBaseMiddle;
    positionRef += translation;
}

void IsoceleTriangle::moveC(cgp::vec3 translationC)
{
    //calculer la nouvelle hauteur
    cgp::vec3 newHeightDirection = cgp::normalize((b + a) / 2 - translationC);
    //calculer la nouvelle position de a et b
    cgp::vec3 abDirection = cgp::cross(newHeightDirection, normal);
    abDirection = abDirection / cgp::norm(abDirection);

    a = abDirection * base / 2.0f + newHeightDirection * height;
    b = -abDirection * base / 2.0f + newHeightDirection * height;

    //update the positionRef and then move the point so C = 0
    positionRef += translationC;
}

void IsoceleTriangle::setC(cgp::vec3 newC)
{
    moveC(newC - positionRef);   
}

void IsoceleTriangle::translate(cgp::vec3 translation)
{
    positionRef += translation;
}

void IsoceleTriangle::initialize()
{
    segment.initialize();
}
void IsoceleTriangle::draw(environment_structure& environment)
{
    segment.draw(environment, a + positionRef, b + positionRef, cgp::vec3(0, 1, 0));
    segment.draw(environment, c + positionRef, a + positionRef, cgp::vec3(0, 1, 0));
    segment.draw(environment, c + positionRef, b + positionRef, cgp::vec3(0, 1, 0));
}

HumanSkeleton::HumanSkeleton(float scale, cgp::vec3 position)
{
    this->scale = scale;
  
    leftArm = Arm(scale, true);
    rightArm = Arm(scale, false);
    leftLeg = Leg(scale, true);
    rightLeg = Leg(scale, false);

    torso = IsoceleTriangle(leftArm.getShoulderPos(), rightArm.getShoulderPos(), cgp::vec3(0, 0, 0));   //a is left, b is right
    pelvis = IsoceleTriangle(leftLeg.getHipPos(), rightLeg.getHipPos(), cgp::vec3(0, 0, 0));    //a is left, b is right

    leftArm.translateBones(position);
    rightArm.translateBones(position);
    leftLeg.translateBones(position);
    rightLeg.translateBones(position);

    torso.translate(position);
    pelvis.translate(position);

    leftArm.skeleton = this;
    rightArm.skeleton = this;
    leftLeg.skeleton = this;
    rightLeg.skeleton = this;

    
}

void HumanSkeleton::initialize()
{
    leftArm.initialize();
    rightArm.initialize();
    leftLeg.initialize();
    rightLeg.initialize();
    torso.initialize();
    pelvis.initialize();
}

void HumanSkeleton::draw(environment_structure& environment)
{
    leftArm.draw(environment);
    rightArm.draw(environment);
    leftLeg.draw(environment);
    rightLeg.draw(environment);
    //std::cout << "torso pos : " << torso.positionRef << std::endl;
    torso.draw(environment);
    //std::cout << "pelvis pos : " << pelvis.positionRef << std::endl;
    pelvis.draw(environment);
}


void HumanSkeleton::translate(cgp::vec3 translation)
{
    position += translation;
    leftArm.translateBones(translation);
    rightArm.translateBones(translation);
    leftLeg.translateBones(translation);
    rightLeg.translateBones(translation);
}

void HumanSkeleton::movePelvis(cgp::vec3 translation)
{
    //move the pelvis and the legs
    setPelvisPos(position + translation);
}
void HumanSkeleton::setPelvisPos(cgp::vec3 position)
{

}

void HumanSkeleton::setPosLegs(cgp::vec3 leftFootPos, cgp::vec3 rightFootPos)
{

}
