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
    vec3 navelPos = cgp::vec3(0, 0, 0);
    vec3 shoulderPos = cgp::vec3(0.09*scale, 0, std::abs(scale) * 0.25);
    vec3 elbowPos = cgp::vec3(0.09*scale, 0, std::abs(scale) * 0.062);
    vec3 wristPos = cgp::vec3(0.09*scale, 0, std::abs(scale) * -0.083);
    vec3 handPos = cgp::vec3(0.09*scale, 0, std::abs(scale) * -0.183);

    Bone* trapezius = new Bone("ribs", navelPos, shoulderPos);
    Bone* bicep = new Bone("bicep", shoulderPos, elbowPos);
    Bone* forearm = new Bone("forearm", elbowPos, wristPos);
    Bone* hand = new Bone("hand", wristPos, handPos);

    // Initialize the joints
    Joint* shoulder = new GeneralRotule("shoulder", trapezius, bicep, vec3(0, 1, 0),1 * PI / 2);
    Joint* elbow = new GeneralRotule("elbow", bicep, forearm, vec3(0, 1, 0), 1 * PI / 2);
    Joint* wrist = new ParentRotule("wrist", forearm, hand, 1 * PI / 2);

    std::cout << "bones and joints created" << std::endl;
    // Add bones and joints to the arm
    addBone(trapezius);
    addBone(bicep);
    addBone(forearm);
    addBone(hand);

    addJoint(shoulder);
    addJoint(elbow);
    addJoint(wrist);
    std::cout << "bones and joints added" << std::endl;

    position = cgp::vec3(0, 0, 0);

    std::cout << "arm created" << std::endl;
    std::cout << " ribs length : " << trapezius->length << std::endl;
}

void Arm::initialize()
{
    //initialize the arm must be done after OpenGl is initialized
    KinematicChain::initialize();
}

cgp::vec3 Arm::getShoulderPos()
{
    return getBone("bicep")->start;
}

void Arm::moveFromHandToShoulder(cgp::vec3 handTarget)
{
    //on va déplacer la main à la position cible
    fabrik("hand", "bicep", handTarget - position);
}
void Arm::moveFromShoulderToHandWorldPos(cgp::vec3 shoulderTarget)
{
    //on va déplacer l'épaule à la position cible
    setBoneStartWorldPos("bicep", shoulderTarget);
}
void Arm::moveFromShoulderToHandRelativePos(cgp::vec3 shoulderTarget)
{
    //on va déplacer l'épaule à la position cible
    std::cout<<"i'm here"<<std::endl;
    setBoneStartRelativePos("bicep", shoulderTarget);
    std::cout<<"i'm also there"<<std::endl;
}

void Arm::updateJoints()
{
    //on met à jour la position de l'axe de l'épaule qui est la normale du torso
    Joint* shoulder = getJoint("shoulder");
    Joint* elbow = getJoint("elbow");

    //Joint* wrist = getJoint("wrist");         //wrist joint axis is defined by the forearm

    // Cast dynamique vers GeneralRotule*
    GeneralRotule* shoulderRotule = dynamic_cast<GeneralRotule*>(shoulder);
    if (shoulderRotule != nullptr) {
        shoulderRotule->set(cgp::normalize(skeleton->torso.normal), 1 * PI / 2);
    } else {
        std::cerr << "Error: Joint 'shoulder' is not a GeneralRotule." << std::endl;
    }

    GeneralRotule* elbowRotule = dynamic_cast<GeneralRotule*>(elbow);
    if (elbowRotule != nullptr) {
        elbowRotule->set(cgp::normalize(skeleton->torso.normal), 1 * PI / 2);
    } else {
        std::cerr << "Error: Joint 'elbow' is not a GeneralRotule." << std::endl;
    }
}

Leg::Leg(float scale, bool isLeft)
{
    if (isLeft) {
        scale = -scale;
    }
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

    position = cgp::vec3(0, 0, 0);
}

void Leg::initialize()
{
    //initialize the arm must be done after OpenGl is initialized
    KinematicChain::initialize();
}

cgp::vec3 Leg::getHipPos()
{
    return getBone("thigh")->start;
}

void Leg::moveFromFootToHip(cgp::vec3 footTarget)
{
    //on va déplacer le pied à la position cible
    fabrik("foot", "thigh", footTarget - position);
}
void Leg::moveFromHipToFootWorldPos(cgp::vec3 hipTarget)
{
    //on va déplacer la hanche à la position cible
    setBoneStartWorldPos("thigh", hipTarget);
}
void Leg::moveFromHipToFootRelativePos(cgp::vec3 hipTarget)
{
    //on va déplacer la hanche à la position cible
    setBoneStartRelativePos("thigh", hipTarget);
}

void Leg::updateJoints()
{
    Joint* ankle = getJoint("ankle");
    GeneralRotule* ankleRotule = dynamic_cast<GeneralRotule*>(ankle); // Utilisation d'un nouveau nom
    if (ankleRotule != nullptr) {
        ankleRotule->set(cgp::normalize(skeleton->torso.normal), PI / 2);
    } else {
        std::cerr << "Error: Joint 'ankle' is not a GeneralRotule." << std::endl;
    }

    Joint* knee = getJoint("knee");
    GeneralRotule* kneeRotule = dynamic_cast<GeneralRotule*>(knee);
    if (kneeRotule != nullptr) {
        kneeRotule->set(cgp::normalize(skeleton->pelvis.heightDirection), PI / 2);
    } else {
        std::cerr << "Error: Joint 'elbow' is not a GeneralRotule." << std::endl;
    }
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
    //find the best point C to keep the triangle isoceles with the same height with the new A and B
    //let's move a and b to the new position
    a = newA;
    b = newB;
    //find the new csss
    cgp::vec3 ab = b - a;
    cgp::vec3 ab_normal = cgp::cross(ab, normal);
    ab_normal = ab_normal / cgp::norm(ab_normal);
    c = a + ab / 2.0f + ab_normal * height;
    
    //update the positionRef and then move the point so C = 0
    positionRef += c;
    a -= c;
    b -= c;
    c -= c;

    //update the normal
    normal = cgp::cross(b - a, c - a);
    normal = normal / cgp::norm(normal);

    //update the heightDirection
    heightDirection = cgp::normalize((b + a) / 2 - c);
    //update the height
    height = cgp::norm(c - (a + b) / 2);
    //update the base
    base = cgp::norm(b - a);
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
    this->position = position;
  
    leftArm = Arm(scale, true);
    rightArm = Arm(scale, false);
    leftLeg = Leg(scale, true);
    rightLeg = Leg(scale, false);

    torso = IsoceleTriangle(leftArm.getShoulderPos(), rightArm.getShoulderPos(), cgp::vec3(0, 0, 0));   //a is left, b is right
    pelvis = IsoceleTriangle(leftLeg.getHipPos(), rightLeg.getHipPos(), cgp::vec3(0, 0, 0));    //a is left, b is right

    leftArm.translate(position);
    rightArm.translate(position);
    leftLeg.translate(position);
    rightLeg.translate(position);

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

void HumanSkeleton::updateJoints()
{
    leftArm.updateJoints();
    rightArm.updateJoints();
    leftLeg.updateJoints();
    rightLeg.updateJoints();
}

void HumanSkeleton::translate(cgp::vec3 translation)
{
    position += translation;
    leftArm.translate(translation);
    rightArm.translate(translation);
    leftLeg.translate(translation);
    rightLeg.translate(translation);
}

void HumanSkeleton::movePelvis(cgp::vec3 translation)
{
    //move the pelvis and the legs
    setPelvisPos(position + translation);
}
void HumanSkeleton::setPelvisPos(cgp::vec3 position)
{
    //set the pelvis position and move the legs
    std::cout<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
    this->position = position;
    pelvis.setC(position);
    std::cout << "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB" << std::endl;
    torso.setC(position);
    std::cout << "CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC" << std::endl;
    updateJoints();
    
    //translate the limbs to match the new pelvis position
    leftArm.setReferencePos(position);
    rightArm.setReferencePos(position);
    leftLeg.setReferencePos(position);
    rightLeg.setReferencePos(position);
    
    std::cout << "DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDDD" << std::endl;


    //move the legs to match the new pelvis position
    leftLeg.moveFromHipToFootRelativePos(pelvis.a);
    rightLeg.moveFromHipToFootRelativePos(pelvis.b);

    std::cout << "EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE" << std::endl;

    //move the arms to match the new torso position
    leftArm.moveFromShoulderToHandRelativePos(torso.a);
    std::cout << "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF" << std::endl;
    rightArm.moveFromShoulderToHandRelativePos(torso.b);

    std::cout << "GGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGGG" << std::endl;


    std::cout << "pelvis pos a : " << pelvis.a << std::endl;
    std::cout << "leftLeg pos : " << leftLeg.getHipPos() << std::endl;

    std::cout << "pelvis ref : " << pelvis.positionRef << std::endl;
    std::cout << "leftLeg ref : " << leftLeg.position << std::endl;
}

void HumanSkeleton::setPosLegs(cgp::vec3 leftFootPos, cgp::vec3 rightFootPos)
{
    //move legs : 
    leftLeg.setEndEffectorWorldPos(leftFootPos);
    rightLeg.setEndEffectorWorldPos(rightFootPos);
    //update pelvis 
    std::cout<<"AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
    std::cout << "leftLeg pos : " << leftLeg.getHipPos() << std::endl;
    std::cout << "rightLeg pos : " << rightLeg.getHipPos() << std::endl;
    pelvis.moveAandB(leftLeg.getHipPos(), rightLeg.getHipPos());
    std::cout << "BBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBBB" << std::endl;
    setPelvisPos(pelvis.positionRef);
}
