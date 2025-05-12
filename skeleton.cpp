#include "skeleton.hpp"
#include "Nmath.hpp"
#include <cmath>

using namespace cgp;

void Bone::initialize()
{
    bone.display_type = curve_drawable_display_type::Segments;
    bone.initialize_data_on_gpu({{0,0,0},{1,0,0}});
}

void Bone::setEnd(cgp::vec3 end)
{
    this->end = end;
    direction = (end - start);
    length = cgp::norm(direction);
    direction = (end - start) / length;
}
void Bone::setStart(cgp::vec3 start)
{
    this->start = start;
    direction = (end - start);
    length = cgp::norm(direction);
    direction = (end - start) / length;
}

void Bone::translate(cgp::vec3 translation)
{
    this->start += translation;
    this->end += translation;
}

void Bone::draw(environment_structure& environment, cgp::vec3 poseReference)
{   
    if (cgp::norm(end - start) < 0.01f) {
        return;
    }
    // Draw the bone
    this->bone.vbo_position.update(numarray<cgp::vec3>{start + poseReference, end + poseReference});
    cgp::draw(this->bone, environment);
}

void KinematicChain::initialize()
{
    articulation.initialize_data_on_gpu(cgp::mesh_primitive_sphere(0.05f));
    articulation.material.color = cgp::vec3(0.5f,0.5f,0.5f);
    std::cout << "Initializing the kinematic chain" << std::endl;
    for (auto& bone : bones) {
        std::cout << "Initializing bone " << std::endl;
        std::cout << "Initializing bone " << bone->name << std::endl;
        bone->initialize();
    }
}

void KinematicChain::addBone(Bone* bone)
{
    bones.push_back(bone);
}
void KinematicChain::addJoint(Joint* joint)
{
    joints.push_back(joint);
}

Bone* KinematicChain::getBone(std::string name)
{
    for (auto bone : bones) {
        if (bone->name == name) {
            return bone;
        }
    }
    return nullptr;
}

void KinematicChain::draw(environment_structure& environment)
{
    // Draw the bones and articulations
    for (auto& bone : bones) {
        bone->draw(environment, position);
        articulation.model.translation = bone->end + position;
        cgp::draw(articulation, environment);
        articulation.model.translation = bone->start + position;
        cgp::draw(articulation, environment);
    }
}

void KinematicChain::backwardFabrik(std::vector<Bone*> bonesOrder, vec3 target, float tolerance)
{
    // Implementation of the FABRIK algorithm
    // This function will take to bones, the target position and the tolerance
    // It will try to move the end of the first bone to the target position 
    // and will not move the start of the second

    //on procède de facon itérative en parcourant bonesOrder
    
    for (auto& bone : bonesOrder) {
        float length = bone->length;
        bone->setEnd(target);
        vec3 tragetStart = bone->end - bone->direction * length;
        bone->setStart(tragetStart);
        target = bone->start;
    }
}
    

void KinematicChain::forwardFabrik(std::vector<Bone*> bonesOrder, vec3 target, float tolerance)
{
    // Implementation of the FABRIK algorithm
    // This function will take to bones, the target position and the tolerance
    // It will try to move the end of the first bone to the target position 
    // and will not move the start of the second

    ////on procède de facon itérative en parcourant bonesOrder à l'envers

    
    for (auto it = bonesOrder.rbegin(); it != bonesOrder.rend(); ++it) {
        Bone* bone = *it;
        float length = bone->length;
        bone->setStart(target);
        vec3 targetEnd = bone->start + bone->direction * length;
        bone->setEnd(targetEnd);
        target = bone->end;
    }
}
void KinematicChain::backwardApplyConstraints(std::vector<Bone*> bonesOrder)
{
    // Implementation of the FABRIK algorithm
    // This function recusively apply the constraints to the KinematicChain

    //on procède de facon itérative en parcourant bonesOrder en évitant le dernier os qui n'a pas de joint parent

    for (auto& bone : bonesOrder) {
        if (bone->jointFather != nullptr) {
            bone->jointFather->applyConstraintOnFather(skeleton);
        }
    }
}

void KinematicChain::forwardApplyConstraints(std::vector<Bone*> bonesOrder)
{
    // Implementation of the FABRIK algorithm
    // This function recusively apply the constraints to the KinematicChain

    //on procède de facon itérative en parcourant bonesOrder à l'envers

    for (auto it = bonesOrder.rbegin(); it != bonesOrder.rend(); ++it) {
        Bone* bone = *it;
        if (bone->jointFather != nullptr) {
            bone->jointFather->applyConstraintOnChild(skeleton);
        }
    }
}

void KinematicChain::fabrik(std::vector<Bone*> bonesOrder, vec3 target, float tolerance, int max_iter){
    std::cout << "coucou" << std::endl;
    //fixedpoint est start du dernier os
    cgp::vec3 fixedPoint = bonesOrder.back()->start;
    for (int i = 0; i < max_iter; i++) {
        //backward FABRIK
        backwardFabrik(bonesOrder, target, tolerance);
        //constraint
        backwardApplyConstraints(bonesOrder);
        //forward FABRIK
        forwardFabrik(bonesOrder, fixedPoint, tolerance);
        //constraint
        forwardApplyConstraints(bonesOrder);
    }
}

void KinematicChain::fabrik(vec3 target, float tolerance, int max_iter)
{
    //on applique fabrik déjà implémenté avec un vecteur de pointeur sur les bones mais dans l'ordre inverse
    std::vector<Bone*> bonesOrder;
    for (auto it = bones.rbegin(); it != bones.rend(); ++it) {
        bonesOrder.push_back(*it);
    }
    fabrik(bonesOrder, target, tolerance, max_iter);
}

void KinematicChain::fabrik(Bone* targetBone, Bone* fixedBone, vec3 target, float tolerance, int max_iter)
{
    //on va construire un vecteur de pointeur sur les bones pour les parcourir à l'endroit puis à l'envers
    
    std::vector<Bone*> bonesOrder;
    Bone* currentBone = targetBone;
    bonesOrder.push_back(currentBone);
    //on remonte l'arbre jusqu'à targetBone
    while (currentBone != nullptr && currentBone->name != fixedBone->name) {
        currentBone = currentBone->jointFather->boneFather;
        bonesOrder.push_back(currentBone);
    }
    
    //on applique fabrik déjà implémenté avec un vecteur de pointeur sur les bones
    fabrik(bonesOrder, target, tolerance, max_iter);
}

void KinematicChain::fabrik(std::string targetBoneName, std::string fixedBoneName, vec3 target, float tolerance, int max_iter)
{
    Bone* targetBone = getBone(targetBoneName);
    Bone* fixedBone = getBone(fixedBoneName);

    if (targetBone == nullptr) {
        std::cerr << "Error: Bone 'targetBone' not found." << std::endl;
        return;
    }
    if (targetBone->jointFather == nullptr) {
        std::cerr << "Error: jointFather is null for targetBone." << std::endl;
        return;
    }

    if (targetBone != nullptr && fixedBone != nullptr) {
        fabrik(targetBone, fixedBone, target, tolerance, max_iter);
    } else {
        std::cerr << "Error: Bone not found" << std::endl;
    }
}

void KinematicChain::setEndEffectorWorldPos(cgp::vec3 target)
{
    fabrik(target - position);
    endEffectorPos = target - position;
}
void KinematicChain::setEndEffectorRelativePos(cgp::vec3 target)
{
    fabrik(target);
    endEffectorPos = target;
}
void KinematicChain::moveEndEffector(cgp::vec3 move)
{
    setEndEffectorRelativePos(endEffectorPos + move);
}
void KinematicChain::translate(cgp::vec3 translation)
{
    position += translation;
}
void KinematicChain::setStartEffector(cgp::vec3 target)
{
    //on translate la chaine cinématique pour que le point de départ soit à la position cible 
    //puis on applique Fabrik pour remettre la fin de la chaine à sa position initiale

    cgp::vec3 translation = target - position;
    translate(translation);
    moveEndEffector(-translation);
}

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
    Joint* shoulder = new Joint("shoulder", trapezius, bicep);
    Joint* elbow = new ParentRotule("elbow", bicep, forearm, 0.5f);
    Joint* wrist = new ParentRotule("wrist", forearm, hand, 1.7f);

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
    Joint* hip = new Joint("hip", pelvis, thigh);
    Joint* knee = new ParentRotule("knee", thigh, calf, 1.5f);
    Joint* ankle = new ParentRotule("ankle", calf, foot, 1.7f);

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
}

void IsoceleTriangle::moveC(cgp::vec3 newC)
{
    positionRef += newC;
}

void IsoceleTriangle::setC(cgp::vec3 newC)
{
    positionRef = newC;
}

HumanSkeleton::HumanSkeleton(float scale, cgp::vec3 position)
: scale(scale),
      position(position),
      torso(leftArm.getShoulderPos(), rightArm.getShoulderPos(), cgp::vec3(0, 0, 0)),
      pelvis(leftLeg.getHipPos(), rightLeg.getHipPos(), cgp::vec3(0, 0, 0))
{
    this->scale = scale;
    this->position = position;
  
    leftArm = Arm(scale, true);
    rightArm = Arm(scale, false);
    leftLeg = Leg(scale, true);
    rightLeg = Leg(scale, false);

    leftArm.translate(position);
    rightArm.translate(position);
    leftLeg.translate(position);
    rightLeg.translate(position);

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
}

void HumanSkeleton::draw(environment_structure& environment)
{
    leftArm.draw(environment);
    rightArm.draw(environment);
    leftLeg.draw(environment);
    rightLeg.draw(environment);
}

void HumanSkeleton::translate(cgp::vec3 translation)
{
    position += translation;
    leftArm.translate(translation);
    rightArm.translate(translation);
    leftLeg.translate(translation);
    rightLeg.translate(translation);
}


void HumanSkeleton::moveLegs(cgp::vec3 leftFootMove, cgp::vec3 rightFootMove)
{
    leftLeg.setEndEffectorWorldPos(leftFootMove);
    rightLeg.setEndEffectorWorldPos(rightFootMove);
    //update the pelvis position
    
}
