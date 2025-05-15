#pragma once

#include "kinematicChain.hpp"
#include "Nmath.hpp"
#include <cmath>

using namespace cgp;

Bone ::Bone(std::string name, cgp::frame frame, float length)
{
    this->name = name;
    this->frameAbsolut = frame;
    this->length = length;
}
void Bone::initialize()
{
    bone.initialize();
}
cgp::vec3 Bone::getStart()
{
    return frameAbsolut.position;
}
cgp::vec3 Bone::getEnd()
{
    return frameAbsolut.position + getDirection() * length;
}
cgp::vec3 Bone::getDirection()
{
    return cgp::normalize(frameAbsolut.uz());
}
cgp::vec3 Bone::getX()
{
    return cgp::normalize(frameAbsolut.ux());
}
cgp::vec3 Bone::getY()
{
    return cgp::normalize(frameAbsolut.uy());
}
cgp::vec3 Bone::getZ()
{
    return cgp::normalize(frameAbsolut.uz());
}
void Bone::translate(cgp::vec3 translation)
{
    frameAbsolut = frameAbsolut + translation;
}
void Bone::setEnd(cgp::vec3 end)
{
    //Translate the bone to the new end position
    cgp::vec3 currentEnd = getEnd();
    cgp::vec3 translation = end - currentEnd;
    translate(translation);
}
void Bone::setStart(cgp::vec3 start)
{
    //Translate the bone to the new start position
    cgp::vec3 translation = start - frameAbsolut.position;
    translate(translation);
}

void Bone::fabricStepForward(cgp::vec3 target)
{
    cgp::vec3 oldDirection = getDirection();
    cgp::vec3 end = getEnd();

    //Translate the bone to the new start position
    setStart(target);


    cgp::vec3 newDirection = cgp::normalize(end - target);
    cgp::rotation_transform rotation = cgp::rotation_transform::from_vector_transform(oldDirection, newDirection);
    frameAbsolut = rotation * frameAbsolut;
}

void Bone::fabricStepBackward(cgp::vec3 target)
{
    cgp::vec3 oldDirection = getDirection();
    cgp::vec3 newDirection = cgp::normalize(target - getStart());

    cgp::rotation_transform rotation = cgp::rotation_transform::from_vector_transform(oldDirection, newDirection);
    frameAbsolut = rotation * frameAbsolut;

    setEnd(target);
}

void Bone::forwardConstraint()
{
    //on va faire en sorte que le bone soit bien placé par rapport à son parent
    if (jointFather != nullptr) {
        jointFather->applyConstraintOnChild();
    }
}
void Bone::backWardConstraint()
{
    //on va faire en sorte que le bone soit bien placé par rapport à son enfant
    if (jointChild != nullptr) {
        jointChild->applyConstraintOnFather();
    }
}

void Bone::draw(environment_structure& environment)
{   
    
    // Draw the bone
    cgp::vec3 start = getStart();
    cgp::vec3 end = getEnd();

    if (cgp::norm(end - start) < 0.01f) {
        return;
    }

    bone.draw(environment, start, end, cgp::vec3(1.0f, 0.5f, 0.5f));
    // Draw the frame of the bone
    bone.drawFrame(environment, frameAbsolut);
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

Joint* KinematicChain::getJoint(std::string name)
{
    for (auto joint : joints) {
        if (joint->name == name) {
            return joint;
        }
    }
    return nullptr;
}

void KinematicChain::draw(environment_structure& environment)
{
    // Draw the bones and articulations
    for (auto& bone : bones) {
        bone->draw(environment);
        articulation.model.translation = bone->getStart();
        cgp::draw(articulation, environment);
        articulation.model.translation = bone->getEnd();
        cgp::draw(articulation, environment);

        std::cout << "bone name : " << bone->name << std::endl;
        std::cout << "bone length : " << bone->length << std::endl;
    }
}

void KinematicChain::translateBones(cgp::vec3 translation)
{
    for (auto& bone : bones) {
        bone->translate(translation);
    }
}

void KinematicChain::backwardFabrik(std::vector<Bone*> bonesOrder, vec3 target, float tolerance)
{
    // Implementation of the FABRIK algorithm
    // This function will take to bones, the target position and the tolerance
    // It will try to move the end of the first bone to the target position 
    // and will not move the start of the second
    //on procède de facon itérative en parcourant bonesOrder
    
    for (auto it = bonesOrder.rbegin(); it != bonesOrder.rend(); ++it){
        Bone* bone = *it;
        bone->fabricStepBackward(target);
        bone->backWardConstraint();
        target = bone->getStart();
    }
}
    

void KinematicChain::forwardFabrik(std::vector<Bone*> bonesOrder, vec3 target, float tolerance)
{
    // Implementation of the FABRIK algorithm
    // This function will take to bones, the target position and the tolerance
    // It will try to move the end of the first bone to the target position 
    // and will not move the start of the second
    ////on procède de facon itérative en parcourant bonesOrder à l'envers
    for (auto& bone : bonesOrder){
        bone->fabricStepForward(target);
        bone->forwardConstraint();
        target = bone->getEnd();
    }
}


void KinematicChain::fabrik(std::vector<Bone*> bonesOrder, vec3 target, float tolerance, int max_iter){
    //std::cout << "coucou" << std::endl;
    //fixedpoint est start du premier os
    cgp::vec3 fixedPoint = bonesOrder.front()->getStart();
    for (int i = 0; i < max_iter; i++) {
        //backward FABRIK
        backwardFabrik(bonesOrder, target, tolerance);
        //forward FABRIK
        forwardFabrik(bonesOrder, fixedPoint, tolerance);
    }
}

void KinematicChain::fabrik(vec3 target, float tolerance, int max_iter)
{
    //on applique fabrik déjà implémenté avec un vecteur de pointeur sur les bones
    fabrik(bones, target, tolerance, max_iter);
}

void KinematicChain::fabrik(Bone* targetBone, Bone* fixedBone, vec3 target, float tolerance, int max_iter)
{
    //on va construire un vecteur de pointeur sur les bones pour les parcourir à l'endroit puis à l'envers
    
    std::vector<Bone*> bonesOrder;
    Bone* currentBone = fixedBone;
    bonesOrder.push_back(currentBone);
    //on remonte l'arbre jusqu'à targetBone
    while (currentBone != nullptr && currentBone->name != targetBone->name) {
        currentBone = currentBone->jointChild->boneChild;
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


void KinematicChain::fabrikAscendingTree(std::vector<Bone*> bonesOrder, vec3 target, float tolerance, int max_iter)
{
    //std::cout << "coucou" << std::endl;
    //fixedpoint est end du dernier os
    cgp::vec3 fixedPoint = bonesOrder.back()->getEnd();
    for (int i = 0; i < max_iter; i++) {
        //forward FABRIK
        forwardFabrik(bonesOrder, target, tolerance);
        //backward FABRIK
        backwardFabrik(bonesOrder, fixedPoint, tolerance);
    }
}

void KinematicChain::fabrikAscendingTree(vec3 target, float tolerance, int max_iter)
{
    fabrikAscendingTree(bones, target, tolerance);
}

void KinematicChain::fabrikAscendingTree(std::string targetBoneName, std::string fixedBoneName, vec3 target, float tolerance, int max_iter)
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
        fabrikAscendingTree(targetBone, fixedBone, target, tolerance);
    } else {
        std::cerr << "Error: Bone not found" << std::endl;
    }
}

void KinematicChain::fabrikAscendingTree(Bone* targetBone, Bone* fixedBone, vec3 target, float tolerance, int max_iter)
{
    //on va construire un vecteur de pointeur sur les bones pour les parcourir à l'endroit puis à l'envers
    
    std::vector<Bone*> bonesOrder;
    Bone* currentBone = targetBone;
    bonesOrder.push_back(currentBone);
    //on monte l'arbre jusqu'à targetBone
    while (currentBone != nullptr && currentBone->name != fixedBone->name) {
        currentBone = currentBone->jointChild->boneChild;
        bonesOrder.push_back(currentBone);
    }
    
    //on applique fabrikAscendingTree déjà implémenté avec un vecteur de pointeur sur les bones
    fabrikAscendingTree(bonesOrder, target, tolerance);
}


/*
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
void KinematicChain::translateBones(cgp::vec3 translation)
{
    for (auto& bone : bones) {
        bone->translate(translation);
    }
}
void KinematicChain::setReferencePos(cgp::vec3 position)
{
    this->position = position;
}

bool KinematicChain::translateFromEndEffectorToBone(std::string name, cgp::vec3 translation)
{
    //on récupère le bone
    Bone* bone = getBone(name);
    if (bone == nullptr) {
        std::cerr << "Error: Bone not found" << std::endl;
        return false;
    }
    //on vérifie que la nouvelle position est valide (c'est à dire que la distance par rapport au parent est respectée)
    if (bone->jointFather != nullptr) {
        cgp::vec3 direction = bone->start + translation - bone->jointFather->boneFather->start;
        float length = cgp::norm(direction);
        if (length / bone->jointFather->boneFather->length < 0.98f || length / bone->jointFather->boneFather->length > 1.02f) {
            std::cerr << "Error: New position is too far from parent" << std::endl;
            std::cerr << "length ratio : " << length / bone->jointFather->boneFather->length << std::endl;
            return false;
        }
        else {
            std::cout << "IT S OKAYYYYYYYYYYYYYYYY"<< std::endl;
            std::cout << "length ratio : " << length / bone->jointFather->boneFather->length << std::endl;
        }
        
    }
    //on translate end du parent
    if (bone->jointFather != nullptr) {
        bone->jointFather->boneFather->setEnd(bone->jointFather->boneFather->end + translation);
    }
    //on va translater tous les bones en partant de bone jusqu'au end effector (en descendant l'arbre)
    Bone* currentBone = bone;
    while (currentBone->jointChild != nullptr) {
        currentBone->translate(translation);
        currentBone = currentBone->jointChild->boneChild;
    }
    currentBone->translate(translation);
    return true;
}




void KinematicChain::setBoneStartWorldPos(std::string name, cgp::vec3 target)
{
    //on va appeler fabrikAscendingTree pour déplacer le la chaine cinématique
    //on récupère le bone endEffector
    Bone* endEffector = bones.back();
    std::cout << "end effector name : " << endEffector->name << std::endl;
    fabrikAscendingTree(name, endEffector->name, target - position);
    

    //on deplace end du parent de target - position
    Bone* bone = getBone(name);
    if (bone == nullptr) {
        std::cerr << "Error: Bone not found in setBoneStartWorldPos" << std::endl;
        return;
    }
    if (bone->jointFather == nullptr) {
        std::cerr << "Error: jointFather is null for targetBone in setBoneStartWorldPos" << std::endl;
        return;
    }
   
    bone->jointFather->boneFather->setEnd(target - position);
    
}

void KinematicChain::setBoneStartRelativePos(std::string name, cgp::vec3 target)
{
    setBoneStartWorldPos(name, target + position);
}

void KinematicChain::setStartEffector(cgp::vec3 target)
{
    //on va appeler fabrikAscendingTree pour déplacer la chaine cinématique
    fabrikAscendingTree(target - position);
    //on translate les bones de position - target
    translateBones(position - target);
    //on met à jour la position de reference de la chaine cinématique
    position = target;
}
*/
