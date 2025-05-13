#pragma once

#include "kinematicChain.hpp"
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
    endEffectorPos = bones.back()->end;
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
        bone->draw(environment, position);
        articulation.model.translation = bone->end + position;
        cgp::draw(articulation, environment);
        articulation.model.translation = bone->start + position;
        cgp::draw(articulation, environment);

        std::cout << "bone name : " << bone->name << std::endl;
        std::cout << "bone length : " << bone->length << std::endl;
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
    //std::cout << "coucou" << std::endl;
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

void KinematicChain::fabrikAscendingTree(std::vector<Bone*> bonesOrder, vec3 target, float tolerance, int max_iter)
{
    //std::cout << "coucou" << std::endl;
    //fixedpoint est end du premier os
    cgp::vec3 fixedPoint = bonesOrder.front()->end;
    for (int i = 0; i < max_iter; i++) {
        //forward FABRIK
        forwardFabrik(bonesOrder, target, tolerance);
        //constraint
        forwardApplyConstraints(bonesOrder);
        //backward FABRIK
        backwardFabrik(bonesOrder, fixedPoint, tolerance);
        //constraint
        backwardApplyConstraints(bonesOrder);
    }
}

void KinematicChain::fabrikAscendingTree(vec3 target, float tolerance, int max_iter)
{
    std::vector<Bone*> bonesOrder;
    for (auto it = bones.rbegin(); it != bones.rend(); ++it) {
        bonesOrder.push_back(*it);
    }
    fabrikAscendingTree(bonesOrder, target, tolerance);
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
    Bone* currentBone = fixedBone;
    bonesOrder.push_back(currentBone);
    //on monte l'arbre jusqu'à targetBone
    while (currentBone != nullptr && currentBone->name != targetBone->name) {
        currentBone = currentBone->jointFather->boneFather;
        bonesOrder.push_back(currentBone);
    }
    
    //on applique fabrikAscendingTree déjà implémenté avec un vecteur de pointeur sur les bones
    fabrikAscendingTree(bonesOrder, target, tolerance);
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

/*
void KinematicChain::setBoneStartWorldPos(std::string name, cgp::vec3 target)
{
    //on translate la chaîne entre le end effector et le bone cible 
    Bone* bone = getBone(name);
    if (bone == nullptr) {
        std::cerr << "Error: Bone not found" << std::endl;
        return;
    }
    std::cout << "################################################################# "<< std::endl;
    std::cout << "bone start : " << bone->start + position << std::endl;
    std::cout << "target : " << target << std::endl;

    cgp::vec3 translation = target - position - bone->start;
    if(!translateFromEndEffectorToBone(name, translation)){
        std::cerr << "Something went wrong" << std::endl;
        return;
    }
    
    //on récupère le end effector
    Bone* endEffector = bones.back();
    //on applique Fabrik pour remettre la fin de la chaine à sa position initiale
    fabrik(endEffector, bone, endEffector->end - translation);
}
*/
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

