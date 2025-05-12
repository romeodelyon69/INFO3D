#include <stdlib.h>
#include <stdio.h>
#include "joint.hpp"
#include "Nmath.hpp"



Joint::Joint(std::string name, Bone* boneFather, Bone* boneChild)
{
    this->name = name;
    this->boneFather = boneFather;
    this->boneChild = boneChild;
    boneFather->jointChild = this;
    boneChild->jointFather = this;
}

void Joint::applyConstraintOnChild(HumanSkeleton *skeleton)
{
    cgp::vec3 translation =  boneFather->end - boneChild->start;
    boneChild->translate(translation);

}
void Joint::applyConstraintOnFather(HumanSkeleton *skeleton)
{
    cgp::vec3 translation =  boneChild->start - boneFather->end;
    boneFather->translate(translation);
}

GeneralRotule::GeneralRotule(std::string name, Bone* boneFather, Bone* boneChild, vec3 axis, float maxAngle)
    : Joint(name, boneFather, boneChild), axis(axis), maxAngle(maxAngle) {}


void GeneralRotule::set(vec3 axis, float maxAngle)
{
    this->axis = axis;
    this->maxAngle = maxAngle;
}

void GeneralRotule::applyConstraintOnChild(HumanSkeleton *skeleton)
{   
    //We are going to move the child bone to verify the constraint 
    cgp::vec3 translation =  boneFather->end - boneChild->start;
    boneChild->translate(translation);
    
    //get the position of the child bone 

    //get the angle between the child bone and the axis
    float angle = angleBetween(boneChild->direction, axis);

    //std::cout << "#########################################################################" << std::endl;
    //std::cout << "Applying constraint for joint " << name << "angle is "<< angle<< std::endl;

    //check if the angle is greater than the max angle
    if (angle > maxAngle) {
        //std::cout << "Angle is greater than max angle and is " << angle << std::endl;
       //rotate the child bone aroud the cross product of the child vector and the axis
        vec3 rotationAxis = normalize(cross(boneChild->direction, axis));
        //rotate the child bone
        float angleToRotate = angle - maxAngle;
        mat3 rotationMatrix = mat3::build_rotation_from_axis_angle(rotationAxis, angleToRotate);
        vec3 newChildVector = boneChild->length * rotationMatrix * boneChild->direction;

        boneChild->setEnd(boneChild->start + newChildVector);
        //std::cout<<"le nouvel angle vaut : "<<angleBetween(newChildVector, axis) << std::endl;
    }
}

void GeneralRotule::applyConstraintOnFather(HumanSkeleton *skeleton)
{
    

    cgp::vec3 translation =  boneChild->start - boneFather->end;
    boneFather->translate(translation);
    //get the angle between the father bone and the axis
    float angle = angleBetween(boneChild->direction, axis);

    //std::cout << "#########################################################################" << std::endl;
    //std::cout << "Applying constraint for joint " << name << "angle is "<< angle<< std::endl;

    //check if the angle is greater than the max angle
    if (angle > maxAngle) {
        //std::cout << "Angle is greater than max angle and is " << angle << std::endl;
       //rotate the child bone aroud the cross product of the child vector and the axis
        vec3 rotationAxis = normalize(cross(boneChild->direction, axis));
        //rotate the father bone
        //std::cout << "ancienne longueuer " << boneFather->length << std::endl;
        float angleToRotate = maxAngle - angle;
        mat3 rotationMatrix = mat3::build_rotation_from_axis_angle(rotationAxis, angleToRotate);
        vec3 newFatherVector = boneFather->length * rotationMatrix * boneFather->direction;
        //std::cout << "bone father direction norme " << cgp::norm(boneFather->direction) << std::endl;
        //std::cout << "newFatherVector norme " << cgp::norm(newFatherVector) << std::endl;
        
        boneFather->setStart(boneFather->end - newFatherVector);
        //std::cout << "nouvelle longueur" << boneFather->length << std::endl;
        //std::cout<<"le nouvel angle vaut : "<<angleBetween(newChildVector, axis) << std::endl;
    }
}


ParentRotule::ParentRotule(std::string name, Bone* boneFather, Bone* boneChild, float maxAngle)
    : GeneralRotule(name, boneFather, boneChild, normalize(boneFather->end - boneFather->start), maxAngle) {}

void ParentRotule::setAngle(float maxAngle)
{
    this->maxAngle = maxAngle;
}

void ParentRotule::update(){
    this-> axis = normalize(boneFather->end - boneFather->start);
}

void ParentRotule::applyConstraintOnChild(HumanSkeleton *skeleton)
{
    //update the axis
    this->update();
    //apply the constraint
    GeneralRotule::applyConstraintOnChild(skeleton);
}

void ParentRotule::applyConstraintOnFather(HumanSkeleton *skeleton)
{
    //update the axis
    this->update();
    //apply the constraint
    GeneralRotule::applyConstraintOnFather(skeleton);
}



