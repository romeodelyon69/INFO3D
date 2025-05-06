#include <stdlib.h>
#include <stdio.h>
#include "joint.hpp"
#include "Nmath.hpp"



Joint::Joint(std::string name, Bone* boneFather, Bone* boneChild)
{
    this->name = name;
    this->boneFather = boneFather;
    this->boneChild = boneChild;
}

void Joint::applyConstraint(Skeleton *skeleton)
{
    //get the position of the bones
    

}

GeneralRotule::GeneralRotule(std::string name, Bone* boneFather, Bone* boneChild, vec3 axis, float maxAngle)
    : Joint(name, boneFather, boneChild), axis(axis), maxAngle(maxAngle) {}


void GeneralRotule::set(vec3 axis, float maxAngle)
{
    this->axis = axis;
    this->maxAngle = maxAngle;
}

void GeneralRotule::applyConstraint(Skeleton *skeleton)
{
    //get the position of the child bone 
    vec3 childVector = boneChild->end - boneChild->start;

    //get the angle between the child bone and the axis
    float angle = angleBetween(childVector, axis);
    //check if the angle is greater than the max angle
    if (angle > maxAngle) {
       //rotate the child bone aroud the cross product of the child vector and the axis
        vec3 rotationAxis = normalize(cross(childVector, axis));
        //rotate the child bone
        float angleToRotate = angle - maxAngle;
        mat3 rotationMatrix = mat3::build_rotation_from_axis_angle(rotationAxis, angleToRotate);
        vec3 newChildVector = rotationMatrix * childVector;

        boneChild->setEnd(boneChild->start + newChildVector);
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



