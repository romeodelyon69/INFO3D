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

void Joint::applyConstraintOnChild()
{
    cgp::vec3 translation =  boneFather->getEnd() - boneChild->getStart();
    boneChild->translate(translation);
    if(cgp::norm(translation) > 0.01f)
    {
        std::cerr << "Error: Joint " << name << " is not working properly" << std::endl;
    }

}
void Joint::applyConstraintOnFather()
{
    cgp::vec3 translation =  boneChild->getStart() - boneFather->getEnd();
    boneFather->translate(translation);
    if(cgp::norm(translation) > 0.01f)
    {
        std::cerr << "Error: Joint " << name << " is not working properly" << std::endl;
    }
}



void ParentRotule::applyConstraintOnChild()
{   
    Joint::applyConstraintOnChild();
    
    cgp::vec3 axis = boneFather->getDirection();
    cgp::vec3 direction = boneChild->getDirection();
    float angle = angleBetween(direction, axis);

    if (angle > maxAngle) {
        vec3 rotationAxis = normalize(cross(direction, axis));
        float angleToRotate = angle - maxAngle;
        cgp::rotation_transform rotationMatrix = cgp::rotation_transform::from_axis_angle(rotationAxis, angleToRotate);
        boneChild->frameAbsolut = rotationMatrix * boneChild->frameAbsolut;
    }
}

void ParentRotule::applyConstraintOnFather()
{
    Joint::applyConstraintOnFather();

    cgp::vec3 axis = boneChild->getDirection();
    cgp::vec3 direction = boneFather->getDirection();
    float angle = angleBetween(direction, axis);

    if (angle > maxAngle) {
        vec3 rotationAxis = normalize(cross(direction, axis));
        float angleToRotate = angle - maxAngle;
        cgp::rotation_transform rotationMatrix = cgp::rotation_transform::from_axis_angle(rotationAxis, angleToRotate);
        boneFather->frameAbsolut = rotationMatrix * boneFather->frameAbsolut;
    }
}


ParentRotule::ParentRotule(std::string name, Bone* boneFather, Bone* boneChild, float maxAngle)
    : Joint(name, boneFather, boneChild) // Explicitly call the Joint constructor
{
    this->maxAngle = maxAngle;
}

void ParentRotule::setAngle(float maxAngle)
{
    this->maxAngle = maxAngle;
}


Hinge::Hinge(std::string name, Bone* boneFather, Bone* boneChild, float maxAngle)
    : Joint(name, boneFather, boneChild) // Explicitly call the Joint constructor
{
    this->maxAngle = maxAngle;
}
void Hinge::setAngle(float maxAngle)
{
    this->maxAngle = maxAngle;
}
void Hinge::applyConstraintOnChild()
{
    Joint::applyConstraintOnChild();
    
    //on veut veut que (O,xfather, zFather)

    cgp::vec3 xFather = boneFather->getX();
    cgp::vec3 xChild = boneChild->getX();
    cgp::vec3 zChild = boneChild->getZ();

    cgp::rotation_transform rotation = cgp::rotation_transform::from_vector_transform(xChild, xFather);
    //boneChild->frameAbsolut = rotation * boneChild->frameAbsolut;
}

void Hinge::applyConstraintOnFather()
{
    Joint::applyConstraintOnFather();
    
    //on va faire matcher les axes x de boneFather et boneChild en faisant tourner boneFather autour de son axe z 
    //ainsi l'os reste en position on le fait juste trouner

    cgp::vec3 xFather = boneFather->getX();
    cgp::vec3 xChild = boneChild->getX();
    cgp::vec3 zFather = boneFather->getZ();

    cgp::rotation_transform rotation = cgp::rotation_transform::from_vector_transform(xFather, xChild);
    boneFather->frameAbsolut = rotation * boneFather->frameAbsolut;
}





