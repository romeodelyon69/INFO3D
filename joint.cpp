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
    if(cgp::norm(translation) > 0.05f)
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
    
    //on veut dans un premier temps que zChild soit dans le plan (O,xfather, zFather)

    cgp::vec3 xFather = boneFather->getX();
    cgp::vec3 yFather = boneFather->getY();
    cgp::vec3 zFather = boneFather->getZ();
    cgp::vec3 xChild = boneChild->getX();
    cgp::vec3 yChild = boneChild->getY();
    cgp::vec3 zChild = boneChild->getZ();

    //on va projeter zChild sur le plan (O, xFather, zFather)
    cgp::vec3 zChildProjzFather = cgp::dot(zChild, zFather) * zFather; // ||zFather||=1
    cgp::vec3 zChildProjxFather = cgp::dot(zChild, xFather) * xFather; // ||xFather||=1

    cgp::vec3 zChildProjection = zChildProjzFather + zChildProjxFather;

    if(cgp::norm(zChildProjection) < 0.5){
        //ce n'est pas censé arriver si l'on fait varier par petit pat nos éléments 
        std::cout<<"Hinge/applyConstraintOnChild a rencontré un problème, on le résout brutalement, à surveiller"<<std::endl;
        cgp::rotation_transform rotation = cgp::rotation_transform::from_vector_transform(xChild, xFather);
        boneChild->frameAbsolut = rotation * boneChild->frameAbsolut;
        return; 
    }

    zChildProjection = cgp::normalize(zChildProjection);
    std::cout<<"Hinge Child :"<<cgp::dot(yFather, zChildProjection)<<std::endl;
    std::cout<<"Hinge Child fram orthogonalité : "<<cgp::dot(yChild, zChild)<<std::endl;
    cgp::rotation_transform rotation = cgp::rotation_transform::from_frame_transform(yChild, zChild, yFather, zChildProjection);
    //boneChild->frameAbsolut = rotation * boneChild->frameAbsolut;
    std::cout<<"On a passé la transfo Child"<<std::endl;
    return;
}

void Hinge::applyConstraintOnFather()
{
    Joint::applyConstraintOnFather();
    
    //on veut dans un premier temps que zFather soit dans le plan (O,xChild, zChild)

    cgp::vec3 xFather = boneFather->getX();
    cgp::vec3 yFather = boneFather->getY();
    cgp::vec3 zFather = boneFather->getZ();
    cgp::vec3 xChild = boneChild->getX();
    cgp::vec3 yChild = boneChild->getY();
    cgp::vec3 zChild = boneChild->getZ();

    //on va projeter zFather sur le plan (O, xChild, zChild)
    cgp::vec3 zFatherProjzChild = cgp::dot(zChild, zFather) * zChild; // ||zChild||=1
    cgp::vec3 zFatherProjxChild = cgp::dot(zFather, xChild) * xChild; // ||xChild||=1

    cgp::vec3 zFatherProjection = zFatherProjzChild + zFatherProjxChild;

    if(cgp::norm(zFatherProjection) < 0.5){
        //ce n'est pas censé arriver si l'on fait varier par petit pat nos éléments 
        std::cout<<"Hinge/applyConstraintOnFather a rencontré un problème, on le résout brutalement, à surveiller"<<std::endl;
        cgp::rotation_transform rotation = cgp::rotation_transform::from_vector_transform(xFather, xChild);
        boneFather->frameAbsolut = rotation * boneFather->frameAbsolut;
        return; 
    }

    zFatherProjection = cgp::normalize(zFatherProjection);
    std::cout<<"Hinge Father projection orthogonalité : "<< cgp::dot(yChild, zFatherProjection)<<std::endl;
    std::cout<<"Hinge Father fram orthogonalité : "<<cgp::dot(yFather, zFather)<<std::endl;
    cgp::rotation_transform rotation = cgp::rotation_transform::from_frame_transform(yFather, zFather, yChild, zFatherProjection);
    //boneFather->frameAbsolut = rotation * boneFather->frameAbsolut;   
    std::cout<<"On a passé la transfo Father"<<std::endl;
    return;
}





