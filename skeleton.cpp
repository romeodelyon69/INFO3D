#include "skeleton.hpp"
#include "Nmath.hpp"

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

void Bone::draw(environment_structure& environment)
{   
    if (cgp::norm(end - start) < 0.01f) {
        return;
    }
    // Draw the bone
    this->bone.vbo_position.update(numarray<cgp::vec3>{start, end});
    cgp::draw(this->bone, environment);

}



void KinematicChain::initialize()
{
    articulation.initialize_data_on_gpu(cgp::mesh_primitive_sphere(0.05f));
    articulation.material.color = cgp::vec3(0.5f,0.5f,0.5f);

    for (auto& bone : bones) {
        bone.initialize();
    }


}

void KinematicChain::addBone(Bone bone)
{
    bones.push_back(bone);
}
void KinematicChain::addJoint(Joint* joint)
{
    joints.push_back(joint);
}

Bone* KinematicChain::getBone(std::string name)
{
    for (auto& bone : bones) {
        if (bone.name == name) {
            return &bone;
        }
    }
    return nullptr;
}

void KinematicChain::draw(environment_structure& environment)
{
    // Draw the bones and articulations
    for (auto& bone : bones) {
        bone.draw(environment);
        articulation.model.translation = bone.end;
        cgp::draw(articulation, environment);
        articulation.model.translation = bone.start;
        cgp::draw(articulation, environment);
    }
}

void KinematicChain::forwardFabrik(std::vector<Bone*> bonesOrder, vec3 target, float tolerance)
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
    

void KinematicChain::backwardFabrik(std::vector<Bone*> bonesOrder, vec3 target, float tolerance)
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
void KinematicChain::forwardApplyConstraints(std::vector<Bone*> bonesOrder)
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

void KinematicChain::backwardApplyConstraints(std::vector<Bone*> bonesOrder)
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
    Bone* left_hand = getBone("left_hand");
    Bone* left_forearm = getBone("left_forearm");

    //calculons l'angle entre left_hand et left_forearm
    float angle = angleBetween(left_hand->direction, left_forearm->direction);
    

    //std::cout << "#########################################################################" << std::endl;
    //std::cout << "Angle between left_hand and left_forearm: " << angle << std::endl;
}

void KinematicChain::fabrik(Bone* targetBone, Bone* fixedBone, vec3 target, float tolerance, int max_iter)
{
    // Implementation of the FABRIK algorithm
    // This function will take to bones, the target position and the tolerance
    // It will try to move the end of the first bone to the target position 
    // and will not move the start of the second

    //on va construire un vecteur de pointeur sur les bones pour les parcourir à l'endroit puis à l'envers

    
    std::vector<Bone*> bonesOrder;
    Bone* currentBone = targetBone;
    bonesOrder.push_back(currentBone);
    //on remonte l'arbre jusqu'à targetBone
    while (currentBone != nullptr && currentBone->name != fixedBone->name) {
        currentBone = currentBone->jointFather->boneFather;
        bonesOrder.push_back(currentBone);
    }
    

    cgp::vec3 fixedPoint = fixedBone->start;
    for (int i = 0; i < max_iter; i++) {
    
        //forward FABRIK
        forwardFabrik(bonesOrder, target, tolerance);
        //constraint
        forwardApplyConstraints(bonesOrder);
        //backward FABRIK
        backwardFabrik(bonesOrder, fixedPoint, tolerance);
        //constraint
        backwardApplyConstraints(bonesOrder);
        
        
        //check if the target bone is close enough to the target
        if (cgp::norm(targetBone->end - target) < tolerance) {
            break;
        }
    }

    //std::cout << "FABRIK finished" << std::endl;
    //we are going to verify that the constraint for the hand is respected
    Bone* left_hand = getBone("left_hand");
    Bone* left_forearm = getBone("left_forearm");

    //calculons l'angle entre left_hand et left_forearm
    float angle = angleBetween(left_hand->direction, left_forearm->direction);
    //std::cout << "Angle between left_hand and left_forearm: " << angle << std::endl;
}

void KinematicChain::fabrik(std::string targetBoneName, std::string fixedBoneName, vec3 target, float tolerance, int max_iter)
{
    //implementation that use bone names instead of pointers, easier to use
    // Implementation of the FABRIK algorithm
    // This function will take to bones, the target position and the tolerance
    // It will try to move the end of the first bone to the target position 
    // and will not move the start of the second

    Bone* targetBone = getBone(targetBoneName);
    Bone* fixedBone = getBone(fixedBoneName);

    if (targetBone == nullptr) {
        std::cerr << "Error: Bone 'targetBone' not found." << std::endl;
        return;
    }
    if (targetBone->jointFather == nullptr) {
        std::cerr << "Error: jointFather is null for bone 'left_hand'." << std::endl;
        return;
    }

    if (targetBone != nullptr && fixedBone != nullptr) {
        fabrik(targetBone, fixedBone, target, tolerance, max_iter);
    } else {
        std::cerr << "Error: Bone not found" << std::endl;
    }
}

void HumanSkeleton::addJoint(std::string jointName, std::string boneFatherName, std::string boneChildName) {
    Bone* boneFather = getBone(boneFatherName);
    Bone* boneChild = getBone(boneChildName);

    if (boneFather != nullptr && boneChild != nullptr) {
        // Create the joint
        Joint* joint = new Joint(jointName, boneFather, boneChild);
        // Add the joint to the skeleton
        joints.push_back(joint);

        boneFather->jointChild = joint;
        boneChild->jointFather = joint;

        std::cout << "Joint " << jointName << " added between " << boneFatherName << " and " << boneChildName << std::endl;
        std::cout << "Joint name: " << joint->name << std::endl;
    } else {
        std::cerr << "Error: Joint not created, missing bones: "
                  << boneFatherName << " or " << boneChildName << std::endl;
    }
}

void HumanSkeleton::addParentRotule(std::string jointName, std::string boneFatherName, std::string boneChildName, float maxAngle) {
    Bone* boneFather = getBone(boneFatherName);
    Bone* boneChild = getBone(boneChildName);

    if (boneFather != nullptr && boneChild != nullptr) {
        // Create the joint
        Joint* joint = new ParentRotule(jointName, boneFather, boneChild, maxAngle);
        // Add the joint to the skeleton
        joints.push_back(joint);

        boneFather->jointChild = joint;
        boneChild->jointFather = joint;

        std::cout << "Joint " << jointName << " added between " << boneFatherName << " and " << boneChildName << std::endl;
    } else {
        std::cerr << "Error: Joint not created, missing bones: "
                  << boneFatherName << " or " << boneChildName << std::endl;
    }
}



void HumanSkeleton::initialize()
{
    //initialize the bones
    //head
    addBone(Bone("left_foot", scale * vec3{-FEETSPACING/2,0,0}, scale * vec3{-FEETSPACING/2-FOOT,0,0}));
    addBone(Bone("right_foot",  scale * vec3{FEETSPACING/2,0,0},  scale * vec3{FEETSPACING/2+FOOT,0,0}));

    vec3 left_knee = vec3{-FEETSPACING/2 + std::sin(LEGANGLE)*TIBIA,0,std::cos(LEGANGLE)*TIBIA};
    vec3 right_knee = vec3{FEETSPACING/2 - std::sin(LEGANGLE)*TIBIA,0,std::cos(LEGANGLE)*TIBIA};

    addBone(Bone("left_tibia",   scale * left_knee, scale * vec3{-FEETSPACING/2,0,0}));
    addBone(Bone("right_tibia", scale * right_knee,  scale * vec3{FEETSPACING/2,0,0}));

    vec3 left_hip = left_knee + vec3{+std::sin(LEGANGLE)*THIGH,0,std::cos(LEGANGLE)*THIGH};
    vec3 right_hip = right_knee + vec3{-std::sin(LEGANGLE)*THIGH,0,std::cos(LEGANGLE)*THIGH};
    addBone(Bone("left_thigh",  scale * left_hip, scale * left_knee));
    addBone(Bone("right_thigh",  scale * right_hip,  scale * right_knee));

    vec3 navel = (left_hip + right_hip)/2 + vec3{0,0,HIP};
    addBone(Bone("left_hip", scale * navel,  scale * left_hip));
    addBone(Bone("right_hip", scale * navel, scale * right_hip));

    vec3 neck = navel + vec3{0,0,TORSO};
    addBone(Bone("body",  scale * navel,  scale * neck));

    vec3 left_shoulder = neck + vec3{-std::cos(ARMANGLE)*SHOULDER,0,-std::sin(ARMANGLE)*SHOULDER};
    vec3 right_shoulder = neck + vec3{std::cos(ARMANGLE)*SHOULDER,0,-std::sin(ARMANGLE)*SHOULDER};
    addBone(Bone("left_shoulder",  scale * neck,  scale * left_shoulder));
    addBone(Bone("right_shoulder",  scale * neck,  scale * right_shoulder));

    vec3 left_elbow = left_shoulder + vec3{-std::cos(ARMANGLE)*UPPER_ARM,0,-std::sin(ARMANGLE)*UPPER_ARM};
    vec3 right_elbow = right_shoulder + vec3{std::cos(ARMANGLE)*UPPER_ARM,0,-std::sin(ARMANGLE)*UPPER_ARM};
    addBone(Bone("left_upper_arm",  scale * left_shoulder,  scale * left_elbow));
    addBone(Bone("right_upper_arm",  scale * right_shoulder,  scale * right_elbow));

    vec3 left_wrist = left_elbow + vec3{-std::cos(ARMANGLE)*FOREARM,0,-std::sin(ARMANGLE)*FOREARM};
    vec3 right_wrist = right_elbow + vec3{std::cos(ARMANGLE)*FOREARM,0,-std::sin(ARMANGLE)*FOREARM};
    addBone(Bone("left_forearm",  scale * left_elbow,  scale * left_wrist));
    addBone(Bone("right_forearm",  scale * right_elbow,  scale * right_wrist));

    addBone(Bone("left_hand",  scale * left_wrist,  scale * (left_wrist - vec3{HAND,0,0})));
    addBone(Bone("right_hand",  scale * right_wrist,  scale * (right_wrist + vec3{HAND,0,0})));

    addBone(Bone("neck",  scale * neck,  scale * (neck + vec3{0,0,HEAD})));

    //initialize the joints

    //spine
    addJoint("Neck", "body", "neck");
    
    //arms
    addParentRotule("LeftShoulder", "left_shoulder", "left_upper_arm", 0.5f);
    addJoint("RightShoulder", "right_shoulder", "right_upper_arm");
    addJoint("LeftElbow", "left_upper_arm", "left_forearm");
    addJoint("RightElbow", "right_upper_arm", "right_forearm");
    addParentRotule("LeftWrist", "left_forearm", "left_hand", 0.5f);
    addParentRotule("RightWrist", "right_forearm", "right_hand", 0.5f);

    //legs
    addJoint("LeftHip", "left_hip", "left_thigh");
    addJoint("RightHip", "right_hip", "right_thigh");
    addJoint("LeftKnee", "left_thigh", "left_tibia");
    addJoint("RightKnee", "right_thigh", "right_tibia");
    addJoint("LeftAnkle", "left_tibia", "left_foot");
    addJoint("RightAnkle", "right_tibia", "right_foot");

    //initialize the skeleton
    KinematicChain::initialize();
    std::cout << "Skeleton initialized" << std::endl;

}