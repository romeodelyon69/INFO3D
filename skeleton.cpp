#include "skeleton.hpp"

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

void Bone::draw(environment_structure& environment)
{   
    if (cgp::norm(end - start) < 0.01f) {
        return;
    }
    // Draw the bone
    //this->bone.vbo_position.update(numarray<cgp::vec3>{start, end});

    //c'est horrible de faire ca mais je ne comprends pas pourquoi la ligne au dessus ne fonctionne pas
    //compile mais crash à l'execution
    bone.clear();
    bone.initialize_data_on_gpu({start,end});
    cgp::draw(this->bone, environment);

}



void Skeleton::initialize()
{
    articulation.initialize_data_on_gpu(cgp::mesh_primitive_sphere(0.05f));
    articulation.material.color = cgp::vec3(0.5f,0.5f,0.5f);

    for (auto& bone : bones) {
        bone.initialize();
    }
}

void Skeleton::addBone(Bone bone)
{
    bones.push_back(bone);
}
void Skeleton::addJoint(Joint& joint)
{
    joints.push_back(joint);
}

Bone* Skeleton::getBone(std::string name)
{
    for (auto& bone : bones) {
        if (bone.name == name) {
            return &bone;
        }
    }
    return nullptr;
}

void Skeleton::draw(environment_structure& environment)
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

void Skeleton::forwardFabrik(Bone* targetBone, Bone* fixedBone, vec3 target, float tolerance)
{
    // Implementation of the FABRIK algorithm
    // This function will take to bones, the target position and the tolerance
    // It will try to move the end of the first bone to the target position 
    // and will not move the start of the second

    //on procède de facon recursive jusqu'à ce que targetBone = fixedBone 
    //dans ce cas on fait une dernière itération
    
    //on va process targetBone
    float length = targetBone->length;
    std::cout << "length avant update = " << length << std::endl;
    targetBone->setEnd(target);
   
    vec3 tragetStart = targetBone->end - targetBone->direction * length;
    targetBone->setStart(tragetStart);
    std::cout << "length après update = " << targetBone->length << std::endl;
    
    std::cout << "end = " << targetBone->end << std::endl;
    std::cout << "start = " << targetBone->start << std::endl;
    if(targetBone->name == fixedBone->name) {
        return;
    }
    
    //on appelle recursivement la fonction sur le parent
    forwardFabrik(targetBone->jointFather->boneFather, fixedBone, targetBone->start, tolerance);
}

void Skeleton::backwardFabrik(Bone* targetBone, Bone* fixedBone, vec3 target, float tolerance)
{
    // Implementation of the FABRIK algorithm
    // This function will take to bones, the target position and the tolerance
    // It will try to move the end of the first bone to the target position 
    // and will not move the start of the second

    //on procède de facon recursive jusqu'à ce que targetBone = fixedBone 
    //dans ce cas on fait une dernière itération

    
    //on va process targetBone
    float length = targetBone->length;
    targetBone->setStart(target);
    vec3 tragetEnd = targetBone->start + targetBone->direction * length;
    targetBone->setEnd(tragetEnd);
    if(targetBone->name == fixedBone->name) {

        return;
    }
    //on appelle recursivement la fonction sur le parent
    backwardFabrik(targetBone->jointChild->boneChild, fixedBone, targetBone->end, tolerance);
}
void Skeleton::applyConstraints(Bone* targetBone, Bone* fixedBone)
{
    // Implementation of the FABRIK algorithm
    // This function recusively apply the constraints to the skeleton

    //on procède de facon recursive jusqu'à ce que targetBone = fixedBone 
    //dans ce cas on fait une dernière itération

    //on va process targetBone
    if (targetBone->jointFather != nullptr) {
        targetBone->jointFather->applyConstraint(this);
    }

    if(targetBone->name == fixedBone->name) {
        return;
    }

    //on appelle recursivement la fonction sur le parent
    applyConstraints(targetBone->jointFather->boneFather, fixedBone);
}

void Skeleton::fabrik(Bone* targetBone, Bone* fixedBone, vec3 target, float tolerance, int max_iter)
{
    // Implementation of the FABRIK algorithm
    // This function will take to bones, the target position and the tolerance
    // It will try to move the end of the first bone to the target position 
    // and will not move the start of the second

    cgp::vec3 fixedPoint = fixedBone->start;
    for (int i = 0; i < max_iter; i++) {
        std::cout << "Iteration " << i << std::endl;
        std::cout << "target = " << target << std::endl;
        //forward FABRIK
        forwardFabrik(targetBone, fixedBone, target, tolerance);
        //constraint
        applyConstraints(targetBone, fixedBone);
        //backward FABRIK
        backwardFabrik(fixedBone, targetBone, fixedPoint, tolerance);
        //constraint
        std::cout << "on passe là ? " << std::endl;
        
        //check if the target bone is close enough to the target
        if (cgp::norm(targetBone->end - target) < tolerance) {
            break;
        }
    }
}

void Skeleton::fabrik(std::string targetBoneName, std::string fixedBoneName, vec3 target, float tolerance, int max_iter)
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
        joints.emplace_back(jointName, boneFather, boneChild);
        Joint& joint = joints.back();

        boneFather->jointChild = &joint;
        boneChild->jointFather = &joint;

        std::cout << "Joint " << jointName << " added between " << boneFatherName << " and " << boneChildName << std::endl;
        std::cout << "Joint name: " << joint.name << std::endl;
    } else {
        std::cerr << "Error: Joint not created, missing bones: "
                  << boneFatherName << " or " << boneChildName << std::endl;
    }
}

void HumanSkeleton::initialize()
{
    //initialize the skeleton
    Skeleton::initialize();

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
    addJoint("LeftShoulder", "left_shoulder", "left_upper_arm");
    addJoint("RightShoulder", "right_shoulder", "right_upper_arm");
    addJoint("LeftElbow", "left_upper_arm", "left_forearm");
    addJoint("RightElbow", "right_upper_arm", "right_forearm");
    addJoint("LeftWrist", "left_forearm", "left_hand");
    addJoint("RightWrist", "right_forearm", "right_hand");

    //legs
    addJoint("LeftHip", "left_hip", "left_thigh");
    addJoint("RightHip", "right_hip", "right_thigh");
    addJoint("LeftKnee", "left_thigh", "left_tibia");
    addJoint("RightKnee", "right_thigh", "right_tibia");
    addJoint("LeftAnkle", "left_tibia", "left_foot");
    addJoint("RightAnkle", "right_tibia", "right_foot");

    std::cout << "Skeleton initialized" << std::endl;
    Bone* leftHand = getBone("left_hand");
  
    std::cout << "RIGHT HIP" << getBone("right_hip")->start << "%%%%%%%%%%%" << std::endl;
    cgp::vec3 target = { -1.327092f + std::cos(0.2f) * 0.2 , 0.9f, 2.011435f -  0.4};
	fabrik("left_hand", "left_shoulder", target, 0.001f, 10);
    std::cout << "giga chad" << std::endl;

    target = {1,0.4,1};
    std::cout << "RIGHT HIP" << getBone("right_hip")->start << "%%%%%%%%%%%" << std::endl;
    fabrik("right_foot", "right_hip", target, 0.001f, 100);
    std::cout << "RIGHT HIP" << getBone("right_hip")->start << "%%%%%%%%%%%" << std::endl;

    std::cout << "nombre d'os = " << bones.size() << std::endl;
    std::cout << "nombre d'articulations = " << joints.size() << std::endl;

    //afichons les coordonées de la main gauche
    std::cout << "left hand = " << leftHand->end << std::endl;

}