#include "animator.h"
#include <cassert>

namespace USTC_CG::character_animation {

using namespace pxr;

Joint::Joint(int idx,string name,int parent_idx,const GfMatrix4f& bind_transform)
    : idx_(idx),name_(name),parent_idx_(parent_idx),bind_transform_(bind_transform)
{}

void Joint::compute_world_transform()
{
    // Recursively compute world space transforms for this joint and its children
    // For each child:
    // 1. Combine child's local transform with current joint's world transform
    // 2. Propagate computation to child's subtree
    for(int i = 0; i < children_.size(); i++) {
        auto child = children_[i];
        child->world_transform_ = child->local_transform_ * world_transform_;
        child->compute_world_transform();
    }
}

void JointTree::compute_world_transforms_for_each_joint()
{
    // Initialize the transformation chain from the root joint
    // 1. Set root's world transform as its local transform
    // 2. Propagate transforms through the entire joint hierarchy
    root_->world_transform_ = root_->local_transform_;
    root_->compute_world_transform();
}

void JointTree::add_joint(int idx,std::string name,int parent_idx,const GfMatrix4f& bind_transform)
{
    auto joint = make_shared<Joint>(idx,name,parent_idx,bind_transform);
    joints_.push_back(joint);
    if(parent_idx < 0) {
        root_ = joint;
    } else {
        joints_[parent_idx]->children_.push_back(joint);

        if(parent_idx < joints_.size())
            joint->parent_ = joints_[parent_idx];
        else {
            std::cout << "[add_joint_error] parent_idx out of range" << std::endl;
            exit(1);
        }
    }
}

void JointTree::update_joint_local_transform(const VtArray<GfMatrix4f>& new_local_transforms)
{
    assert(new_local_transforms.size() == joints_.size());

    for(int i = 0; i < joints_.size(); ++i) {
        joints_[i]->local_transform_ = new_local_transforms[i];
    }
}

void JointTree::print()
{
    for(auto joint_ptr : joints_) {
        std::cout << "Joint idx: " << joint_ptr->idx_ << " name: " << joint_ptr->name_ << " parent_idx: " << joint_ptr->parent_idx_ << std::endl;
    }
}


Animator::Animator(const shared_ptr<MeshComponent> mesh,const shared_ptr<SkelComponent> skel)
    : mesh_(mesh),
    skel_(skel)
{
    auto joint_order = skel_->jointOrder;
    auto topology = skel_->topology;
    for(size_t i = 0; i < joint_order.size(); ++i) {
        SdfPath jointPath(joint_order[i]);

        string joint_name = jointPath.GetName();
        int parent_idx = topology.GetParent(i);

        joint_tree_.add_joint(i,joint_name,parent_idx,GfMatrix4f(skel->bindTransforms[i]));
    }

    joint_tree_.print();
}

void Animator::step(const shared_ptr<SkelComponent> skel)
{
    joint_tree_.update_joint_local_transform(skel->localTransforms);

    joint_tree_.compute_world_transforms_for_each_joint();

    update_mesh_vertices();
}

void Animator::update_mesh_vertices()
{
    // Update vertex positions based on skeletal animation
    // 1. Get joint influences (indices and weights) for each vertex
    // 2. For each vertex:
    //    - Transform from bind pose to current pose
    //    - Apply weighted influence from each affecting joint
    //    - Accumulate final position
    const auto& indices = skel_->jointIndices;
    const auto& weights = skel_->jointWeight;
    auto vertices = mesh_->get_vertices();

    int joints_per_vertex = indices.size() / vertices.size();

    for(int vertex_idx = 0; vertex_idx < vertices.size(); vertex_idx++) {
        GfVec3f final_position{0,0,0};

        for(int joint_influence = 0; joint_influence < joints_per_vertex; joint_influence++) {
            auto joint = joint_tree_.get_joint(indices[vertex_idx * joints_per_vertex + joint_influence]);
            float weight = weights[vertex_idx * joints_per_vertex + joint_influence];

            // Transform vertex: bind_pose -> joint_local -> world_space
            final_position += weight * joint->get_world_transform().TransformAffine(
                joint->get_bind_transform().GetInverse().TransformAffine(vertices[vertex_idx])
            );
        }

        vertices[vertex_idx] = final_position;
    }

    mesh_->set_vertices(vertices);
}

}  // namespace USTC_CG::character_animation
