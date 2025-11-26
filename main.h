#pragma once

// perhaps switch to using forward declarations since we have references only
#include <string>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/rigid_body.h>
#include <Eigen/Core>

struct ArmWithCup {
    const drake::multibody::RevoluteJoint<double>* shoulder;
    const drake::multibody::RevoluteJoint<double>* elbow;
    const drake::multibody::RigidBody<double>* cup_body;
};

ArmWithCup AddTripleLinkArmWithCup(
    drake::multibody::MultibodyPlant<double>* mbp,
    const std::string& name_prefix,
    const drake::multibody::Body<double>& parent_body,
    const drake::math::RigidTransformd& X_PShoulder,
    double link_length,
    double link_radius,
    double link_mass,
    double cup_radius,
    double cup_height
);

Eigen::Vector3d ObjectPosition(
    const Eigen::Vector3d& p_r,
    const Eigen::Vector3d& v0,
    double t,
    double t_r,
    const Eigen::Vector3d& g
);

Eigen::Vector3d ObjectVelocity(
    const Eigen::Vector3d& v0,
    double t,
    double t_r,
    const Eigen::Vector3d& g
);

Eigen::Vector3d ComputeThrowVelocity(
    const Eigen::Vector3d& p_r,
    const Eigen::Vector3d& p_c,
    double t_r,
    double t_c,
    const Eigen::Vector3d& g
);

Eigen::Vector3d CupTorsoTarget(
    double t,
    const Eigen::Vector3d& shoulder_T,
    double link_length,
    double h_mid_offset,
    double h_amp,
    double period,
    double arm_phase
);

struct TwoLinkIKSolution {
    bool success{};
    double shoulder{};
    double elbow{};
};

TwoLinkIKSolution Solve2LinkIK(
    const Eigen::Vector3d& p_W,
    double theta_torso,
    const Eigen::Vector3d& shoulder_T,
    double L1,
    double L2
);