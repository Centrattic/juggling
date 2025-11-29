#pragma once

// perhaps switch to using forward declarations since we have references only
#include <string>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/rigid_body.h>
#include <Eigen/Core>

struct JointTarget {
    std::string name;
    double angle;
};

struct ArmWithCup {
    const drake::multibody::RevoluteJoint<double>* shoulder;
    const drake::multibody::RevoluteJoint<double>* elbow;
    const drake::multibody::RevoluteJoint<double>* wrist;
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

struct ThreeLinkIKSolution {
    bool success{};
    double shoulder{};
    double elbow{};
    double wrist{};
};

TwoLinkIKSolution Solve2LinkIK(
    const Eigen::Vector3d& p_W,
    double theta_torso,
    const Eigen::Vector3d& shoulder_T,
    double L1,
    double L2
);

TwoLinkIKSolution Solve2LinkIKWithOrientation(
    const Eigen::Vector3d& cup_pos_W,
    const Eigen::Vector3d& cup_direction_W, // Desired cup orientation
    double theta_torso,
    const Eigen::Vector3d& shoulder_T,
    double L1,
    double L2
);

ThreeLinkIKSolution Solve3LinkIKWithOrientation(
    const Eigen::Vector3d& cup_pos_W,
    const Eigen::Vector3d& cup_direction_W, // Desired cup orientation
    double theta_torso,
    const Eigen::Vector3d& shoulder_T,
    double L1,
    double L2,
    double L3
);

ThreeLinkIKSolution SimpleKinematicsSolution(
    const Eigen::Vector2d& cup_pos_W, // radius, z
    double horizontal_cup_angle,
    double torso_height,
    double L1,
    double L2,
    double L3
);

std::pair<double, double> CalculateDropHeightAndTime(
    double cup_radius,
    double cup_z,
    double torso_w,
    const Eigen::Vector3d& g
);