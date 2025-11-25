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
    const drake::multibody::RevoluteJoint<double>* wrist;
    const drake::multibody::RigidBody<double>* cup_body;
};

ArmWithCup AddTripleLinkArmWithCup(
    drake::multibody::MultibodyPlant<double>* mbp,
    const std::string& name_prefix,
    const drake::math::RigidTransformd& X_WShoulder,
    double link_length,
    double link_radius,
    double link_mass,
    double cup_radius,
    double cup_height
);

Eigen::VectorXd SolveIKForCup(
    const drake::multibody::MultibodyPlant<double> &plant,
    drake::systems::Context<double>* plant_context,
    const drake::multibody::RigidBody<double>* cup_body,
    const Eigen::Vector3d& p_Wcup_target
);

Eigen::Vector3d CupPos1(
    double t,
    Eigen::Vector3d Center1,
    double radius
);

Eigen::Vector3d CupPos2(
    double t,
    Eigen::Vector3d Center2,
    double radius
);

struct ArmIKSolution {
    bool success;
    double yaw;
    double shoulder;
    double elbow;
    double wrist;
};

ArmIKSolution SolveAnalyticIK(
    double x,
    double y,
    double z,
    double L1,
    double L2,
    double L3
);