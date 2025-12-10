#pragma once

// perhaps switch to using forward declarations since we have references only
#include <string>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/rigid_body.h>
#include <drake/systems/framework/context.h>
#include <Eigen/Core>

struct JointTarget {
    std::string name;
    double angle;
};

struct BallState {
    const drake::multibody::RigidBody<double>* ball_body;
    int arm_index; // arm balls belong to
    bool ball_caught;
    double catch_time;
    double throw_release_time;
    Eigen::Vector3d throw_velocity;
    double throw_flight_time;
    bool firstThrow;
};

std::string ReplaceNameInSdf(
    std::string sdf_string,
    const std::string& type_of_name,
    int length_of_name,
    const std::string& model_name
);

const drake::multibody::RigidBody<double>* BuildObj(
    drake::multibody::MultibodyPlant<double>* mbp,
    const std::string& name,
    const std::string& sdf_path
);

const drake::multibody::RigidBody<double>* BuildBall(
    drake::multibody::MultibodyPlant<double>* mbp,
    const std::string& name = "ball"
);

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
    const drake::math::RigidTransformd& X_FM_link1,
    double link_length,
    double link_radius,
    double link_mass,
    double cup_radius,
    double cup_height
);

struct ThreeLinkIKSolution {
    bool success{};
    double shoulder{};
    double elbow{};
    double wrist{};
};

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

std::pair<Eigen::Vector3d, double> CalculateThrowVelocityAndTime(
    const Eigen::Vector3d& release_pos,
    double angle_at_release,
    double target_z,
    double cup_radius,
    double torso_w,
    int num_rotations,
    int num_arms,
    const Eigen::Vector3d& g
);