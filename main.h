#pragma once

// perhaps switch to using forward declarations since we have references only
#include <string>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/rigid_body.h>

struct ArmWithCup {
    const drake::multibody::RevoluteJoint<double>* shoulder;
    const drake::multibody::RevoluteJoint<double>* elbow;
    const drake::multibody::RigidBody<double>* cup_body;
};

ArmWithCup AddDoubleLinkArmWithCup(
    drake::multibody::MultibodyPlant<double>* mbp,
    const std::string& name_prefix,
    const drake::math::RigidTransformd& X_WShoulder,
    double link_length,
    double link_radius,
    double link_mass,
    double cup_radius,
    double cup_height
);
