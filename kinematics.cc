#include "main.h"
#include <Eigen/Core>
#include <cmath>
#include <iostream>

#include <drake/math/rigid_transform.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/tree/rigid_body.h>
#include <drake/solvers/solve.h>

using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;

// To Do: create consts .cc file

/* Object kinematics */

Eigen::Vector3d ObjectPosition(
    const Eigen::Vector3d& p_r,
    const Eigen::Vector3d& v0,
    double t,
    double t_r,
    const Eigen::Vector3d& g
) {
    double dt = t - t_r;
    return p_r + v0 * dt + 0.5 * g * dt * dt;
}

Eigen::Vector3d ObjectVelocity(
    const Eigen::Vector3d& v0,
    double t,
    double t_r,
    const Eigen::Vector3d& g
) {
    double dt = t - t_r;
    return v0 + g * dt;
}

Eigen::Vector3d ComputeThrowVelocity(
    const Eigen::Vector3d& p_r,
    const Eigen::Vector3d& p_c,
    double t_r,
    double t_c,
    const Eigen::Vector3d& g
) {
    double dt = t_c - t_r;
    return (p_c - p_r - 0.5 * g * dt * dt) / dt;
}

Eigen::Vector3d CupTorsoTarget(
    double t,
    const Eigen::Vector3d& shoulder_T,
    double link_length,
    double h_mid_offset,
    double h_amp,
    double period,
    double arm_phase
) {

    Eigen::Vector3d base_dir = Eigen::Vector3d(
        (shoulder_T.x() >= 0.0 ? 1.0 : -1.0),
        0.0,
        0.0
    );

    double r = 1.2 * link_length;

    double omega = 2.0 * M_PI / period;

    double h = h_mid_offset + h_amp * std::sin(
        omega * t + arm_phase
    );

    Eigen::Vector3d target_T = shoulder_T;

    target_T += r* base_dir;
    
    target_T.z() = shoulder_T.z() + h;

    return target_T;

}

ThreeLinkIKSolution SimpleKinematicsSolution(
    const Eigen::Vector2d& cup_pos_W, // radius, z, currently pos of link 2, not of cup
    double horizontal_cup_angle,
    double torso_height,
    double L1,
    double L2,
    double L3
) {
    ThreeLinkIKSolution sol;

    double above_horizontal_wrist = horizontal_cup_angle;

    double radius = cup_pos_W.x() - L3 * std::cos(
        above_horizontal_wrist
    );

    double z = torso_height - cup_pos_W.y() + L3 * std::sin(
        above_horizontal_wrist
    );

    double theta = std::atan2(
        radius,
        z
    );

    double Ky = (L1*L1 - L2*L2 - radius*radius - z*z) / (2.0 * L2);

    double above_horizontal_elbow = theta + std::asin(
        Ky / radius // Ky < r for real solution
    );

    std::cout << "Ky = " << Ky << "radius = " << radius << std::endl;


    double below_horizontal_shoulder = std::atan2(
        z + L2 * std::sin(
            above_horizontal_elbow
        ),
        radius - L2 * std::cos(
            above_horizontal_elbow
        )
    );

    sol.shoulder = -(M_PI / 2.0 + below_horizontal_shoulder);

    sol.elbow = above_horizontal_elbow + below_horizontal_shoulder;

    sol.wrist = above_horizontal_wrist - above_horizontal_elbow;

    sol.success = true;

    return sol;
}