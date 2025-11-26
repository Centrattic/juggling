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

    double r = 1.5 * link_length;

    double omega = 2.0 * M_PI / period;

    double h = h_mid_offset + h_amp * std::sin(
        omega * t + arm_phase
    );

    Eigen::Vector3d target_T = shoulder_T;

    target_T += r* base_dir;
    
    target_T.z() = shoulder_T.z() + h;

    return target_T;
    
}

TwoLinkIKSolution Solve2LinkIK(
    const Eigen::Vector3d& p_W,
    double theta_torso,
    const Eigen::Vector3d& shoulder_T,
    double L1,
    double L2
) {
    TwoLinkIKSolution sol;

    Eigen::AngleAxisd Rz_neg(
        -theta_torso,
        Eigen::Vector3d::UnitZ()
    );

    Eigen::Vector3d p_T = Rz_neg * p_W;

    Eigen::Vector3d p_rel = p_T - shoulder_T;

    double px_plane = std::sqrt(
        p_rel.x()*p_rel.x() + p_rel.y()*p_rel.y()
    );

    double pz_plane = p_rel.z();

    double D2 = px_plane*px_plane + pz_plane*pz_plane;

    double D = std::sqrt(
        D2
    );

    if (D > L1 + L2 || D < std::fabs(L1 - L2)) {

        sol.success = false;
        return sol; // unreachable

    }

    double c2 = (D2 - L1*L1 - L2*L2) / (2.0 * L1 * L2);

    if (c2 < -1.0 || c2 > 1.0) {

        sol.success = false;
        return sol;

    }

    double s2 = std::sqrt(
        1.0 - c2*c2
    );
    
    double elbow = std::atan2(
        s2, 
        c2
    );

    double k1 = L1 + L2 * c2;

    double k2 = L2 * s2;

    double shoulder = std::atan2(
        pz_plane,
        px_plane
    ) - std::atan2(
        k2,
        k1
    );

    sol.success = true;

    sol.shoulder = shoulder;

    sol.elbow = elbow;

    return sol;
}