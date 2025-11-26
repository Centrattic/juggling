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

Eigen::Vector3d CupPos1(
    double t,
    Eigen::Vector3d Center1,
    double radius
) {

    return Center1 + Eigen::Vector3d(
        radius * cos(0.02*t),
        radius * sin(0.02*t),
        0.0 // traj is in x-y plane
    );
        
}

Eigen::Vector3d CupPos2(
    double t,
    Eigen::Vector3d Center2,
    double radius
) {
    
    return Center2 + Eigen::Vector3d(
        radius * cos(0.02*t),
        radius * sin(0.02*t),
        0.0
    );

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

    if (D > L1 + L2) {

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

JugglerIKSolution SolveJugglerIK(
    const Eigen::Vector3d& p_W,
    double theta_torso,
    const Eigen::Vector3d& shoulder_T,
    double L1,
    double L2
) {
    JugglerIKSolution sol;

    sol.success = false;

    Eigen::AngleAxisd Rz_neg(
        theta_torso * -1.0,
        Eigen::Vector3d::UnitZ()
    );

    Eigen::Vector3d p_T = Rz_neg * p_W;

    Eigen::Vector3d p_rel = p_T - shoulder_T;

    double px = p_rel.x();

    double pz = p_rel.z();

    double D2 = px*px + pz*pz;

    double D  = std::sqrt(
        D2
    );

    if (D > L1 + L2) {

        return sol; // unreachable
    }

    double c2 = (D2 - L1*L1 - L2*L2) / (2.0 * L1 * L2);

    if (c2 < -1.0 || c2 > 1.0) {

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
        pz, 
        px
    ) - std::atan2(
        k2, 
        k1
    );

    sol.success = true;

    sol.torso_yaw = theta_torso;

    sol.shoulder = shoulder;

    sol.elbow = elbow;

    return sol;
}