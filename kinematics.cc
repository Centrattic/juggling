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
    double px_signed = (
        p_rel.x() >= 0.0 ? px_plane : -px_plane
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
        px_signed
    ) - std::atan2(
        k2,
        k1
    );

    sol.success = true;

    sol.shoulder = shoulder;

    sol.elbow = elbow;

    return sol;
}

TwoLinkIKSolution Solve2LinkIKWithOrientation(
    const Eigen::Vector3d& cup_pos_W,
    const Eigen::Vector3d& cup_direction_W,
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

    Eigen::Vector3d cup_pos_T = Rz_neg * cup_pos_W;

    Eigen::Vector3d cup_dir_T = Rz_neg * cup_direction_W;

    Eigen::Vector3d cup_rel = cup_pos_T - shoulder_T;

    Eigen::Vector3d cup_dir_normalized = cup_dir_T.normalized();

    Eigen::Vector3d elbow_pos_T = cup_pos_T - L2 * cup_dir_normalized;
    
    Eigen::Vector3d elbow_rel = elbow_pos_T - shoulder_T;

    double px_plane = std::sqrt(
        elbow_rel.x() * elbow_rel.x() + elbow_rel.y() * elbow_rel.y()
    );
    double px_signed = (
        elbow_rel.x() >= 0.0 ? px_plane : -px_plane
    );

    double pz_plane = elbow_rel.z();

    double D = std::sqrt(
        px_plane * px_plane + pz_plane * pz_plane
    );

    if (D > L1 || D < 1e-6) {
        sol.success = false;
        return sol;
    }

    double shoulder = std::atan2(
        pz_plane, 
        px_signed
    );

    // elbow angle: direction of link2 relative to link1
    Eigen::Vector3d link1_dir_T(
        std::cos(shoulder),
        0.0,
        std::sin(shoulder)
    );

    double cos_elbow = link1_dir_T.dot(
        cup_dir_normalized
    );

    cos_elbow = std::max(
        -1.0,
        std::min(
            1.0,
            cos_elbow
        )
    );
    
    double elbow = std::acos(
        cos_elbow
    );
    
    // Determine sign based on cross product
    double cross_y = link1_dir_T.x() * cup_dir_normalized.z() - 
                     link1_dir_T.z() * cup_dir_normalized.x();

    if (cross_y < 0) {
        elbow = -elbow;
    }

    sol.success = true;

    sol.shoulder = shoulder;

    sol.elbow = elbow;

    return sol;
}

ThreeLinkIKSolution Solve3LinkIKWithOrientation(
    const Eigen::Vector3d& cup_pos_W,
    const Eigen::Vector3d& cup_direction_W,
    double theta_torso,
    const Eigen::Vector3d& shoulder_T,
    double L1,
    double L2,
    double L3
) {
    ThreeLinkIKSolution sol;
    
    // transform to torso frame
    Eigen::AngleAxisd Rz_neg(
        -theta_torso,
        Eigen::Vector3d::UnitZ()
    );
    
    Eigen::Vector3d cup_pos_T = Rz_neg * cup_pos_W;

    Eigen::Vector3d cup_dir_T = Rz_neg * cup_direction_W;

    Eigen::Vector3d cup_dir_normalized = cup_dir_T.normalized();
    
    Eigen::Vector3d wrist_pos_T = cup_pos_T - L3 * cup_dir_normalized;
    
    // solve 2-link IK directly in torso frame to reach wrist position
    Eigen::Vector3d wrist_rel = wrist_pos_T - shoulder_T;
    
    double px_plane = std::sqrt(
        wrist_rel.x() * wrist_rel.x() + wrist_rel.y() * wrist_rel.y()
    );
    
    double px_signed = (
        wrist_rel.x() >= 0.0 ? px_plane : -px_plane
    );

    double pz_plane = wrist_rel.z();
    
    double D2 = px_plane * px_plane + pz_plane * pz_plane;

    double D = std::sqrt(
        D2
    );
    
    if (D > L1 + L2 || D < std::fabs(L1 - L2)) {

        sol.success = false;

        std::cerr << "IK failed: wrist distance D=" << D 
                  << " not in range [" << std::fabs(L1 - L2) << ", " << (L1 + L2) << "]" << std::endl;
                  std::cerr << "  cup_pos_T = (" << cup_pos_T.x() << ", " << cup_pos_T.y() << ", " << cup_pos_T.z() << ")" << std::endl;
        
        std::cerr << "  wrist_pos_T = (" << wrist_pos_T.x() << ", " << wrist_pos_T.y() << ", " << wrist_pos_T.z() << ")" << std::endl;
        
        std::cerr << "  shoulder_T = (" << shoulder_T.x() << ", " << shoulder_T.y() << ", " << shoulder_T.z() << ")" << std::endl;
        
        return sol;

    }
    
    double c2 = (D2 - L1*L1 - L2*L2) / (2.0 * L1 * L2);
    
    if (c2 < -1.0 || c2 > 1.0) {
        sol.success = false;
        return sol;
    }
    
    double s2 = std::sqrt(
        1.0 - c2*c2
    );
    
    double elbow_angle = std::atan2(
        s2, 
        c2
    );
    
    double k1 = L1 + L2 * c2;

    double k2 = L2 * s2;
    
    double shoulder_angle = std::atan2(
        pz_plane,
        px_signed
    ) - std::atan2(
        k2, 
        k1
    );
    
    sol.shoulder = shoulder_angle;
    sol.elbow = elbow_angle;

    // Link1 direction in torso frame
    Eigen::Vector3d link1_dir_T(
        std::cos(sol.shoulder),
        0.0,
        std::sin(sol.shoulder)
    );
    
    double cos_elbow = std::cos(sol.elbow);

    double sin_elbow = std::sin(sol.elbow);
    
    Eigen::Vector3d link2_dir_T(
        link1_dir_T.x() * cos_elbow - link1_dir_T.z() * sin_elbow,
        0.0,
        link1_dir_T.x() * sin_elbow + link1_dir_T.z() * cos_elbow
    );
    
    // project directions to X-Z plane
    Eigen::Vector2d link2_plane(
        link2_dir_T.x(), 
        link2_dir_T.z()
    );

    Eigen::Vector2d cup_plane(
        cup_dir_normalized.x(), 
        cup_dir_normalized.z()
    );
    
    double link2_angle = std::atan2(
        link2_plane.y(), 
        link2_plane.x()
    );

    double cup_angle = std::atan2(
        cup_plane.y(), 
        cup_plane.x()
    );
    
    sol.wrist = cup_angle - link2_angle;
    
    // normalize to [-pi, pi]
    while (sol.wrist > M_PI) sol.wrist -= 2.0 * M_PI;

    while (sol.wrist < -M_PI) sol.wrist += 2.0 * M_PI;
    
    sol.success = true;
    
    return sol;
}