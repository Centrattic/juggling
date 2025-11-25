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

// To be honest, since it's only 3 links, I don't even need a solver.
// Could just calculate equations directly for the position, but should also be easy to solve
// Might be much slower though, we'll see
Eigen::VectorXd SolveIKForCup(
    const MultibodyPlant<double> &plant,
    drake::systems::Context<double>* plant_context,
    const RigidBody<double>* cup_body,
    const Eigen::Vector3d& p_Wcup_target
) {

    using drake::math::RigidTransformd;
    using drake::math::RotationMatrixd;
    using drake::multibody::InverseKinematics;
    using drake::solvers::Solve;

    InverseKinematics ik(
        plant, 
        plant_context
    );

    double tol = 0.02;

    ik.AddPositionConstraint(
        // plant.GetFrameByName(
        //     cup_body->name()
        // ),
        cup_body->body_frame(),
        Eigen::Vector3d::Zero(),
        plant.world_frame(),
        p_Wcup_target - Eigen::Vector3d(tol, tol, tol),
        p_Wcup_target + Eigen::Vector3d(tol, tol, tol)
    );

    Eigen::Matrix3d R_WCup = Eigen::Matrix3d::Identity();

    ik.AddOrientationConstraint(
        plant.GetFrameByName(
            cup_body->name()
        ),
        drake::math::RotationMatrixd::Identity(),
        plant.world_frame(),
        drake::math::RotationMatrixd(R_WCup),
        0.05
    );

    auto& prog = ik.prog();
    
    auto result = Solve(
        prog
    );

    if (!result.is_success()) {
        std::cerr << "IK failed!\n";
        return Eigen::VectorXd();
    }

    return result.GetSolution(
        ik.q()
    );

}

ArmIKSolution SolveAnalyticIK(
    double x,
    double y,
    double z,
    double L1,
    double L2,
    double L3
) {
    ArmIKSolution sol;

    double yaw = atan2(
        y, 
        x
    );

    // planar yaw matching
    double r = sqrt(
        x*x + y*y
    );

    double px = r;

    double pz = z;

    double a1 = L1;

    double a2 = L2;

    double a3 = L3;

    double L = a2 + a3;

    // planar distance
    double D = sqrt(
        px*px + pz*pz
    );

    if (D > a1 + L) {
        sol.success = false;
        return sol; // unreachable
    }

    // planar IK
    double c2 = (px*px + pz*pz - a1*a1 - L*L) / (2 * a1 * L);

    if (c2 < -1 || c2 > 1) {
        sol.success = false;
        return sol;
    }

    double s2 = sqrt(
        1 - c2*c2
    );

    double theta2 = atan2(
        s2, 
        c2
    ); // elbow down

    double k1 = a1 + L * c2;

    double k2 = L * s2;

    double theta1 = atan2(
        pz, 
        px
    ) - atan2(
        k2, 
        k1
    );

    // wrist to keep end upright
    double wrist = -(theta1 + theta2);

    sol.success = true;

    sol.yaw = yaw;

    sol.shoulder = theta1;
    
    sol.elbow = theta2;
    
    sol.wrist = wrist;

    return sol;
}

