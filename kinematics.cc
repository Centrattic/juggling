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
        radius * cos(t),
        radius * sin(t),
        0.0 // traj is in x-y plane
    );
        
}

Eigen::Vector3d CupPos2(
    double t,
    Eigen::Vector3d Center2,
    double radius
) {
    
    return Center2 + Eigen::Vector3d(
        radius * cos(t),
        radius * sin(t),
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
