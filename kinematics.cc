#include "main.h"
#include "consts.h"
#include <Eigen/Core>
#include <cmath>
#include <iostream>

#include <drake/math/rigid_transform.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/inverse_kinematics/inverse_kinematics.h>
#include <drake/multibody/tree/rigid_body.h>
#include <drake/multibody/math/spatial_velocity.h>
#include <drake/systems/framework/context.h>
#include <drake/solvers/solve.h>
#include <drake/math/rigid_transform.h>

using drake::multibody::MultibodyPlant;
using drake::multibody::RigidBody;
using drake::multibody::SpatialVelocity;
using drake::systems::Context;
using drake::math::RigidTransformd;

// oops mass doesn't rlly matter at all...

/* Object kinematics */

/* Calculate drop height so ball reaches cup_z when cup is at y=0.
cup_y = 0 is when the cup (at radius cup_radius from center) aligns with y-axis
Given torso angular velocity torso_w, we need to find when cup reaches y=0.
Returns: {drop_height, drop_time} */

std::pair<double, double> CalculateDropHeightAndTime(
    double cup_radius,
    double cup_z,
    double torso_w,
    const Eigen::Vector3d& g
) {
    // catch next time reaches y = 0    
    double t_catch = 2.0 * M_PI / torso_w;
    
    // simple kinematics eq    
    double drop_height = cup_z - 0.5 * g.z() * t_catch * t_catch;
    
    return {
        drop_height,
        t_catch
    };
}

/* Calculate throw velocity and flight time for ball to land back in same arm
After n rotations, the cup is back at the same position (y=0)
throw_velocity is the initial velocity needed (in world frame)
Returns: {throw_velocity, flight_time} */

std::pair<Eigen::Vector3d, double> CalculateThrowVelocityAndTime(
    const Eigen::Vector3d& release_pos, // ball release pos
    const Eigen::Vector3d& catch_pos, // predicted catch position (ball center)
    double torso_w,
    int num_rotations, // num rots before catch
    int num_arms,
    const Eigen::Vector3d& g
) {
    // flight time: cup needs to rotate num_rotations * 2pi, + (2pi/num_arms) for angular displacement to next cup
    double angular_displacement = 2.0 * M_PI / num_arms;

    double flight_time = (
        2.0 * M_PI * num_rotations + angular_displacement
    ) / torso_w;
    
    // Calculate throw velocity to hit catch_pos
    // Using: catch_pos = release_pos + v0 * t + 0.5 * g * t^2
    // Solving: v0 = (catch_pos - release_pos - 0.5 * g * t^2) / t
    Eigen::Vector3d displacement = catch_pos - release_pos;
    
    Eigen::Vector3d throw_velocity = (displacement - 0.5 * g * flight_time * flight_time) / flight_time;
    
    return {
        throw_velocity,
        flight_time
    };
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