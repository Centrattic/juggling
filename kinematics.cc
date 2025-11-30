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

BallThrowState MaintainBallState(
    MultibodyPlant<double>* mbp,
    const RigidBody<double>* ball_body,
    Context<double>* plant_context,
    double ball_drop_height,
    int active_arm
) {
    
    mbp->SetFreeBodyPose(
        plant_context,
        *ball_body, // need to extract reference
        RigidTransformd(
            Eigen::Vector3d(
                -consts::cup_radius,  // x: aligned with cup
                0.0,         // y: at y=0
                ball_drop_height  // z: calculated drop height
            )
        )
    );

    mbp->SetFreeBodySpatialVelocity(
        plant_context,
        *ball_body,
        SpatialVelocity<double>(
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero() 
        )
    );

    return BallThrowState{
        ball_body,
        0.9,
        false
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
    double angle_at_release,
    double target_z, // target catch height
    double cup_radius, // radius of cup from torso center
    double torso_w,
    int num_rotations, // num rots before catch
    const Eigen::Vector3d& g
) {
    // flight time: cup needs to rotate num_rotations * 2pi, +pi for horizontal displacement
    double flight_time = (2.0 * M_PI * num_rotations + M_PI) / torso_w;
    
    // Ball needs to go up and come back down to target_z in flight_time
    // Using: target_z = release_pos.z + v0.z * t + 0.5 * g.z * t^2
    // Solving for v0.z: v0.z = (target_z - release_pos.z - 0.5 * g.z * t^2) / t
    // Since target_z = release_pos.z (same height), this simplifies to:
    double v0_z = -0.5 * g.z() * flight_time;
    
    // Horizontal velocity: ball needs to travel from +cup_radius to -cup_radius
    // Horizontal distance = 2 * cup_radius (from +x to -x)
    // v0.x = (target_x - release_x) / t = (-cup_radius - cup_radius) / t = -2 * cup_radius / t
    // release_pos.x() should be ±cup_radius when cup is at y=0
    // Direction: if throwing from +cup_radius, go to -cup_radius (negative velocity)

    // Calculate catch position (π radians from release)
    double catch_angle = angle_at_release + M_PI;

    Eigen::Vector3d catch_pos(
        cup_radius * std::cos(
            catch_angle
        ),
        cup_radius * std::sin(
            catch_angle
        ),
        target_z
    );

    // Calculate horizontal displacement
    Eigen::Vector2d horizontal_displacement = (
        catch_pos - release_pos
    ).head<2>();

    Eigen::Vector2d v0_xy = horizontal_displacement / flight_time;

    Eigen::Vector3d throw_velocity(
        v0_xy.x(),
        v0_xy.y(), // there is y-component since released at non y=0 position
        v0_z
    );
    
    return {
        throw_velocity,
        flight_time
    };
}