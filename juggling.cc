#include "main.h"

#include <Eigen/Core>

#include <iostream>
#include <thread>
#include <chrono>

#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/shape_specification.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/plant/coulomb_friction.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/spatial_inertia.h>
#include <drake/multibody/math/spatial_velocity.h>
#include <drake/multibody/tree/unit_inertia.h>
#include <drake/multibody/tree/weld_joint.h>
#include <drake/systems/analysis/simulator.h>
#include <drake/systems/framework/diagram_builder.h>
#include <drake/geometry/shape_specification.h>
#include <drake/systems/controllers/pid_controller.h>

using drake::geometry::Meshcat;
using drake::geometry::MeshcatVisualizer;
using drake::math::RigidTransformd;
using drake::multibody::CoulombFriction;
using drake::multibody::MultibodyPlant;
using drake::multibody::RevoluteJoint;
using drake::multibody::SpatialInertia;
using drake::multibody::SpatialVelocity;
using drake::multibody::UnitInertia;
using drake::multibody::WeldJoint;
using drake::systems::DiagramBuilder;
using drake::systems::controllers::PidController;

//  Note: Run ./build/juggling_demo from the /home/juggling folder

int main() {
    DiagramBuilder<double> builder;

    auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(
        &builder, 
        0.0
    );

    MultibodyPlant<double>& mbp = plant;

    const double torso_height = 0.4;

    const double torso_radius = 0.03;

    const double torso_mass = 5.0;

    SpatialInertia<double> torso_inertia = SpatialInertia<double>::MakeFromCentralInertia(
        torso_mass,
        Eigen::Vector3d::Zero(),
        UnitInertia<double>::SolidCylinder(
            torso_radius,
            torso_height,
            Eigen::Vector3d::UnitZ()
        )
    );

    auto &torso = mbp.AddRigidBody(
        "torso",
        torso_inertia
    );

    mbp.RegisterVisualGeometry(
        torso,
        RigidTransformd(
            Eigen::Vector3d(
                0,
                0,
                torso_height / 2.0
            )
        ),
        drake::geometry::Cylinder(
            torso_radius, 
            torso_height
        ),
        "torso_visual",
        Eigen::Vector4d(
            0.6,
            0.6,
            0.9,
            1.0
        )
    );

    auto& base_yaw = mbp.AddJoint<RevoluteJoint>(
        "base_yaw",
        mbp.world_body(),
        RigidTransformd::Identity(),
        torso,
        std::nullopt,
        Eigen::Vector3d::UnitZ()
    );

    const double link_length = 0.5;

    const double link_radius = 0.03;

    const double link_mass = 1.0;

    const double cup_radius = 0.18;

    const double cup_height = 0.18;

    ArmWithCup arm1 = AddTripleLinkArmWithCup(
        &mbp,
        "arm1_",
        torso,
        RigidTransformd( // X_WShoulder (ground weld pos)
            Eigen::Vector3d(
                torso_radius,
                0.0,
                torso_height
            )
        ),
        link_length,
        link_radius,
        link_mass,
        cup_radius,
        cup_height
    );

    ArmWithCup arm2 = AddTripleLinkArmWithCup(
        &mbp,
        "arm2_",
        torso,
        RigidTransformd(
            Eigen::Vector3d(
                -torso_radius,
                0.0,
                torso_height
            )
        ),
        link_length,
        link_radius,
        link_mass,
        cup_radius,
        cup_height
    );

    const double ball_radius = 0.04;

    const double ball_mass = 0.1;

    SpatialInertia<double> ball_inertia = SpatialInertia<double>::MakeFromCentralInertia(
        ball_mass,
        Eigen::Vector3d::Zero(),
        UnitInertia<double>::SolidSphere(
            ball_radius
        )
    );

    auto& ball = mbp.AddRigidBody(
        "ball",
        ball_inertia
    );

    mbp.RegisterCollisionGeometry(
        ball,
        RigidTransformd::Identity(),
        drake::geometry::Sphere(
            ball_radius
        ),
        "ball_collision",
        CoulombFriction<double>(
            0.9,
            0.5
        )
    );

    mbp.RegisterVisualGeometry(
        ball,
        RigidTransformd::Identity(),
        drake::geometry::Sphere(
            ball_radius
        ),
        "ball_visual",
        Eigen::Vector4d(
            1.0,
            0.0,
            0.0,
            1.0
        )
    );

    // mbp.mutable_gravity_field().set_gravity_vector(
    //     Eigen::Vector3d::Zero()
    // );

    mbp.Finalize();

    /* Visualizing in meschat */

    auto meshcat = std::make_shared<Meshcat>();

    MeshcatVisualizer<double>::AddToBuilder(
        &builder,
        scene_graph,
        meshcat
    );

    /* PID control */

    const int nu = plant.num_actuators();

    Eigen::VectorXd Kp(nu), Ki(nu), Kd(nu);

    Kp.setConstant(50.0);

    Ki.setConstant(0.0);

    Kd.setConstant(5.0);

    auto* pid = builder.AddSystem<PidController<double>>(
        Kp,
        Ki,
        Kd
    );

    builder.Connect(
        mbp.get_state_output_port(),
        pid->get_input_port_estimated_state()
    );

    builder.Connect(
        pid->get_output_port_control(),
        mbp.get_actuation_input_port()
    );

    builder.ExportInput(
        pid->get_input_port_desired_state(),
        "desired_state"
    );

    auto diagram = builder.Build();

    drake::systems::Simulator<double> simulator(
        *diagram
    );

    auto& plant_context = diagram->GetMutableSubsystemContext(
        mbp,
        &simulator.get_mutable_context()
    );

    const int nq = mbp.num_positions();

    const int nv = mbp.num_velocities();

    Eigen::VectorXd q_des = Eigen::VectorXd::Zero(
        nq
    );

    Eigen::VectorXd v_des = Eigen::VectorXd::Zero(
        nv
    );

    /* Simulation, with inverse kinematics for positioning */
    
    // Eigen::Vector3d Center1 = Eigen::Vector3d(
    //     0,
    //     0.0,
    //     0.3
    // );

    // Eigen::Vector3d Center2 = Eigen::Vector3d(
    //     0,
    //     0.0,
    //     0.6
    // );

    // Eigen::Vector3d g = Eigen::Vector3d(
    //     0.0,
    //     0.0,
    //     -9.81
    // );
    
    // ball release consts
    // double t_release = 2.0;
    
    // double t_catch = 3.0;

    // bool ball_released = false;

    // Eigen::Vector3d p_release_W;

    // Eigen::Vector3d v0_W;

    double radius = 0.10;

    double t_final = 20.0;

    double dt = 0.05;

    double torso_w = 4.0;

    Eigen::Vector3d shoulder1_T(
        torso_radius,
        0.0,
        torso_height
    );

    Eigen::Vector3d shoulder2_T(
        -torso_radius,
        0.0,
        torso_height
    );

    double r_side = 0.5 * (link_length + link_length); 
    
    Eigen::Vector3d cup1_T_desired = shoulder1_T + Eigen::Vector3d(
        r_side, 
        0.0, 
        0.0
    );

    Eigen::Vector3d cup2_T_desired = shoulder2_T + Eigen::Vector3d(
        -r_side,
        0.0, 
        0.0
    );

    TwoLinkIKSolution rest1 = Solve2LinkIK(
        cup1_T_desired,
        0.0,
        shoulder1_T,
        link_length,
        link_length
    );

    TwoLinkIKSolution rest2 = Solve2LinkIK(
        cup2_T_desired,
        0.0,
        shoulder2_T,
        link_length,
        link_length
    );

    double shoulder1_rest = rest1.shoulder;
    double elbow1_rest    = -rest1.elbow;

    double shoulder2_rest = -rest1.shoulder;
    double elbow2_rest    = rest1.elbow;

    simulator.Initialize();

    simulator.set_target_realtime_rate(
        1.0
    );

    for (double t = 0; t < t_final; t += dt) {

        // spin torso
        base_yaw.set_angle(
            &plant_context,
            torso_w*t
        );



        arm1.shoulder->set_angle(
            &plant_context,
            shoulder1_rest
        );

        double theta     = arm1.shoulder->get_angle(plant_context);

        double theta_dot = arm1.shoulder->get_angular_rate(plant_context);  // or similar

        double error_pos = theta_des - theta;

        double error_vel = 0.0 - theta_dot;   // you want zero angular velocity

        double tau = Kp * error_pos + Kd * error_vel + Ki * integral_error;

        arm1.shoulder_actuator->set_torque(&plant_context, tau);


        arm1.elbow->set_angle(
            &plant_context,
            elbow1_rest
        );

        arm2.shoulder->set_angle(
            &plant_context,
            shoulder2_rest
        );

        arm2.elbow->set_angle(
            &plant_context,
            elbow2_rest
        );

        // Eigen::Vector3d cup1_T = CupTorsoTarget(
        //     t,
        //     Eigen::Vector3d(
        //         torso_radius,
        //         0.0,
        //         torso_height
        //     ),
        //     link_length,
        //     h_mid,
        //     h_amp,
        //     period,
        //     0.0 // phase
        // );

        // Eigen::Vector3d cup2_T = CupTorsoTarget(
        //     t,
        //     Eigen::Vector3d(
        //         -torso_radius,
        //         0.0,
        //         torso_height
        //     ),
        //     link_length,
        //     h_mid,
        //     h_amp,
        //     period,
        //     M_PI // phase
        // );

        // Eigen::AngleAxisd Rz = Eigen::AngleAxisd(
        //     torso_w,
        //     Eigen::Vector3d::UnitZ()
        // );

        // Eigen::Vector3d p1_W = Rz * cup1_T;

        // Eigen::Vector3d p2_W = Rz * cup2_T;

        // TwoLinkIKSolution ik1 = Solve2LinkIK(
        //     p1_W,
        //     torso_w,
        //     Eigen::Vector3d(
        //         torso_radius,
        //         0.0,
        //         torso_height
        //     ),
        //     link_length,
        //     link_length
        // );

        // if (ik1.success) {

        //     arm1.shoulder->set_angle(
        //         &plant_context, 
        //         ik1.shoulder
        //     );

        //     arm1.elbow->set_angle(
        //         &plant_context, 
        //         ik1.elbow
        //     );

        // } else {

        //     std::cout << "IK 1 failed" << std::endl;

        // }

        // TwoLinkIKSolution ik2 = Solve2LinkIK(
        //     p2_W,
        //     torso_w,
        //     Eigen::Vector3d(
        //         -torso_radius,
        //         0.0,
        //         torso_height
        //     ),
        //     link_length,
        //     link_length
        // );

        // if (ik2.success) {

        //     arm2.shoulder->set_angle(
        //         &plant_context,
        //         ik2.shoulder);

        //     arm2.elbow->set_angle(
        //         &plant_context,
        //         ik2.elbow
        //     );

        // } else {
        //     std::cout << "IK 2 failed" << std::endl;
        // }

        auto X_WC = mbp.EvalBodyPoseInWorld(
            plant_context,
            *arm1.cup_body
        );

        mbp.SetFreeBodyPose( // keep ball in arm 1
            &plant_context,
            ball,
            X_WC
        );

        // plan and execute a single throw
        // if (!ball_released && t >= t_release) {

        //     auto X_WC = mbp.EvalBodyPoseInWorld(
        //         plant_context,
        //         *arm1.cup_body
        //     );

        //     p_release_W = X_WC.translation();

        //     Eigen::Vector3d p_catch_W = CupPos2( // following circular traj
        //         t_catch,
        //         Center2,
        //         radius
        //     );

        //     v0_W = ComputeThrowVelocity(
        //         p_release_W,
        //         p_catch_W,
        //         t_release,
        //         t_catch,
        //         g
        //     );

        //     ball_released = true;
        // }

        // if (ball_released) {
            
        //     Eigen::Vector3d p_ball = ObjectPosition(
        //         p_release_W,
        //         v0_W,
        //         t,
        //         t_release,
        //         g
        //     );

        //     mbp.SetFreeBodyPose(
        //         &plant_context,
        //         ball,
        //         RigidTransformd(
        //             p_ball
        //         )
        //     );
        // } else { // prior to release

        //     auto X_WC = mbp.EvalBodyPoseInWorld(
        //         plant_context,
        //         *arm1.cup_body
        //     );

        //     mbp.SetFreeBodyPose(
        //         &plant_context,
        //         ball,
        //         X_WC
        //     );
        // }

        simulator.AdvanceTo(t);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    }

    std::cout << "Simulation done. Press Enter to exit..." << std::endl;

    return 0;
}
