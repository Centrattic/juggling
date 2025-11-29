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
#include <drake/systems/framework/input_port.h>
#include <drake/systems/primitives/matrix_gain.h>

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
using drake::systems::MatrixGain;

//  Note: Run ./build/juggling_demo from the /home/juggling folder

int main() {
    DiagramBuilder<double> builder;

    auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(
        &builder, 
        0
    );

    MultibodyPlant<double>& mbp = plant;

    const double torso_height = 0.5;

    const double torso_radius = 0.05;

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

    const double link_length = 0.25;

    const double link_radius = 0.03;

    const double link_mass = 1.0;

    // cup position in cylindrical coordinates
    const double cup_radius = 0.22;

    const double cup_z = torso_height + 0.20;
    

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
        cup_z
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
        cup_z
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

    /* Defining consts and setting desired angles */
    
    double radius = 0.10;

    double t_final = 20.0;

    double dt = 0.02;

    double torso_w = 30.0;

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
        
    // arm offsets: for arm1, angle = 0, arm2 angle = M_PI
    double arm1_angle = 0.0;
    double arm2_angle = M_PI;
    
    Eigen::Vector3d cup1_pos_desired = Eigen::Vector3d(
        cup_radius * std::cos(arm1_angle),
        cup_radius * std::sin(arm1_angle),
        cup_z
    );
    
    Eigen::Vector3d cup2_pos_desired = Eigen::Vector3d(
        cup_radius * std::cos(arm2_angle),
        cup_radius * std::sin(arm2_angle),
        cup_z
    );
    
    // must be in x-z plane?? idk
    Eigen::Vector3d cup1_direction_up = Eigen::Vector3d(
        0.0,
        0.0,
        1.0
    ).normalized();
    
    Eigen::Vector3d cup2_direction_up = Eigen::Vector3d(
        0.0,
        0.0,
        1.0
    ).normalized();

    std::cout << "arm1: shoulder1_T = (" << shoulder1_T.x() << ", " << shoulder1_T.y() << ", " << shoulder1_T.z() << ")" << std::endl;
    
    std::cout << "arm1: cup1_pos_desired = (" << cup1_pos_desired.x() << ", " << cup1_pos_desired.y() << ", " << cup1_pos_desired.z() << ")" << std::endl;

    ThreeLinkIKSolution rest1 = Solve3LinkIKWithOrientation(
        cup1_pos_desired,
        cup1_direction_up,
        0.0, // torso angle
        shoulder1_T,
        link_length,
        link_length,
        link_length
    );

    std::cout << "arm2: shoulder2_T = (" << shoulder2_T.x() << ", " << shoulder2_T.y() << ", " << shoulder2_T.z() << ")" << std::endl;
    
    std::cout << "arm2: cup2_pos_desired = (" << cup2_pos_desired.x() << ", " << cup2_pos_desired.y() << ", " << cup2_pos_desired.z() << ")" << std::endl;
    
    ThreeLinkIKSolution rest2 = Solve3LinkIKWithOrientation(
        cup2_pos_desired,
        cup2_direction_up, // also face upward
        0.0,
        shoulder2_T,
        link_length,
        link_length,
        link_length
    );

    if (!rest1.success) {

        std::cerr << "ERROR: IK solution failed for arm1!" << std::endl;

        std::cerr << "  cup_pos_desired = (" << cup1_pos_desired.x() << ", " 
                  << cup1_pos_desired.y() << ", " << cup1_pos_desired.z() << ")" << std::endl;

        std::cerr << "  shoulder1_T = (" << shoulder1_T.x() << ", " 
                  << shoulder1_T.y() << ", " << shoulder1_T.z() << ")" << std::endl;

        std::cerr << "  link_length = " << link_length << std::endl;

        return 1;

    }
    
    if (!rest2.success) {

        std::cerr << "ERROR: IK solution failed for arm2!" << std::endl;

        std::cerr << "  cup_pos_desired = (" << cup2_pos_desired.x() << ", " 
                  << cup2_pos_desired.y() << ", " << cup2_pos_desired.z() << ")" << std::endl;

        std::cerr << "  shoulder2_T = (" << shoulder2_T.x() << ", " 
                  << shoulder2_T.y() << ", " << shoulder2_T.z() << ")" << std::endl;

        std::cerr << "  link_length = " << link_length << std::endl;

        return 1;

    }

    double shoulder1_rest = rest1.shoulder;

    double elbow1_rest = rest1.elbow;

    double wrist1_rest = rest1.wrist;

    double shoulder2_rest = rest2.shoulder;

    double elbow2_rest = rest2.elbow;

    double wrist2_rest = rest2.wrist;

    // DEBUG: IK solutions 
    std::cout << "IK solution - arm1: shoulder=" << shoulder1_rest 
              << ", elbow=" << elbow1_rest << ", wrist=" << wrist1_rest << std::endl;
              
    std::cout << "IK solution - arm2: shoulder=" << shoulder2_rest 
              << ", elbow=" << elbow2_rest << ", wrist=" << wrist2_rest << std::endl;
    
    std::vector<JointTarget> targets = {
        {
            "arm1_shoulder", 
            shoulder1_rest
        },
        {
            "arm1_elbow",
            elbow1_rest
        },
        {
            "arm1_wrist",
            wrist1_rest
        },
        {
            "arm2_shoulder", 
            shoulder2_rest
        },
        {
            "arm2_elbow", 
            elbow2_rest
        },

        {
            "arm2_wrist",
            wrist2_rest
        }
    };

    /* PID control */

    const int m  = static_cast<int>(targets.size());

    const int nx = mbp.num_multibody_states();

    const int nq = mbp.num_positions();

    const int nv = mbp.num_velocities();

    Eigen::MatrixXd S(2 * m, nx);

    S.setZero();

    for (int j = 0; j < m; ++j) {

        const auto& jt = targets[j];

        const auto& joint = mbp.GetJointByName<RevoluteJoint>(
            jt.name
        );

        const int q_index = joint.position_start();  

        const int v_index = nq + joint.velocity_start();
        
        // DEBUG
        if (q_index < 0 || q_index >= nq) {
            
            std::cerr << "ERROR: Invalid position index " << q_index 
                      << " for joint " << jt.name << " (nq=" << nq << ")" << std::endl;
            
                      return 1;
        }
        if (v_index < nq || v_index >= nx) {

            std::cerr << "ERROR: Invalid velocity index " << v_index 
                      << " for joint " << jt.name << " (nv=" << nv << ", nx=" << nx << ")" << std::endl;
           
            return 1;
        }

        S(j, q_index) = 1.0; // to select

        S(m + j, v_index) = 1.0;
        
        std::cout << "Selector: joint " << jt.name << " -> S[" << j << "," << q_index 
                  << "] and S[" << (m+j) << "," << v_index << "]" << std::endl;

    }

    auto* selector = builder.AddSystem<MatrixGain<double>>(
        S
    );

    Eigen::VectorXd Kp(m), Ki(m), Kd(m);

    Kp.setConstant(30.0);

    Ki.setConstant(0.0);

    Kd.setConstant(2.0);

    auto* pid = builder.AddSystem<PidController<double>>(
        Kp,
        Ki,
        Kd
    );

    builder.Connect(
        mbp.get_state_output_port(),
        selector->get_input_port()
    );

    builder.Connect(
        selector->get_output_port(),
        pid->get_input_port_estimated_state()
    );

     builder.Connect(
        pid->get_output_port_control(),
        mbp.get_actuation_input_port()
    );

    const drake::systems::InputPortIndex desired_state_port_index = builder.ExportInput(
        pid->get_input_port_desired_state(),
        "desired_state"
    );

    auto diagram = builder.Build();
    
    std::cout << "Diagram built successfully" << std::endl;

    drake::systems::Simulator<double> simulator(
        *diagram
    );
    
    std::cout << "Simulator created successfully" << std::endl;

    auto& plant_context = diagram->GetMutableSubsystemContext(
        mbp,
        &simulator.get_mutable_context()
    );
    
    std::cout << "Got contexts, about to set joint angles BEFORE Initialize()" << std::endl;

    for (int j = 0; j < m; ++j) {

        const auto& jt = targets[j];

        try {
            
            const auto& joint = mbp.GetJointByName<RevoluteJoint>(
                jt.name
            );

            std::cout << "Setting joint " << jt.name << " to angle " << jt.angle << std::endl;

            joint.set_angle( // initial value
                &plant_context,
                jt.angle
            );

            joint.set_angular_rate(
                &plant_context,
                0.0
            );

        } catch (const std::exception& e) {

            std::cerr << "ERROR: Failed to set joint " << jt.name << ": " << e.what() << std::endl;

            return 1;

        }
    }
    
    auto& diagram_context = simulator.get_mutable_context();
    
    Eigen::VectorXd x_des(2 * m);

    for (int j = 0; j < m; ++j) {

        x_des[j] = targets[j].angle;

        x_des[m + j] = 0.0; // desired velocity is always 0
    }
        
    // fix value is expensive (rebuilds graph)
    diagram->get_input_port(desired_state_port_index).FixValue(
        &diagram_context,
        x_des
    );
    
    simulator.Initialize();

    std::cout << "Simulator initialized" << std::endl;
    
    // DEBUG: Check current state and PID inputs/outputs before first step
    auto current_positions = mbp.GetPositions(plant_context);

    auto current_velocities = mbp.GetVelocities(plant_context);

    std::cout << "Current positions: " << current_positions.transpose() << std::endl;

    std::cout << "Current velocities: " << current_velocities.transpose() << std::endl;
    
    // construct full state vector [q; v]
    Eigen::VectorXd full_state_value(nx);

    full_state_value.head(nq) = current_positions;

    full_state_value.tail(nv) = current_velocities;

    std::cout << "Full state size: " << full_state_value.size() << " (nq=" << nq << ", nv=" << nv << ")" << std::endl;
    
    std::cout << "Full state from plant: " << full_state_value.transpose() << std::endl;
    

    simulator.set_target_realtime_rate(
        1.0
    );

    /* throw details */
    // double throw_start = 1.0;

    // double throw_duration = 0.4;

    // double throw_amp = 0.8;

    // bool ball_released = false;

    // double release_time = throw_start + 0.9 * throw_duration;

    base_yaw.set_angular_rate(
        &plant_context,
        torso_w  // constant rotation rate
    );

    for (double t = 0; t < t_final; t += dt) {
    
        /* DEBUG: check PID inputs/outputs before each step */

        auto current_pos = mbp.GetPositions(plant_context);

        auto current_vel = mbp.GetVelocities(plant_context);

        Eigen::VectorXd full_state(nx);

        full_state.head(nq) = current_pos;

        full_state.tail(nv) = current_vel;

        Eigen::VectorXd selected = S * full_state;

        Eigen::VectorXd error = x_des - selected;
        
        std::cout << "\n=== Step at t=" << t << " ===" << std::endl;

        std::cout << "Selected state: " << selected.transpose() << std::endl;

        std::cout << "Error: " << error.transpose() << std::endl;

        Eigen::VectorXd expected_torque = Kp.asDiagonal() * error.head(m) + 
                                            Kd.asDiagonal() * error.tail(m);

        std::cout << "Expected torque (Kp*pos_err + Kd*vel_err): " << expected_torque.transpose() << std::endl;
        
        for (int i = 0; i < expected_torque.size(); ++i) {

            // Warn if torque is very large (might cause numerical issues)
            if (std::abs(expected_torque[i]) > 20.0) {

                std::cerr << "WARNING: Large torque at index " << i 
                            << " at t=" << t << ": " << expected_torque[i] 
                            << " Nm (may cause numerical issues)" << std::endl;

            }
        }

        simulator.AdvanceTo(t+dt);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    }

    std::cout << "Simulation done. Press Enter to exit..." << std::endl;

    return 0;
}
