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
    const double cup_radius = 0.37; // radius from the center of the cup

    const double cup_z = torso_height + 0.10; // height from the ground

    const double ball_cup_offset_z = 0.10;

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

    /* Ball settings: connected to world with planar joint */
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

    // optimize collision so it's possibly a bit faster?
    // mbp.set_contact_model(
    //     drake::multibody::ContactModel::kPoint
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
    
    double t_final = 20.0;

    double dt = 0.01;

    double torso_w = 8.0;

    ThreeLinkIKSolution rest1 = SimpleKinematicsSolution(
        Eigen::Vector2d(
            cup_radius, // from torso center
            cup_z // from ground
        ),
        M_PI / 2.0,
        torso_height,
        link_length,
        link_length,
        link_length
    );
    
    // ThreeLinkIKSolution rest2 = Solve3LinkIKWithOrientation(
    //     cup2_pos_desired,
    //     cup2_direction_up, // also face upward
    //     0.0,
    //     shoulder2_T,
    //     link_length,
    //     link_length,
    //     link_length
    // );

    ThreeLinkIKSolution rest2 = ThreeLinkIKSolution(
        true,
        -rest1.shoulder,
        -rest1.elbow,
        -rest1.wrist
    );

    if (!rest1.success) {

        std::cerr << "ERROR: IK solution failed for arm1!" << std::endl;

        std::cerr << "  link_length = " << link_length << std::endl;

        return 1;

    }
    
    if (!rest2.success) {

        std::cerr << "ERROR: IK solution failed for arm2!" << std::endl;

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

    Kp.setConstant(40.0);

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

    // calc drop height
    const Eigen::Vector3d g(
        0.0,
        0.0,
        -9.81
    );

    auto [ball_drop_height, catch_time] = CalculateDropHeightAndTime(
        cup_radius,
        cup_z + ball_cup_offset_z,
        torso_w,
        g
    );

    std::cout << "Ball drop height: " << ball_drop_height << std::endl;

    std::cout << "Catch time: " << catch_time << std::endl;
    
    // init ball as free body at drop position
    mbp.SetFreeBodyPose(
        &plant_context,
        ball,
        RigidTransformd(
            Eigen::Vector3d(
                -cup_radius,  // x: aligned with cup
                0.0,         // y: at y=0
                ball_drop_height  // z: calculated drop height
            )
        )
    );
    
    mbp.SetFreeBodySpatialVelocity(
        &plant_context,
        ball,
        SpatialVelocity<double>(
            Eigen::Vector3d::Zero(),
            Eigen::Vector3d::Zero() 
        )
    );
    
    // Ball state tracking
    double ball_drop_time = 0.0;

    bool ball_caught = false;

    int active_arm = 1; // 1 or 2, eventually both
    
    /* Throw state tracking */

    double hold_time = 2.0; // time to hold ball in cup before throw

    double throw_release_time = -1.0; // when to release ball (-1 = not scheduled)

    Eigen::Vector3d throw_velocity = Eigen::Vector3d::Zero();

    double throw_flight_time = 0.0;

    int num_rotations = 1;  // num rotations before catch

    std::cout << "Simulator initialized" << std::endl;
    
    // DEBUG: Check current state and PID inputs/outputs before first step
    auto current_positions = mbp.GetPositions(
        plant_context
    );

    auto current_velocities = mbp.GetVelocities(
        plant_context
    );

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

    // Meschat connection delay
    std::cout << "Waiting for Meshcat to connect..." << std::endl;

    std::this_thread::sleep_for(
        std::chrono::milliseconds(
            5000
        )
    );

    std::cout << "Starting simulation..." << std::endl;

    for (double t = 0; t < t_final; t += dt) {

        RigidTransformd ball_pose = mbp.GetFreeBodyPose(
            plant_context, 
            ball
        );

        SpatialVelocity<double> ball_spatial_vel = mbp.EvalBodySpatialVelocityInWorld(
            plant_context, 
            ball
        );
        
        Eigen::Vector3d ball_pos = ball_pose.translation();

        Eigen::Vector3d ball_velocity = ball_spatial_vel.translational();
        
        RigidTransformd cup1_pose = mbp.EvalBodyPoseInWorld(
            plant_context,
            *arm1.cup_body
        );

        RigidTransformd cup2_pose = mbp.EvalBodyPoseInWorld(
            plant_context,
            *arm2.cup_body
        );
        
        Eigen::Vector3d cup1_pos_W = cup1_pose.translation();

        Eigen::Vector3d cup2_pos_W = cup2_pose.translation();
        
        // key: same arm catches, it rotates to catch pos
        Eigen::Vector3d catch_cup_pos = (
            active_arm == 1
        ) ? cup1_pos_W : cup2_pos_W;
        
        double catch_tolerance = 0.02; // high

        bool ball_near_catch_cup = (
            ball_pos - catch_cup_pos
        ).head<2>().norm() < catch_tolerance;

        
        /** Handle ball catches */
        
        // Check if cup is at y=0 (or close) for catching
        bool cup_at_catch_position = std::abs(
            catch_cup_pos.y()
        ) < 0.1;
        
        // Debug: print catch conditions near catch_time
        if (!ball_caught && t >= catch_time - 0.1 && t <= catch_time + 0.5) {
            std::cout << "t=" << t << " catch_time=" << catch_time 
                      << " cup_y=" << catch_cup_pos.y() 
                      << " ball_z=" << ball_pos.z() 
                      << " catch_z=" << (cup_z + ball_cup_offset_z)
                      << " near=" << ball_near_catch_cup
                      << " cup_at_pos=" << cup_at_catch_position << std::endl;
        }

        if (!ball_caught && 
            t >= catch_time && 
            // cup_at_catch_position && dont want this since not true beyond first catch
            ball_pos.z() <= cup_z + ball_cup_offset_z + 0.15 &&
            ball_pos.z() >= cup_z + ball_cup_offset_z - 0.2
            // ball_near_catch_cup, something wrong with this?
        ) {

            std::cout << "Ball caught at time" << t << std::endl;
            
            ball_caught = true;
            
            throw_release_time = -1.0; // reset throw
            
            // Eigen::Vector3d release_pos = catch_cup_pos;
            
            // release_pos.z() += ball_cup_offset_z;

            double angle_at_catch = std::atan2(
                catch_cup_pos.y(), 
                catch_cup_pos.x()
            );

            double angle_at_release = angle_at_catch + torso_w * hold_time;

            Eigen::Vector3d release_pos(
                cup_radius * std::cos(
                    angle_at_release
                ),
                cup_radius * std::sin(
                    angle_at_release
                ),
                cup_z + ball_cup_offset_z
            );
            
            auto [throw_vel, flight_t] = CalculateThrowVelocityAndTime(
                release_pos,
                angle_at_release,
                cup_z + ball_cup_offset_z, // target catch height, mihgt be different from release height later
                cup_radius,
                torso_w,
                num_rotations,
                g
            );
            
            throw_velocity = throw_vel;

            throw_flight_time = flight_t;

            throw_release_time = t + hold_time; // hold ball in cup for 1 second before throw
            
            std::cout << "Ball caught by arm" << active_arm << " at t=" << t << ". Throw scheduled: release at t=" 
                      << throw_release_time << ", flight_time=" << flight_t << "s" << std::endl;
        }
        
        /* Handle ball throws */

        if (ball_caught &&
            throw_release_time > 0 &&
            t >= throw_release_time && 
            t < throw_release_time + dt
        ) {
            // release ball with throw velocity from active arm
            Eigen::Vector3d release_cup_pos = (
                active_arm == 1
            ) ? cup1_pos_W : cup2_pos_W;

            Eigen::Vector3d release_pos = release_cup_pos;

            release_pos.z() += ball_cup_offset_z;
            
            mbp.SetFreeBodyPose(
                &plant_context,
                ball,
                RigidTransformd(
                    release_pos
                )
            );
            
            mbp.SetFreeBodySpatialVelocity(
                &plant_context,
                ball,
                SpatialVelocity<double>(
                    Eigen::Vector3d::Zero(), // angular vel
                    throw_velocity // translational vel
                )
            );
            
            ball_caught = false;

            catch_time = throw_release_time + throw_flight_time; // next catch time
            
            std::cout << "Ball released at t=" << t << " with velocity (" 
                      << throw_velocity.x() << ", " << throw_velocity.y() << ", " << throw_velocity.z() << ")" << std::endl;
            
            std::cout << "Next catch scheduled at t=" << catch_time << std::endl;
        }
        
        /* Handle ball holds in cup */

        if (ball_caught && 
            (throw_release_time < 0 || t < throw_release_time)
        ) {
            // get curr cup pose and velocity from active arm
            RigidTransformd current_cup_pose = mbp.EvalBodyPoseInWorld(
                plant_context,
                *(active_arm == 1 ? arm1.cup_body : arm2.cup_body)
            );
            
            SpatialVelocity<double> cup_velocity = mbp.EvalBodySpatialVelocityInWorld(
                plant_context,
                *(active_arm == 1 ? arm1.cup_body : arm2.cup_body)
            );
            
            Eigen::Vector3d current_cup_pos = current_cup_pose.translation();
            
            Eigen::Vector3d ball_in_cup_pos = current_cup_pos;

            ball_in_cup_pos.z() = current_cup_pos.z() + ball_cup_offset_z;
            
            mbp.SetFreeBodyPose(
                &plant_context,
                ball,
                RigidTransformd(ball_in_cup_pos)
            );
            
            mbp.SetFreeBodySpatialVelocity(
                &plant_context,
                ball,
                cup_velocity
            );
        }        
    
        /* DEBUG: check PID inputs/outputs before each step */

        auto current_pos = mbp.GetPositions(
            plant_context
        );

        auto current_vel = mbp.GetVelocities(
            plant_context
        );

        Eigen::VectorXd full_state(
            nx
        );

        full_state.head(nq) = current_pos;

        full_state.tail(nv) = current_vel;

        Eigen::VectorXd selected = S * full_state;

        Eigen::VectorXd error = x_des - selected;
        
        if (static_cast<int>(t * 10) % 25 == 0) { // Every 0.5 seconds (25 steps at dt=0.02)
            std::cout << "\n=== Step at t=" << t << " ===" << std::endl;

            std::cout << "Ball pos: (" << ball_pos.x() << ", " << ball_pos.y() << ", " << ball_pos.z() << ")" << std::endl;

            std::cout << "Ball vel: (" << ball_velocity.x() << ", " << ball_velocity.y() << ", " << ball_velocity.z() << ")" << std::endl;
        }
        
        // Check for large torques
        Eigen::VectorXd expected_torque = Kp.asDiagonal() * error.head(m) + 
                                            Kd.asDiagonal() * error.tail(m);

        for (int i = 0; i < expected_torque.size(); ++i) {

            if (std::abs(expected_torque[i]) > 20.0) {
                
                std::cerr << "WARNING: Large torque at index " << i 
                            << " at t=" << t << ": " << expected_torque[i] 
                            << " Nm (may cause numerical issues)" << std::endl;
            }
        }

        simulator.AdvanceTo(
            t+dt
        );

        // std::this_thread::sleep_for(
        //     std::chrono::milliseconds(
        //         150
        //     )
        // );

    }

    std::cout << "Simulation done. Press Enter to exit..." << std::endl;

    return 0;
}
