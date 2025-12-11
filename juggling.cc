#include "main.h"
#include "consts.h"

#include <Eigen/Core>

#include <iostream>
#include <thread>
#include <chrono>
#include <vector>
#include <cmath>

#include <drake/geometry/meshcat.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/scene_graph.h>
#include <drake/geometry/shape_specification.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/multibody/plant/coulomb_friction.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/spatial_inertia.h>
#include <drake/multibody/math/spatial_velocity.h>
#include <drake/multibody/tree/unit_inertia.h>
#include <drake/multibody/tree/weld_joint.h>
#include <drake/multibody/tree/rotational_inertia.h>
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
using drake::multibody::RotationalInertia;
using drake::systems::DiagramBuilder;
using drake::systems::controllers::PidController;
using drake::systems::MatrixGain;
using drake::multibody::RigidBody;
using drake::math::RotationMatrixd;

//  Note: Run ./build/juggling_demo from the /home/juggling folder

const std::string JUGGLING_OBJECT_SDF = "hat_01.sdf";

int main() {

    std::string object_model_name = JUGGLING_OBJECT_SDF;

    if (object_model_name.size() > 4 && 
        object_model_name.substr(
            object_model_name.size() - 4
        ) == ".sdf") {

        object_model_name = object_model_name.substr(
            0, 
            object_model_name.size() - 4
        );
    }
    
    std::string object_sdf_path = "objects/" + JUGGLING_OBJECT_SDF;

    DiagramBuilder<double> builder;

    auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(
        &builder, 
        0
    );

    MultibodyPlant<double>& mbp = plant;

    SpatialInertia<double> torso_inertia = SpatialInertia<double>::MakeFromCentralInertia(
        consts::torso_mass,
        Eigen::Vector3d::Zero(),
        UnitInertia<double>::SolidCylinder(
            consts::torso_radius,
            consts::torso_height,
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
                consts::torso_height / 2.0
            )
        ),
        drake::geometry::Cylinder(
            consts::torso_radius, 
            consts::torso_height
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

    
    /* creating arms, equally spaced around the torso */

    std::vector<ArmWithCup> arms;

    arms.reserve(
        consts::num_arms
    );
    
    for (int i = 0; i < consts::num_arms; ++i) {
        
        // arm angle around torso
        double angle = 2.0 * M_PI * i / consts::num_arms;
        
        Eigen::Vector3d shoulder_pos(
            -consts::torso_radius * std::cos(
                angle
            ),
            consts::torso_radius * std::sin(
                angle
            ),
            consts::torso_height
        );

        std::cout << "Shoulder position: " << shoulder_pos.transpose() << std::endl;

        RotationMatrixd joint_frame_rotation = RotationMatrixd::MakeZRotation(
            -angle
        );

        RigidTransformd X_PShoulder(
            joint_frame_rotation,
            shoulder_pos
        );
                
        std::string arm_name = "arm" + std::to_string(
            i + 1
        ) + "_";
        
        arms.push_back(
            AddTripleLinkArmWithCup(
                &mbp,
                arm_name,
                torso,
                X_PShoulder,
                RigidTransformd::Identity(),
                consts::link_length,
                consts::link_radius,
                consts::link_mass,
                consts::cup_radius,
                consts::cup_z
            )
        );
    }

    // vector of ball objects
    std::vector<const drake::multibody::RigidBody<double>*> ball_bodies;

    ball_bodies.reserve(
        consts::num_arms
    );
    
    for (int i = 0; i < consts::num_arms; ++i) {


        std::string object_name = object_model_name + "_" + std::to_string(
            i + 1
        );

        auto* ball_body = BuildObj(
            &plant,
            object_name,
            object_sdf_path
        );
        

        ball_bodies.push_back(
            ball_body
        );
    }

    // RigidBody<double>* ball_body = ball.ball;

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

    ThreeLinkIKSolution rest_solution = SimpleKinematicsSolution(
        Eigen::Vector2d(
            consts::cup_radius,
            consts::cup_z
        ),
        M_PI / 2.0,
        consts::torso_height,
        consts::link_length,
        consts::link_length,
        consts::link_length
    );
    
    if (!rest_solution.success) {
        std::cerr << "ERROR: IK solution failed!" << std::endl;

        std::cerr << "  link_length = " << consts::link_length << std::endl;
        return 1;
    }
    
    // DEBUG: IK solution 
    std::cout << "IK solution: shoulder=" << rest_solution.shoulder 
              << ", elbow=" << rest_solution.elbow << ", wrist=" << rest_solution.wrist << std::endl;
    
    std::vector<JointTarget> targets;

    targets.reserve(
        consts::num_arms * 3
    );
    
    for (int i = 0; i < consts::num_arms; ++i) {

        double arm_angle = 2.0 * M_PI * i / consts::num_arms;
        
        std::string arm_prefix = "arm" + std::to_string(
            i + 1
        ) + "_";

        targets.push_back(
            {
                arm_prefix + "shoulder",
                rest_solution.shoulder
            }
        );
        
        targets.push_back(
            {
                arm_prefix + "elbow",
                rest_solution.elbow
            }
        );
        targets.push_back(
            {
                arm_prefix + "wrist",
                rest_solution.wrist
            }
        );
    }

    /* PID control */

    const int m  = static_cast<int>(
        targets.size()
    );

    const int nx = mbp.num_multibody_states();

    const int nq = mbp.num_positions();

    const int nv = mbp.num_velocities();

    Eigen::MatrixXd S(
        2 * m,
        nx
    );

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

    Kp.setConstant(
        consts::Kp
    );

    Ki.setConstant(
        consts::Ki
    );

    Kd.setConstant(
        consts::Kd
    );

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

    /* set joint angles and ang vel */

    for (int j = 0; j < m; ++j) {

        const auto& jt = targets[j];
            
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

    std::vector<BallState> ball_states;

    ball_states.reserve(
        consts::num_arms
    );
    
    for (int i = 0; i < consts::num_arms; ++i) {
        
        RigidTransformd cup_pose = mbp.EvalBodyPoseInWorld(
            plant_context,
            *arms[
                i
            ].cup_body
        );
        
        SpatialVelocity<double> cup_velocity = mbp.EvalBodySpatialVelocityInWorld(
            plant_context,
            *arms[
                i
            ].cup_body
        );
        
        Eigen::Vector3d cup_pos = cup_pose.translation();

        Eigen::Vector3d ball_in_cup_pos = cup_pos;

        ball_in_cup_pos.z() += consts::ball_cup_offset_z;
        
        // set ball position and velocity to match cup
        mbp.SetFreeBodyPose(
            &plant_context,
            *ball_bodies[
                i
            ],
            RigidTransformd(
                ball_in_cup_pos
            )
        );
        
        mbp.SetFreeBodySpatialVelocity(
            &plant_context,
            *ball_bodies[
                i
            ],
            cup_velocity
        );
        
        /* initialize ball state */

        BallState ball_state;

        ball_state.ball_body = ball_bodies[i];

        ball_state.arm_index = i;  // change to being 0-indexed

        ball_state.ball_caught = true;  // ball starts in cup

        ball_state.catch_time = 1e6;  // initialized to large value, will be set after first throw

        ball_state.firstThrow = false;  // first throw not happened

        ball_state.throw_release_time = consts::first_hold_time[i];

        ball_state.throw_velocity = Eigen::Vector3d::Zero();

        ball_state.throw_flight_time = 0.0;
        
        ball_states.push_back(
            ball_state
        );
        
        std::cout << "Ball " << (i + 1) << " initialized in arm " << (i + 1) 
                  << ", first throw scheduled at t=" << ball_state.throw_release_time << std::endl;
    }

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
    Eigen::VectorXd full_state_value(
        nx
    );

    full_state_value.head(nq) = current_positions;

    full_state_value.tail(nv) = current_velocities;

    std::cout << "Full state size: " << full_state_value.size() << " (nq=" << nq << ", nv=" << nv << ")" << std::endl;
    
    std::cout << "Full state from plant: " << full_state_value.transpose() << std::endl;

    simulator.set_target_realtime_rate(
        1.0
    );
        base_yaw.set_angular_rate(
        &plant_context,
        consts::torso_w  // constant rotation rate
    );

    // Meschat connection delay
    std::cout << "Waiting for Meshcat to connect..." << std::endl;

    std::this_thread::sleep_for(
        std::chrono::milliseconds(
            5000
        )
    );

    std::cout << "Starting simulation..." << std::endl;

    const Eigen::Vector3d g(
        0.0,
        0.0,
        -9.81
    );

    // error tracking variables
    double total_catch_error = 0.0;

    int total_catch_count = 0;
    
    /* SUS */
    // track actual cup position at catch to learn the effective rest position??
    double measured_rest_cup_radius = consts::cup_radius; // start with expected, update as we learn

    double measured_rest_cup_z = consts::cup_z; // start with expected, update as we learn

    int rest_position_measurements = 0;

    /* main sim loop */

    for (double t = 0; t < consts::t_final; t += consts::dt) {

        std::vector<Eigen::Vector3d> cup_positions;

        cup_positions.reserve(
            consts::num_arms
        );
        
        for (int i = 0; i < consts::num_arms; ++i) {

            RigidTransformd cup_pose = mbp.EvalBodyPoseInWorld(
                plant_context,
                *arms[i].cup_body
            );

            cup_positions.push_back(
                cup_pose.translation()
            );
        }
        
        // loop over each ball
        for (int ball_idx = 0; ball_idx < consts::num_arms; ++ball_idx) {

            BallState& ball_state = ball_states[
                ball_idx
            ];

            int arm_idx = ball_state.arm_index;
            
            RigidTransformd ball_pose = mbp.GetFreeBodyPose(
                plant_context, 
                *ball_state.ball_body
            );
            
            SpatialVelocity<double> ball_spatial_vel = mbp.EvalBodySpatialVelocityInWorld(
                plant_context, 
                *ball_state.ball_body
            );
            
            Eigen::Vector3d ball_pos = ball_pose.translation();

            Eigen::Vector3d ball_velocity = ball_spatial_vel.translational();
            
            // key: same arm catches its own ball
            Eigen::Vector3d catch_cup_pos = cup_positions[
                arm_idx
            ];
            
            /** Handle ball catches */
            
            if (!ball_state.ball_caught && 
                t >= ball_state.catch_time && 
                ball_pos.z() <= consts::cup_z + consts::ball_cup_offset_z + 0.15 &&
                ball_pos.z() >= consts::cup_z + consts::ball_cup_offset_z - 0.2
            ) {
                std::cout << "Ball " << (ball_idx + 1) << " caught at time " << t << std::endl;
                
                /* Calculate avg catch error: distance from ball position to center of cup at catch moment */
                Eigen::Vector3d ball_in_cup_pos = catch_cup_pos;

                ball_in_cup_pos.z() += consts::ball_cup_offset_z;
                
                double catch_error = (
                    ball_pos - ball_in_cup_pos
                ).norm();

                total_catch_error += catch_error;

                total_catch_count++;
                
                std::cout << "  Catch error (used for average): " << catch_error << " m" << std::endl;
                
                /* SUS */
                double actual_cup_radius = std::sqrt(
                    catch_cup_pos.x() * catch_cup_pos.x() + 
                    catch_cup_pos.y() * catch_cup_pos.y()
                );

                double actual_cup_z = catch_cup_pos.z();

                if (rest_position_measurements == 0) {
                    measured_rest_cup_radius = actual_cup_radius;
                    measured_rest_cup_z = actual_cup_z;

                } else {

                    double alpha = 0.3; // lr
                    measured_rest_cup_radius = alpha * actual_cup_radius + (1.0 - alpha) * measured_rest_cup_radius;
                    measured_rest_cup_z = alpha * actual_cup_z + (1.0 - alpha) * measured_rest_cup_z;
                }
                
                rest_position_measurements++;
                
                ball_state.ball_caught = true;

                if (!ball_state.firstThrow) {

                    ball_state.throw_release_time = t + consts::first_hold_time[ball_idx];

                    ball_state.firstThrow = true;

                } else {

                    ball_state.throw_release_time = t + consts::hold_time;
                }
                
                std::cout << "Ball " << (ball_idx + 1) << " caught by arm " << (arm_idx + 1) 
                          << " at t=" << t << ". Throw scheduled: release at t=" 
                          << ball_state.throw_release_time << std::endl;
            }
            
            /** Handle ball throws */
            
            if (ball_state.ball_caught &&
                ball_state.throw_release_time > 0 &&
                t >= ball_state.throw_release_time && 
                t < ball_state.throw_release_time + consts::dt
            ) {
                // could cache all vels ig?
                
                Eigen::Vector3d release_cup_pos = cup_positions[
                    arm_idx
                ];

                Eigen::Vector3d release_pos = release_cup_pos;

                release_pos.z() += consts::ball_cup_offset_z;
                
                double angle_at_release = std::atan2(
                    release_cup_pos.y(), 
                    release_cup_pos.x()
                );
                
                /* SUS */
                double effective_rest_radius = measured_rest_cup_radius;
                double effective_rest_z = measured_rest_cup_z;
                if (rest_position_measurements == 0) {

                    double release_radius = std::sqrt(
                        release_cup_pos.x() * release_cup_pos.x() + 
                        release_cup_pos.y() * release_cup_pos.y()
                    );

                    effective_rest_radius = release_radius;

                    effective_rest_z = release_cup_pos.z();
                }
                
                // catch angle: release angle + angular displacement
                double angular_displacement = 2.0 * M_PI / consts::num_arms;

                double catch_angle = angle_at_release + angular_displacement;
                
                // pred catch position
                Eigen::Vector3d predicted_catch_cup_pos(
                    effective_rest_radius * std::cos(
                        catch_angle
                    ),
                    effective_rest_radius * std::sin(
                        catch_angle
                    ),
                    effective_rest_z
                );
                
                // ball catch position (cup position + offset)
                Eigen::Vector3d predicted_catch_pos = predicted_catch_cup_pos;
                
                predicted_catch_pos.z() += consts::ball_cup_offset_z;
                
                auto [throw_vel, flight_t] = CalculateThrowVelocityAndTime(
                    release_pos,
                    predicted_catch_pos,
                    consts::torso_w,
                    consts::num_rotations,
                    consts::num_arms,
                    g
                );
                
                ball_state.throw_velocity = throw_vel;

                /* set arm joint angles */

                // solve IK, can cache once for per arm for performance if needed
                // if dont have to cache, could add more variety to the motion
                double new_x = consts::cup_radius * std::cos( // not sure why this isn't same as release_cup_pos.x()
                    angle_at_release
                ) + throw_vel.x() * consts::throw_motion_delta;

                double new_y = consts::cup_radius * std::sin(
                    angle_at_release
                ) + throw_vel.y() * consts::throw_motion_delta;

                double new_z = consts::cup_z + throw_vel.z() * consts::throw_motion_delta;

                double new_radius = std::sqrt(
                    new_x * new_x + new_y * new_y
                );

                std::cout << "New radius: " << new_radius << ", new z: " << new_z << std::endl;

                ThreeLinkIKSolution throw_solution = SimpleKinematicsSolution(
                    Eigen::Vector2d(
                        new_radius,
                        new_z
                    ),
                    M_PI / 2.0, // PROBABLY SHOULD UPDATE??
                    consts::torso_height,
                    consts::link_length,
                    consts::link_length,
                    consts::link_length
                );

                if (!throw_solution.success) {

                    std::cerr << "ERROR: IK solution failed!" << std::endl;

                    return 1;

                }
                
                // Validate IK solution values
                if (std::isnan(throw_solution.shoulder) || std::isnan(throw_solution.elbow) || 
                    std::isnan(throw_solution.wrist) ||
                    std::isinf(throw_solution.shoulder) || std::isinf(throw_solution.elbow) || 
                    std::isinf(throw_solution.wrist)) {
                    std::cerr << "ERROR: IK solution contains NaN/Inf values!" << std::endl;
                    std::cerr << "  shoulder=" << throw_solution.shoulder 
                              << ", elbow=" << throw_solution.elbow 
                              << ", wrist=" << throw_solution.wrist << std::endl;
                    return 1;
                }

                for (int j = 3*ball_idx; j < 3*(ball_idx+1); ++j) {

                    const auto& jt = targets[j];

                    double joint_throw_angle = 0.0; 

                    try {
                        
                        const auto& joint = mbp.GetJointByName<RevoluteJoint>(
                            jt.name
                        );
            
                        if (jt.name.find("shoulder") != std::string::npos) {
            
                            joint_throw_angle = throw_solution.shoulder;

                        } else if (jt.name.find("elbow") != std::string::npos) {

                            joint_throw_angle = throw_solution.elbow;

                        } else if (jt.name.find("wrist") != std::string::npos) {

                            joint_throw_angle = throw_solution.wrist;

                        } else {

                            std::cerr << "ERROR: Joint name " << jt.name << " doesn't match expected pattern!" << std::endl;
                            
                            return 1;
                        }

                        // check for invalid values
                        if (std::isnan(joint_throw_angle) || std::isinf(joint_throw_angle)) {

                            std::cerr << "ERROR: Invalid joint angle (NaN/Inf) for " << jt.name 
                                      << ": " << joint_throw_angle << std::endl;

                            return 1;
                        }

                        std::cout << "Setting joint " << jt.name << " to angle " << joint_throw_angle << std::endl;

                        joint.set_angle( // if you dont update state, then will look like a flick, which we kind of want
                            &plant_context,
                            joint_throw_angle
                        );
                        
                        // if you want to update desired state for PID controller
                        // x_des[j] = joint_throw_angle;
                        // x_des[m + j] = 0.0; // desired velocity is always 0
                        
                    } catch (const std::exception& e) {
            
                        std::cerr << "ERROR: Failed to set joint " << jt.name << ": " << e.what() << std::endl;
            
                        return 1;
            
                    }
                }
                
                // also updating desired state for PID controller
                // diagram->get_input_port(desired_state_port_index).FixValue(
                //     &diagram_context,
                //     x_des
                // );

                ball_state.throw_flight_time = flight_t;
                
                mbp.SetFreeBodyPose(
                    &plant_context,
                    *ball_state.ball_body,
                    RigidTransformd(
                        release_pos
                    )
                );
                
                mbp.SetFreeBodySpatialVelocity(
                    &plant_context,
                    *ball_state.ball_body,
                    SpatialVelocity<double>(
                        Eigen::Vector3d::Zero(), // angular vel
                        ball_state.throw_velocity // translational vel
                    )
                );
                
                ball_state.ball_caught = false;

                ball_state.catch_time = ball_state.throw_release_time + ball_state.throw_flight_time;
                
                std::cout << "Ball " << (ball_idx + 1) << " released at t=" << t << " with velocity (" 
                          << ball_state.throw_velocity.x() << ", " << ball_state.throw_velocity.y() 
                          << ", " << ball_state.throw_velocity.z() << ")" << std::endl;
                
                std::cout << "Next catch scheduled at t=" << ball_state.catch_time << std::endl;
            }
            
            /** Handle ball holds in cup */
            
            if (ball_state.ball_caught && 
                (ball_state.throw_release_time < 0 || t < ball_state.throw_release_time)
            ) {

                RigidTransformd current_cup_pose = mbp.EvalBodyPoseInWorld(
                    plant_context,
                    *arms[
                        arm_idx
                    ].cup_body
                );
                
                SpatialVelocity<double> cup_velocity = mbp.EvalBodySpatialVelocityInWorld(
                    plant_context,
                    *arms[
                        arm_idx
                    ].cup_body
                );
                
                Eigen::Vector3d current_cup_pos = current_cup_pose.translation();

                Eigen::Vector3d ball_in_cup_pos = current_cup_pos;

                ball_in_cup_pos.z() = current_cup_pos.z() + consts::ball_cup_offset_z;
                
                mbp.SetFreeBodyPose(
                    &plant_context,
                    *ball_state.ball_body,
                    RigidTransformd(
                        ball_in_cup_pos
                    )
                );
                
                mbp.SetFreeBodySpatialVelocity(
                    &plant_context,
                    *ball_state.ball_body,
                    cup_velocity
                );
            }
            
        }        
    
        /* DEBUG: check PID inputs/outputs before each step

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
        */

        simulator.AdvanceTo(
            t+consts::dt
        );

        std::this_thread::sleep_for(
            std::chrono::milliseconds(
                100
            )
        );

    }

    // Calculate and output average catch error
    if (total_catch_count > 0) {
        double average_catch_error = total_catch_error / total_catch_count;
        std::cout << "\n=== Catch Error Statistics ===" << std::endl;
        std::cout << "Total catches: " << total_catch_count << std::endl;
        std::cout << "Average catch error: " << average_catch_error << " m" << std::endl;
        std::cout << "=============================\n" << std::endl;
    } else {
        std::cout << "\nNo catches recorded during simulation." << std::endl;
    }

    std::cout << "Simulation done. Press Enter to exit..." << std::endl;

    return 0;
}
