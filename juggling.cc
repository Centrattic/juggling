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

    auto diagram = builder.Build();

    drake::systems::Simulator<double> simulator(
        *diagram
    );

    auto& plant_context = diagram->GetMutableSubsystemContext(
        mbp,
        &simulator.get_mutable_context()
    );

    /* Simulation, with inverse kinematics for positioning */
    
    Eigen::Vector3d Center1 = Eigen::Vector3d(
        0,
        0.0,
        0.3
    );

    Eigen::Vector3d Center2 = Eigen::Vector3d(
        0,
        0.0,
        0.6
    );

    Eigen::Vector3d g = Eigen::Vector3d(
        0.0,
        0.0,
        -9.81
    );
    
    // ball release consts
    double t_release = 2.0;
    
    double t_catch = 3.0;

    bool ball_released = false;

    Eigen::Vector3d p_release_W;

    Eigen::Vector3d v0_W;

    double radius = 0.10;

    double t_final = 20.0;

    double dt = 0.05;

    double torso_w = 4.0;

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

        Eigen::Vector3d p1 = CupPos1(
            t,
            Center1,
            radius
        );

        Eigen::Vector3d p2 = CupPos2(
            t,
            Center2,
            radius
        );

        TwoLinkIKSolution ik1 = Solve2LinkIK(
            p1,
            torso_w,
            Eigen::Vector3d(
                torso_radius,
                0.0,
                torso_height
            ),
            link_length,
            link_length
        );

        if (ik1.success) {

            arm1.shoulder->set_angle(
                &plant_context, 
                ik1.shoulder
            );

            arm1.elbow->set_angle(
                &plant_context, 
                ik1.elbow
            );

        } else {

            std::cout << "IK 1 failed" << std::endl;

        }

        TwoLinkIKSolution ik2 = Solve2LinkIK(
            p2,
            torso_w,
            Eigen::Vector3d(
                -torso_radius,
                0.0,
                torso_height
            ),
            link_length,
            link_length
        );

        if (ik2.success) {

            arm2.shoulder->set_angle(
                &plant_context,
                ik2.shoulder);

            arm2.elbow->set_angle(
                &plant_context,
                ik2.elbow
            );

        } else {
            std::cout << "IK 2 failed" << std::endl;
        }

        // plan and execute a single throw
        if (!ball_released && t >= t_release) {

            auto X_WC = mbp.EvalBodyPoseInWorld(
                plant_context,
                *arm1.cup_body
            );

            p_release_W = X_WC.translation();

            Eigen::Vector3d p_catch_W = CupPos2( // following circular traj
                t_catch,
                Center2,
                radius
            );

            v0_W = ComputeThrowVelocity(
                p_release_W,
                p_catch_W,
                t_release,
                t_catch,
                g
            );

            ball_released = true;
        }

        if (ball_released) {
            
            Eigen::Vector3d p_ball = ObjectPosition(
                p_release_W,
                v0_W,
                t,
                t_release,
                g
            );

            mbp.SetFreeBodyPose(
                &plant_context,
                ball,
                RigidTransformd(
                    p_ball
                )
            );
        } else { // prior to release

            auto X_WC = mbp.EvalBodyPoseInWorld(
                plant_context,
                *arm1.cup_body
            );

            mbp.SetFreeBodyPose(
                &plant_context,
                ball,
                X_WC
            );
        }

        simulator.AdvanceTo(t);

        std::this_thread::sleep_for(std::chrono::milliseconds(50));

    }

    std::cout << "Simulation done. Press Enter to exit..." << std::endl;

    return 0;
}
