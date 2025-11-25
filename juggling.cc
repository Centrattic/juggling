#include "main.h"

#include <Eigen/Core>

#include <iostream>
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

    const double link_length = 0.4;

    const double link_radius = 0.03;

    const double link_mass = 1.0;

    const double cup_radius = 0.18;

    const double cup_height = 0.18;

    ArmWithCup arm1 = AddTripleLinkArmWithCup(
        &mbp,
        "arm1_",
        RigidTransformd( // X_WShoulder (ground weld pos)
            Eigen::Vector3d(
                1.0,
                0.0,
                0.0
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
        RigidTransformd(
            Eigen::Vector3d(
                -1.0,
                0.0,
                0.0
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
        1.0,
        0.0,
        0.4
    );

    Eigen::Vector3d Center2 = Eigen::Vector3d(
        -1.0,
        0.0,
        0.4
    );

    double radius = 0.10;

    double t_final = 10.0;
    double dt = 0.05;

    simulator.Initialize();

    for (double t = 0; t < t_final; t += dt) {

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

        Eigen::VectorXd q_sol_1 = SolveIKForCup(
            mbp,
            &plant_context,
            arm1.cup_body,
            p1
        );

        Eigen::VectorXd q_sol_2 = SolveIKForCup(
            mbp,
            &plant_context,
            arm2.cup_body,
            p2
        );

        if (q_sol_1.size() != mbp.num_positions() ||
            q_sol_2.size() != mbp.num_positions()) {
            std::cerr << "Skipping t=" << t << " because IK failed.\n";
            simulator.AdvanceTo(t);
            continue;
        }

        arm1.shoulder->set_angle(
            &plant_context,
            q_sol_1[arm1.shoulder->position_start()]
        );

        arm1.elbow->set_angle(
            &plant_context,
            q_sol_1[arm1.elbow->position_start()]
        );

        arm1.wrist->set_angle(
            &plant_context, 
            q_sol_1[arm1.wrist->position_start()]
        );

        arm2.shoulder->set_angle(
            &plant_context,
            q_sol_2[arm2.shoulder->position_start()]
        );

        arm2.elbow->set_angle(
            &plant_context,
            q_sol_2[arm2.elbow->position_start()]
        );

        arm2.wrist->set_angle(
            &plant_context, 
            q_sol_2[arm2.wrist->position_start()]
        );

        simulator.AdvanceTo(t);

    }

    std::cout << "Simulation done. Press Enter to exit..." << std::endl;

    return 0;
}
