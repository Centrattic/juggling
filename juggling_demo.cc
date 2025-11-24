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

int main() {
    DiagramBuilder<double> builder;

    auto [plant, scene_graph] = drake::multibody::AddMultibodyPlantSceneGraph(
        &builder, 
        0.0
    );

    MultibodyPlant<double>& mbp = plant;

    const double link_length = 0.5;

    const double link_radius = 0.03;

    const double link_mass = 1.0;

    SpatialInertia<double> link_inertia = SpatialInertia<double>::MakeFromCentralInertia(
            link_mass,
            Eigen::Vector3d::Zero(),
            UnitInertia<double>::SolidCylinder(
                link_radius,
                link_length,
                Eigen::Vector3d::UnitZ()
            )
        );
    
    // Arm 1, consisting of two links

    auto& link1 = mbp.AddRigidBody("link1", link_inertia);

    auto& joint1 = mbp.AddJoint<RevoluteJoint>(
        "shoulder",
        mbp.world_body(), std::nullopt,
        link1, std::nullopt,
        Eigen::Vector3d::UnitY()
    );

    mbp.RegisterVisualGeometry(
        link1,
        RigidTransformd(
            Eigen::Vector3d(
                0,
                0,
                link_length / 2.0
            )
        ),
        drake::geometry::Cylinder(
            link_radius, 
            link_length
        ),
        "link1_visual",
        Eigen::Vector4d(
            0.1,
            0.1,
            1.0,
            1.0
        )
    ); // blue

    auto& link2 = mbp.AddRigidBody(
        "link2",
        link_inertia
    );

    auto& joint2 = mbp.AddJoint<RevoluteJoint>(
        "elbow",
        link1, RigidTransformd(
            Eigen::Vector3d(
                0,
                0,
                link_length
            )
        ),
        link2,
        std::nullopt,
        Eigen::Vector3d::UnitY()
    );
    
    mbp.RegisterVisualGeometry(
        link2,
        RigidTransformd(
            Eigen::Vector3d(
                0,
                0,
                link_length / 2.0
            )
        ),
        drake::geometry::Cylinder(
            link_radius,
            link_length
        ),
        "link2_visual",
        Eigen::Vector4d(
            0.1,
            1.0,
            0.1,
            1.0)
    ); // green

    // Cup

    const double cup_radius = 0.08;

    const double cup_height = 0.04;

    SpatialInertia<double> cup_inertia = SpatialInertia<double>::MakeFromCentralInertia(
        0.2,
        Eigen::Vector3d::Zero(),
        UnitInertia<double>::SolidCylinder(
            cup_radius,
            cup_height,
            Eigen::Vector3d::UnitZ()
        )
    );

    auto& cup = mbp.AddRigidBody(
        "cup",
        cup_inertia
    );

    mbp.RegisterVisualGeometry(
        cup
    )

    mbp.AddJoint<WeldJoint>(
        "weld_cup",
        link2,
        RigidTransformd(
            Eigen::Vector3d(
                0,
                0,
                link_length
            )
        ), // X_PF
        cup,
        RigidTransformd::Identity(), // X_BM
        RigidTransformd::Identity() // X_FM
    );                                

    mbp.RegisterCollisionGeometry(
        cup,
        RigidTransformd::Identity(),
        drake::geometry::Sphere(
            cup_radius
        ),
        "cup_collision",
        CoulombFriction<double>(
            0.9,
            0.5
        )
    );

    
    // Ball object to juggle

    const double ball_radius = 0.04;

    const double ball_mass = 0.1;

    SpatialInertia<double> ball_inertia = SpatialInertia<double>::MakeFromCentralInertia(
        ball_mass,
        Eigen::Vector3d::Zero(),
        UnitInertia<double>::SolidSphere(
            ball_radius
        )
    );

    auto& ball = mbp.AddRigidBody("ball", ball_inertia);

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

    // Visualizing in meschat and building diagram

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

    joint1.set_angle(
        &plant_context,
        -0.5
    );

    joint2.set_angle(
        &plant_context,
        1.0
    );

    mbp.SetFreeBodyPose(
        &plant_context,
        ball,
        RigidTransformd(
            Eigen::Vector3d(
                0.6,
                0.0,
                0.9
            )
        )
    );

    mbp.SetFreeBodySpatialVelocity(
        &plant_context,
        ball,
        SpatialVelocity<double>(
            Eigen::Vector3d::Zero(), // angular
            Eigen::Vector3d( // translational (m/s)
                -1.0,
                0.0,
                3.0
            )
        )
    );

    simulator.Initialize();

    simulator.set_target_realtime_rate(1.0);

    simulator.AdvanceTo(5.0);

    std::cout << "Simulation done. Press Enter to exit..." << std::endl;

    std::cin.get();

    return 0;
}
