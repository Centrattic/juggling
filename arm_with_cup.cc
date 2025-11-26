#include "main.h"

#include <drake/geometry/shape_specification.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/weld_joint.h>
#include <drake/multibody/tree/unit_inertia.h>
#include <drake/multibody/plant/coulomb_friction.h>
#include <optional>

using drake::math::RigidTransformd;
using drake::multibody::RevoluteJoint;
using drake::multibody::UnitInertia;
using drake::multibody::SpatialInertia;
using drake::geometry::Cylinder;
using drake::geometry::Mesh;
using drake::multibody::MultibodyPlant;
using drake::multibody::WeldJoint;
using drake::multibody::CoulombFriction;

ArmWithCup AddTripleLinkArmWithCup(
    MultibodyPlant<double>* mbp,
    const std::string& name_prefix,
    const drake::multibody::Body<double>& parent_body,
    const RigidTransformd& X_PShoulder,  // pose of shoulder in world
    double link_length,
    double link_radius,
    double link_mass,
    double cup_radius,
    double cup_height
){
    
    /* Defining link and cup inertias */

    SpatialInertia<double> link_inertia = SpatialInertia<double>::MakeFromCentralInertia(
        link_mass,
        Eigen::Vector3d::Zero(),
        UnitInertia<double>::SolidCylinder(
            link_radius,
            link_length,
            Eigen::Vector3d::UnitZ()
        )
    );

    SpatialInertia<double> cup_inertia = SpatialInertia<double>::MakeFromCentralInertia(
        0.2,
        Eigen::Vector3d::Zero(),
        UnitInertia<double>::SolidCylinder(
            cup_radius,
            cup_height,
            Eigen::Vector3d::UnitZ()
        )
    );

    /* Link 1 and shoulder joint */

    auto& link1 = mbp->AddRigidBody(
        name_prefix + "link1",
        link_inertia
    );

    
    auto& shoulder = mbp->AddJoint<RevoluteJoint>(
        name_prefix + "shoulder",
        parent_body, // rotating juggler, not attached to world
        X_PShoulder,
        link1,
        std::nullopt,
        Eigen::Vector3d::UnitY()
    );

    mbp->RegisterVisualGeometry(
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
        name_prefix + "link1_visual",
        Eigen::Vector4d(
            0.1,
            0.1,
            1.0,
            1.0
        )
    ); // blue

    /* Link 2 and elbow joint */

    auto& link2 = mbp->AddRigidBody(
        name_prefix + "link2",
        link_inertia
    );


    auto& elbow = mbp->AddJoint<RevoluteJoint>(
        name_prefix + "elbow",
        link1,
        RigidTransformd(
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

    mbp->RegisterVisualGeometry(
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
        name_prefix + "link2_visual",
        Eigen::Vector4d(
            0.1,
            1.0,
            0.1,
            1.0
        )
    ); // green

    /* Cup rigid body */

    auto& cup = mbp->AddRigidBody(
        name_prefix + "cup",
        cup_inertia
    );

    mbp->RegisterVisualGeometry(
        cup,
        RigidTransformd::Identity(),
        drake::geometry::Mesh(
            "cup_cone.obj",
            1.0 // scale
        ),
        name_prefix + "cup_visual",
        Eigen::Vector4d(
            0.7,
            0.7,
            0.7,
            1.0
        )
    );

    mbp->AddJoint<WeldJoint>(
        name_prefix + "weld_cup",
        link2,
        std::nullopt,               // no separate frame on parent
        cup,
        std::nullopt,               // no separate frame on child
        RigidTransformd(            // fixed transform from parent to child
            Eigen::Vector3d(
                0,
                0,
                link_length
            )
        )
    );

    mbp->RegisterCollisionGeometry(
        cup,
        RigidTransformd(
            Eigen::Vector3d(
                0,
                0,
                cup_height / 2.0
            )
        ),
        drake::geometry::Cylinder( // approximating as cylinder still
            cup_radius,
            cup_height
        ),
        name_prefix + "cup_collision",
        CoulombFriction<double>(
            0.9,
            0.5
        )
    );

    ArmWithCup result;
    result.shoulder = &shoulder;
    result.elbow = &elbow;
    result.cup_body = &cup;
    return result;
    
}