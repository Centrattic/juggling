#include "main.h"

#include "consts.h"
#include <drake/geometry/shape_specification.h>
#include <drake/multibody/tree/revolute_joint.h>
#include <drake/multibody/tree/weld_joint.h>
#include <drake/multibody/tree/spatial_inertia.h>
#include <drake/multibody/math/spatial_velocity.h>
#include <drake/multibody/tree/unit_inertia.h>
#include <drake/multibody/plant/coulomb_friction.h>
#include <drake/math/rotation_matrix.h>
#include <cmath>
#include <optional>

using drake::math::RigidTransformd;
using drake::multibody::RevoluteJoint;
using drake::multibody::JointActuator;
using drake::multibody::UnitInertia;
using drake::multibody::SpatialInertia;
using drake::geometry::Cylinder;
using drake::geometry::Mesh;
using drake::multibody::MultibodyPlant;
using drake::multibody::WeldJoint;
using drake::multibody::CoulombFriction;

const drake::multibody::RigidBody<double>* BuildBall(
    MultibodyPlant<double>* mbp,
    const std::string& name
) {
    SpatialInertia<double> ball_inertia = SpatialInertia<double>::MakeFromCentralInertia(
        consts::ball_mass,
        Eigen::Vector3d::Zero(),
        UnitInertia<double>::SolidSphere(
            consts::ball_radius
        )
    );

    auto& ball = mbp->AddRigidBody(
        name,
        ball_inertia
    );

    mbp->RegisterVisualGeometry(
        ball,
        RigidTransformd::Identity(),
        drake::geometry::Sphere(
            consts::ball_radius
        ),
        name + "_visual",
        Eigen::Vector4d(
            1.0,
            0.0,
            0.0,
            1.0
        )
    );
    
    return &ball;

}

ArmWithCup AddTripleLinkArmWithCup(
    MultibodyPlant<double>* mbp,
    const std::string& name_prefix,
    const drake::multibody::Body<double>& parent_body,
    const RigidTransformd& X_PShoulder,  // pose of shoulder joint frame in parent
    const RigidTransformd& X_FM_link1,  // transform from joint frame to link1 body frame
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
        X_FM_link1,  // rotation to orient shoulder link frame
        Eigen::Vector3d::UnitY()
    );

    mbp->AddJointActuator(
        name_prefix + "shoulder_act",
        shoulder
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

    
    mbp->AddJointActuator(
        name_prefix + "elbow_act",
        elbow
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

    /* Link 3 and wrist joint */

    auto& link3 = mbp->AddRigidBody(
        name_prefix + "link3",
        link_inertia
    );

    auto& wrist = mbp->AddJoint<RevoluteJoint>(
        name_prefix + "wrist",
        link2,
        RigidTransformd(
            Eigen::Vector3d(
                0,
                0,
                link_length
            )
        ),
        link3,
        std::nullopt,
        Eigen::Vector3d::UnitY()
    );

    
    mbp->AddJointActuator(
        name_prefix + "wrist_act",
        wrist
    );

    mbp->RegisterVisualGeometry(
        link3,
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
        name_prefix + "link3_visual",
        Eigen::Vector4d(
            1.0,
            0.1,
            0.1,
            1.0
        )
    ); // red

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
        link3,
        std::nullopt,
        cup,
        std::nullopt,
        RigidTransformd(
            // drake::math::RotationMatrixd::MakeYRotation(
            //     -M_PI / 2.0
            // ),  // rotate mesh's +X -> +Z
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

    result.wrist = &wrist;

    result.cup_body = &cup;
    
    return result;
    
}