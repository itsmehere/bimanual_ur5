# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Configuration for the Universal Robots.

The following configuration parameters are available:

* :obj:`UR10_CFG`: The UR10 arm without a gripper.

Reference: https://github.com/ros-industrial/universal_robot
"""

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
from isaaclab.utils.assets import ISAACLAB_NUCLEUS_DIR

##
# Configuration
##


UR10_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"{ISAACLAB_NUCLEUS_DIR}/Robots/UniversalRobots/UR10/ur10_instanceable.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        activate_contact_sensors=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": -1.712,
            "elbow_joint": 1.712,
            "wrist_1_joint": 0.0,
            "wrist_2_joint": 0.0,
            "wrist_3_joint": 0.0,
        },
    ),
    actuators={
        "arm": ImplicitActuatorCfg(
            joint_names_expr=[".*"],
            velocity_limit=100.0,
            effort_limit=87.0,
            stiffness=800.0,
            damping=40.0,
        ),
    },
)
"""Configuration of UR-10 arm using implicit actuator models."""

BIMANUAL_UR5_CFG = ArticulationCfg(
    spawn=sim_utils.UrdfFileCfg(
        asset_path=f"/home/mrao/frankatask/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/lift/config/bimanual_ur5/ur5e_allegro/robots/dual_ur5e_allegro_real_v2.urdf",
        usd_dir=f"/home/mrao/frankatask/IsaacLab/source/isaaclab_assets/data/Robots/bimanual_ur5",
        usd_file_name=f"dual_ur5e_allegro_real_v2.usd",
        fix_base=True,
        visible=True,
        activate_contact_sensors=False,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=5.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
        ),
        joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
            drive_type="force",
            target_type="position",
            gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=None, damping=None)
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        joint_pos={
            # Left UR5 Arm
            "shoulder_pan_joint": 0.0,
            "shoulder_lift_joint": 1.57,
            "elbow_joint": -1.57,
            "wrist_1_joint": 0.0,
            "wrist_2_joint": 0.0,
            "wrist_3_joint": 3.14,

            # Left Allegro Hand
            "joint_0_0": 0.0,
            "joint_1_0": 0.0,
            "joint_2_0": 0.0,
            "joint_3_0": 0.0,
            "joint_4_0": 0.0,
            "joint_5_0": 0.0,
            "joint_6_0": 0.0,
            "joint_7_0": 0.0,
            "joint_8_0": 0.0,
            "joint_9_0": 0.0,
            "joint_10_0": 0.0,
            "joint_11_0": 0.0,
            "joint_12_0": 0.3,
            "joint_13_0": 0.3,
            "joint_14_0": 0.3,
            "joint_15_0": 0.3,

            # Right UR5 Arm
            "shoulder_pan_joint_r": 0.0,
            "shoulder_lift_joint_r": 1.57,
            "elbow_joint_r": -1.57,
            "wrist_1_joint_r": 0.0,
            "wrist_2_joint_r": 0.0,
            "wrist_3_joint_r": 3.14,

            # Right Allegro Hand
            "joint_0_0_r": 0.0,
            "joint_1_0_r": 0.0,
            "joint_2_0_r": 0.0,
            "joint_3_0_r": 0.0,
            "joint_4_0_r": 0.0,
            "joint_5_0_r": 0.0,
            "joint_6_0_r": 0.0,
            "joint_7_0_r": 0.0,
            "joint_8_0_r": 0.0,
            "joint_9_0_r": 0.0,
            "joint_10_0_r": 0.0,
            "joint_11_0_r": 0.0,
            "joint_12_0_r": 0.3,
            "joint_13_0_r": 0.3,
            "joint_14_0_r": 0.3,
            "joint_15_0_r": 0.3,
        },
    ),
    actuators={
        "left_arm": ImplicitActuatorCfg(
            joint_names_expr=[".*_joint$"],
            velocity_limit=3.14,
            effort_limit=150.0,
            stiffness=80.0,
            damping=0.0,
        ),
        "left_hand": ImplicitActuatorCfg(
            joint_names_expr=["joint_[0-15]_0"],
            velocity_limit=3.14,
            effort_limit=10.0,
            stiffness=2e3,
            damping=1e2,
        ),
        "right_arm": ImplicitActuatorCfg(
            joint_names_expr=[".*_joint_r$"],
            velocity_limit=3.14,
            effort_limit=150.0,
            stiffness=80.0,
            damping=0.0,
        ),
        "right_hand": ImplicitActuatorCfg(
            joint_names_expr=["joint_[0-15]_0_r"],
            velocity_limit=3.14,
            effort_limit=10.0,
            stiffness=2e3,
            damping=1e2,
        ),
    },
    soft_joint_pos_limit_factor=1.0,
)
"""Configuration of Bimanual UR-5 arm using implicit actuator models."""
