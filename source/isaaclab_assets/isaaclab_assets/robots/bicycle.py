"""Configuration for the Mujoco Bicycle robot."""

from __future__ import annotations

import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg
# from isaaclab.utils.assets import ISAAC_NUCLEUS_DIR

##
# Configuration
##

BICYCLE_CFG = ArticulationCfg(
    prim_path="{ENV_REGEX_NS}/Robot",
    spawn=sim_utils.UsdFileCfg(
        usd_path="/home/liuglu/IsaacLab_orca/robot_models/usd/bicycle/bicycle.usd",
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            max_depenetration_velocity=10.0,
            enable_gyroscopic_forces=True,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=False,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
            sleep_threshold=0.005,
            stabilization_threshold=0.001,
        ),
        copy_from_source=False,
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.0),
        # rot=(float(np.sqrt(2) / 2), float(np.sqrt(2) / 2), 0, 0),
        lin_vel=(-0.75, 0.0, 0.0),
        joint_pos={
            "frame_steering_joint": 0.0,
            "steering_wheel_joint": 0.0,
            "frame_wheel_joint": 0.0,
        },
        joint_vel={
            "frame_wheel_joint": 3.0,
            "steering_wheel_joint": 3.0,
        }
    ),
    actuators={
        "frame_steering_joint": ImplicitActuatorCfg(    # 车把
            joint_names_expr=["frame_steering_joint"],
            stiffness=50.0,
            damping=10.0,
            effort_limit_sim=1e6,
        ),
        "frame_wheel_joint": ImplicitActuatorCfg(       # 后轮
            joint_names_expr=["frame_wheel_joint"],
            stiffness=0.0,
            damping=20.0,
            velocity_limit_sim=3.0,
        ),
        "steering_wheel_joint": ImplicitActuatorCfg(    # 前轮
            joint_names_expr=["steering_wheel_joint"],
            stiffness=0.0,
            damping=0.0,
            velocity_limit_sim=3.0,
        ),
    },
)
"""Configuration for the Mujoco Bicycle robot."""
