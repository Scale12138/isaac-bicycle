import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg
from isaaclab.envs import ManagerBasedRLEnvCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import RewardTermCfg as RewTerm
# from isaaclab.managers import SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.terrains import TerrainImporterCfg
from isaaclab.utils import configclass

import isaaclab_tasks.manager_based.classic.humanoid.mdp as mdp

##
# Pre-defined configs
##
from isaaclab_assets.robots.bicycle import BICYCLE_CFG  # isort: skip


@configclass
class MySceneCfg(InteractiveSceneCfg):
    """Configuration for the terrain scene with an bicycle robot."""

    # terrain
    terrain = TerrainImporterCfg(
        prim_path="/World/ground",
        terrain_type="plane",
        collision_group=-1,
        physics_material=sim_utils.RigidBodyMaterialCfg(
            friction_combine_mode="average",
            restitution_combine_mode="average",
            static_friction=1.0,
            dynamic_friction=1.0,
            restitution=0.0,
        ),
        debug_vis=False,
    )

    # robot
    robot = BICYCLE_CFG.replace(prim_path="{ENV_REGEX_NS}/Robot")   # type: ignore

    # lights
    light = AssetBaseCfg(
        prim_path="/World/light",
        spawn=sim_utils.DistantLightCfg(color=(0.75, 0.75, 0.75), intensity=3000.0),
    )

##
# MDP settings
##


@configclass
class ActionsCfg:
    """Action specifications for the MDP."""

    joint_position = mdp.JointPositionActionCfg(
        asset_name="robot",
        joint_names=["frame_steering_joint"],
        scale=1.0,
    )
    joint_velocity = mdp.JointVelocityActionCfg(
        asset_name="robot",
        joint_names=["frame_wheel_joint"],
        scale=1.0,
    )


@configclass
class ObservationsCfg:
    """Observation specifications for the MDP."""

    @configclass
    class PolicyCfg(ObsGroup):
        """Policy observation group."""
        base_yaw_roll = ObsTerm(func=mdp.base_yaw_roll)     # roll, (1,)
        base_ang_vel = ObsTerm(func=mdp.base_ang_vel)       # roll_ang_vel, (1,)
        joint_pos = ObsTerm(func=mdp.joint_pos)             # steer_ang, (1,)
        base_lin_vel = ObsTerm(func=mdp.base_lin_vel)       # base_lin_vel, (1,)
        actions = ObsTerm(func=mdp.last_action)

        def __post_init__(self):
            self.enable_corruption = False
            self.concatenate_terms = True

    # observation groups
    policy: PolicyCfg = PolicyCfg()


@configclass
class EventCfg:
    """Configuration for events."""

    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={"pose_range": {}, "velocity_range": {}},
    )

    reset_robot_joints = EventTerm(
        func=mdp.reset_joints_by_offset,
        mode="reset",
        params={
            "position_range": (-0.1, 0.1),
            "velocity_range": (0, 0),
        },
    )


@configclass
class RewardsCfg:
    """Reward terms for self-balancing bicycle MDP."""

    # (1) Reward for moving forward
    progress = RewTerm(
        func=mdp.progress_reward,    # type: ignore
        weight=2.0,
        params={"target_pos": (-1000.0, 0.0, 0.0)}
    )
    # (2) Stay alive bonus
    alive = RewTerm(
        func=mdp.is_alive,
        weight=3.0,
    )
    # (3) Reward for non-upright posture
    upright = RewTerm(
        func=mdp.upright_posture_bonus,
        weight=1.0,
        params={"threshold": 0.9})
    # (4) Reward for moving in the right direction
    move_to_target = RewTerm(
        func=mdp.move_to_target_bonus,
        weight=0.1,
        params={"threshold": 5, "target_pos": (-1000.0, 0.0, 0.0)}
    )
    # (5) Penalty for large action commands
    action_l2 = RewTerm(func=mdp.action_l2, weight=-0.01)


@configclass
class TerminationsCfg:
    """Termination terms for the MDP."""

    # (1) Terminate if the episode length is exceeded
    time_out = DoneTerm(func=mdp.time_out, time_out=True)
    # (2) bad_orientation
    bad_orientation = DoneTerm(func=mdp.bad_orientation, params={"limit_angle": 0.8})


@configclass
class BicycleEnvCfg(ManagerBasedRLEnvCfg):
    """Configuration for the self-balancing bicycle MDP."""

    # Scene settings
    scene: MySceneCfg = MySceneCfg(num_envs=4096, env_spacing=5.0)
    # Basic settings
    observations: ObservationsCfg = ObservationsCfg()
    actions: ActionsCfg = ActionsCfg()
    # MDP settings
    rewards: RewardsCfg = RewardsCfg()
    terminations: TerminationsCfg = TerminationsCfg()
    # Event settings
    events: EventCfg = EventCfg()

    def __post_init__(self):
        """Post initialization."""
        # general settings
        self.decimation = 2
        self.episode_length_s = 32.0
        # simulation settings
        self.sim.dt = 1 / 120.0
        self.sim.render_interval = self.decimation
        self.sim.physx.bounce_threshold_velocity = 0.2
        # default friction material
        self.sim.physics_material.static_friction = 1.0
        self.sim.physics_material.dynamic_friction = 1.0
        self.sim.physics_material.restitution = 0.0
