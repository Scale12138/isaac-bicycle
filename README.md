# Introduction
This project is based on the NVIDIA Isaac Sim platform and uses reinforcement learning methods to achieve dynamic balance control of bicycles. The project was modified from IsaacLab's instance code, using a custom USD model to achieve more realistic simulation of bicycle physical characteristics and learning of balance strategies.

The dynamic balance problem of bicycles is a classic problem in robotics and control theory. Unlike traditional inverted pendulums, bicycle systems have more complex nonlinear dynamic characteristics, including gyroscopic effects, steering geometry, and tire ground contact dynamics.

Main objectives:
- Trying to build a bicycle physics model in Isaac Sim
- Developing a control strategy that can maintain bicycle balance
- Optimizing control parameters through reinforcement learning
- Validate the robustness of the strategy under different initial conditions and disturbances

Differences from the original IsaacLab code:
- Using custom bicycle USD models
- Modifying the state space and action space, and achieving steering control
- Modifying reward function design to better guide the learning process


# Independencies
- Isaac Sim 4.5.0
- Python 3.10.16
- PyTorch 2.5.1
- CUDA 11.8

# Running
1. Activate conda environment and cd into the project directory
```bash
conda activate isaac
cd /path/to/project
```
2. Train balancing model
```bash
python scripts/reinforcement_learning/rsl_rl/train.py --task Isaac-Bicycle-v0 --headless
```
3. Play
```bash
python scripts/reinforcement_learning/rsl_rl/play.py --task Isaac-Bicycle-v0
```

# About our modifications
1. Add custom bicycle URDF/USD models under `/robot_models`

2. Add `bicycle.py` under `/source/isaaclab_assets/isaaclab_assets/robots/`

3. Add `rsl_rl_ppo_cfg.py` under `source/isaaclab_tasks/isaaclab_tasks/manager_based/classic/bicycle/agents/`

4. Add `bicycle_env_cfg.py` under `source/isaaclab_tasks/isaaclab_tasks/manager_based/classic/bicycle/` 

5. `usd_path` in `bicycle.py` **must be modified before running**

6. Part of joint params are initialized in `bicycle.py`, some are specified in IsaacSim GUI (such as mass, effort, etc.)

7. Model of bicycle consists of 4 rigid bodies: 

   - frame
   - steer
   - front wheel
   - rear wheel

   and 3 joints: 

   - frame_steering_joint
   - steering_wheel_joint
   - frame_wheel_joint

   where frame_steering_joint is position controlled, steering_wheel_joint is freely rotated, and frame_wheel_joint is velocity controlled.

8. Balancing policy is learned through RL, where the actor receives the state of the bicycle and outputs the steering angle and rear wheel velocity.

9. What we're confused about:
   - The modification of physical parameters in the USD model (including rigid bodies and joints) cannot correspond to the physical simulation effect and the real world;
   - Some non-dynamic situations may occur (like rear-wheel slippage).