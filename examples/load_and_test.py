from stable_baselines3 import PPO
import gymnasium as gym
from python_vehicle_simulator.gym.env import GymNavEnv
from python_vehicle_simulator.vehicles.otter import Otter, OtterParameters, OtterThrusterParameters
from python_vehicle_simulator.lib.actuator import Thruster
from python_vehicle_simulator.lib.weather import Wind, Current
from python_vehicle_simulator.utils.unit_conversion import DEG2RAD
import time

# --- Create your environment (must match training setup) ---
dt = 0.02
thruster_params = OtterThrusterParameters()
otter = Otter(
    OtterParameters(),
    dt,
    actuators=[
        Thruster(xy=(0, 0.395), **vars(thruster_params)),
        Thruster(xy=(0, -0.395), **vars(thruster_params))
    ]
)
env = GymNavEnv(
    own_vessel=otter,
    target_vessels=[],
    obstacles=[],
    wind=Wind(0, 0),
    current=Current(beta=-30.0*DEG2RAD, v=0.3),
    render_mode="human"
)

env = gym.wrappers.FlattenObservation(env)


# --- Load the trained PPO model ---
model = PPO.load("ppo_checkpoints\\ppo_gymnavenv_1000000_steps", env=env)

# --- Test and visualize the agent ---
obs, info = env.reset()
for _ in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    env.render() 
    if terminated or truncated:
        obs, info = env.reset()