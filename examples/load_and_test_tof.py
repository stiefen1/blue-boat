from stable_baselines3 import PPO
import gymnasium as gym
from python_vehicle_simulator.gym.env_tof import GymNavEnvTOF
from python_vehicle_simulator.vehicles.otter import Otter, OtterParameters, OtterThrusterParameters
from python_vehicle_simulator.lib.actuator import Thruster
from python_vehicle_simulator.lib.weather import Wind, Current
from python_vehicle_simulator.utils.unit_conversion import DEG2RAD
from python_vehicle_simulator.lib.navigation import NavigationTOF
import numpy as np

# --- Create your environment (must match training setup) ---
dt = 0.02
thruster_params = OtterThrusterParameters()
otter = Otter(
    OtterParameters(),
    dt,
    actuators=[Thruster(xy=(0, 0.395), **vars(thruster_params), dt=dt), Thruster(xy=(0, -0.395), **vars(thruster_params), dt=dt)],
    navigation=NavigationTOF(
        tof_params={
            "range": 50,
            "angles": np.linspace(-25*DEG2RAD, 25*DEG2RAD, 3)
        }
    )
)

env = GymNavEnvTOF(
    own_vessel=otter,
    target_vessels=[],
    obstacles=[],
    wind=Wind(0, 0),
    current=Current(beta=-30.0*DEG2RAD, v=0.),
    render_mode="human",
    verbose=2
)

env = gym.wrappers.FlattenObservation(env)


# --- Load the trained PPO model ---
model = PPO.load("ppo_checkpoints\\ppo_gymnavenvtof_test_500000_steps", env=env)

# --- Test and visualize the agent ---
obs, info = env.reset()
for _ in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    # print("obs: ", obs)
    env.render() 
    if terminated or truncated:
        obs, info = env.reset()