import gymnasium as gym, numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
from python_vehicle_simulator.gym.env_tof import GymNavEnvTOF
from python_vehicle_simulator.vehicles.otter import Otter, OtterParameters, OtterThrusterParameters
from python_vehicle_simulator.lib.actuator import Thruster
from python_vehicle_simulator.lib.weather import Wind, Current
from python_vehicle_simulator.lib.navigation import NavigationTOF
from python_vehicle_simulator.utils.unit_conversion import DEG2RAD

# --- Create your environment ---
dt = 0.02

thruster_params = OtterThrusterParameters()
otter = Otter(
    OtterParameters(),
    dt,
    actuators=[Thruster(xy=(0, 0.395), **vars(thruster_params)), Thruster(xy=(0, -0.395), **vars(thruster_params))],
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
    current=Current(beta=-30.0*DEG2RAD, v=0.)
)

# --- Wrap with Gymnasium compatibility if needed ---
env = gym.wrappers.FlattenObservation(env)  # Optional, if using Dict obs

# --- Create the checkpoint callback ---
checkpoint_callback = CheckpointCallback(
    save_freq=100_000,
    save_path="./ppo_checkpoints/",
    name_prefix="ppo_gymnavenvtof_test"
)

# --- Train with Stable Baselines3 and checkpointing ---
# model = PPO(
#     "MlpPolicy",
#     env,
#     verbose=1,
#     tensorboard_log="./ppo_tensorboard/"
# )

model = PPO.load("ppo_gymnavenvtof", env=env) # load existing model

model.learn(
    total_timesteps=1_000_000,
    tb_log_name="quadratic-cost",
    callback=checkpoint_callback
) # tensorboard --logdir ./ppo_tensorboard/ in terminal and open http://localhost:6006/

# --- Save the model ---
model.save("ppo_gymnavenvtof_200_steps_max")

# --- Test the trained agent ---
obs, info = env.reset()
for _ in range(1000):
    action, _states = model.predict(obs, deterministic=True)
    obs, reward, terminated, truncated, info = env.step(action)
    env.render(mode="human")  # Visualize in 3D
    if terminated or truncated:
        obs, info = env.reset()