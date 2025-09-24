from python_vehicle_simulator.lib.simulator import Simulator
from python_vehicle_simulator.lib.env import NavEnv
from python_vehicle_simulator.vehicles.revolt3 import Revolt3DOF, RevoltBowThrusterParams, RevoltSternThrusterParams, RevoltParameters3DOF, RevoltThrusterParameters
from python_vehicle_simulator.states.states import Eta
from python_vehicle_simulator.lib.control import HeadingAutopilotTwoThrusters
from python_vehicle_simulator.lib.guidance import Guidance
from python_vehicle_simulator.lib.navigation import NavigationTOF, NavigationWithNoise, NavigationRevolt3WithEKF
from python_vehicle_simulator.lib.actuator import Thruster
from python_vehicle_simulator.lib.weather import Wind, Current
from python_vehicle_simulator.utils.unit_conversion import DEG2RAD
from python_vehicle_simulator.lib.map import RandomMapGenerator
from python_vehicle_simulator.lib.actuator import AzimuthThruster
from python_vehicle_simulator.lib.mpc import MPCPathTrackingRevolt
from python_vehicle_simulator.lib.path import PWLPath
from python_vehicle_simulator.lib.guidance import PathFollowingGuidance
from python_vehicle_simulator.lib.diagnosis import GradientDescentDiagnosisRevolt3Actuators, ParticleFilterDiagnosisRevolt3Actuators
import numpy as np, matplotlib.pyplot as plt

dt = 0.2
horizon = 50

vessel = Revolt3DOF(
        params=RevoltParameters3DOF(),
        dt=dt,
        actuators=[
            AzimuthThruster(xy=(-1.65, -0.15), **vars(RevoltSternThrusterParams())), #, faults=[{'type': 'loss-of-efficiency', 't0': 0, 'efficiency': 0.0}]),
            AzimuthThruster(xy=(-1.65, 0.15), **vars(RevoltSternThrusterParams())), #, faults=[{'type': 'loss-of-efficiency', 't0': 20, 'efficiency': 0.}]),
            AzimuthThruster(xy=(1.15, 0.0), **vars(RevoltBowThrusterParams()))
        ],
        control=MPCPathTrackingRevolt(
            vessel_params=RevoltParameters3DOF(),
            actuator_params=RevoltThrusterParameters(),
            dt=dt,
            horizon=horizon
        ),
        guidance=PathFollowingGuidance(
            path=PWLPath([
                (0., 0.),
                (20, 10.),
                (40., 15.),
                (60., 30.),
                (80., 40.)
            ]),
            horizon=horizon,
            dt=dt,
            desired_speed=1
        ),
        # navigation=NavigationWithNoise(np.array([0, 0, 0, 0, 0, 0]), np.array([0, 0, 0, 0, 0, 0])),
        # navigation=NavigationRevolt3WithEKF(np.array([0, 0, 0, 0, 0, 0]), np.array([0, 0, 0, 0, 0, 0]), dt=dt),
        # diagnosis=ParticleFilterDiagnosisRevolt3Actuators(
        #     eta=np.array(6*[0.0]),
        #     nu=np.array(6*[0.0]),
        #     dt=dt,
        #     meas_cov=np.eye(6)*1e-7, # 1e-7 was the best without noise, 1e-8 -> numerical issues
        #     process_cov=np.eye(3)*1e-4, # 1e-4 was the best without noise
        #     n_particles=100,
        #     delta0=np.array([1., 1., 1.]),
        # )
        # diagnosis=GradientDescentDiagnosisRevolt3Actuators(
        #     eta=np.array(6*[0.0]),
        #     nu=np.array(6*[0.0]),
        #     dt=dt,
        #     delta0=np.array([1., 1., 1.]),
        #     lr=5e2
        # )
    )

map_generator = RandomMapGenerator(
        (-100, 100),
        (-100, 100),
        (20, 30),
        min_dist=5
    )

env = NavEnv(
    own_vessel=vessel,
    target_vessels=[],
    obstacles=map_generator.get([(vessel.eta[0], vessel.eta[1])], min_density=0.3)[0],
    dt=dt,
    current=Current(beta=-30.0*DEG2RAD, v=0.)
)

sim = Simulator(
        env,
        dt=dt,
        render_mode="human",
        verbose=2,
        skip_frames=5,
        window_size=(10, 10)
    )

print("=== Running Simulation (no rendering) ===")
# Run simulation without rendering but store data
# Pass simple control commands to move the vessel forward
# control_commands = [np.array([0, 100]), np.array([0, 100]), np.array([0, 0])]  # Simple thrust commands
sim.run(tf=100, render=True, store_data=True)

# After calling plot_gnc_data_multi, add:
nav_data = sim.simulation_data['gnc_data']['navigation']
vessel_data = sim.simulation_data['own_vessel_states']

# print("\n=== Starting Replay ===")
# Replay the simulation with visualization
# sim.replay(speed_factor=5.0)  # 2x speed replay

fig1 = sim.plot_gnc_data_multi([
    'navigation.eta[0]',
    'vessel.eta[0]'
    ], x_path=['navigation.eta[1]', 'vessel.eta[1]'])
vessel.guidance.path.plot(ax=fig1.axes[0])
fig2 = sim.plot_gnc_data_multi([
    'actuators[0].info.u_actual[1]',
    'actuators[0].info.u[1]',
    'actuators[1].info.u_actual[1]',
    'actuators[1].info.u[1]',
    'actuators[2].info.u_actual[1]',
    'actuators[2].info.u[1]'
    ])
# fig3 = sim.plot_gnc_data('actuators[0].info.efficiency')
# fig3 = sim.plot_gnc_data_multi([
#     'actuators[0].info.efficiency',
#     'diagnosis.diagnosis.delta[0]'
#     ])
# fig4 = sim.plot_gnc_data_multi([
#     'actuators[0].info.efficiency',
#     'diagnosis.diagnosis.delta[0]',
#     'actuators[1].info.efficiency',
#     'diagnosis.diagnosis.delta[1]',
#     'actuators[2].info.efficiency',
#     'diagnosis.diagnosis.delta[2]'
#     ])
fig5 = sim.plot_gnc_data_multi([
    'vessel.nu[0]', 'vessel.nu[1]'
])
# fig5 = sim.plot_gnc_data_multi([
#     'diagnosis.info.top-k-particles[0]',
# ])
plt.show(block=True)
