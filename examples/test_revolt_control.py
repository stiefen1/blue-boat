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
import numpy as np

dt = 0.1
horizon = 20

vessel = Revolt3DOF(
        params=RevoltParameters3DOF(),
        dt=dt,
        actuators=[
            AzimuthThruster(xy=(-1.65, -0.15), **vars(RevoltSternThrusterParams())),
            AzimuthThruster(xy=(-1.65, 0.15), **vars(RevoltSternThrusterParams())),
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
        # navigation=NavigationWithNoise()
        navigation=NavigationRevolt3WithEKF(x0=np.array([0, 0, 0, 0, 0, 0]), dt=dt)
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
    current=Current(beta=-30.0*DEG2RAD, v=0.),
    render_mode="human",
    verbose=2,
    skip_frames=0,
    window_size=(10, 10)
)

sim = Simulator(env)
sim.run(100)