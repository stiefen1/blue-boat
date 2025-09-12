from python_vehicle_simulator.lib.simulator import Simulator
from python_vehicle_simulator.lib.env import NavEnv
from python_vehicle_simulator.vehicles.otter import Otter, OtterParameters, OtterThrusterParameters
from python_vehicle_simulator.states.states import Eta
from python_vehicle_simulator.lib.control import UserInputControlTwoTrusters
from python_vehicle_simulator.lib.guidance import Guidance
from python_vehicle_simulator.lib.navigation import NavigationTOF
from python_vehicle_simulator.lib.actuator import Thruster
from python_vehicle_simulator.lib.weather import Wind, Current
from python_vehicle_simulator.utils.unit_conversion import DEG2RAD
from python_vehicle_simulator.lib.map import RandomMapGenerator
import numpy as np

dt = 0.02

thruster_params = OtterThrusterParameters()
otter = Otter(
    OtterParameters(),
    dt,
    control=UserInputControlTwoTrusters(),
    guidance=Guidance(desired_heading=100*DEG2RAD),
    navigation=NavigationTOF(
        tof_params={
            "range":50,
            "angles":np.linspace(-15*DEG2RAD, 15*DEG2RAD, 5)
        }
    ),
    actuators=[Thruster(xy=(0, 0.395), **vars(thruster_params)), Thruster(xy=(0, -0.395), **vars(thruster_params))]
)

map_generator = RandomMapGenerator(
        (-100, 100),
        (-100, 100),
        (20, 30),
        min_dist=5
    )

env = NavEnv(
    own_vessel=otter,
    target_vessels=[],
    obstacles=map_generator.get([(otter.eta[0], otter.eta[1])], min_density=0.3)[0],
    dt=dt,
    current=Current(beta=-30.0*DEG2RAD, v=0.0),
    render_mode="human",
    verbose=2,
    skip_frames=10
)

sim = Simulator(env)
sim.run(100)
