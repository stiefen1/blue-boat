from python_vehicle_simulator.lib.simulator import Simulator
from python_vehicle_simulator.lib.env import NavEnv
from python_vehicle_simulator.vehicles.otter import Otter, OtterParameters, OtterThrusterParameters
from python_vehicle_simulator.states.states import Eta
from python_vehicle_simulator.lib.control import HeadingAutopilotTwoThrusters
from python_vehicle_simulator.lib.guidance import Guidance
from python_vehicle_simulator.lib.actuator import Thruster
from python_vehicle_simulator.lib.weather import Wind, Current
from python_vehicle_simulator.utils.unit_conversion import DEG2RAD

dt = 0.02

thruster_params = OtterThrusterParameters()
otter = Otter(
    OtterParameters(),
    dt,
    control=HeadingAutopilotTwoThrusters(
        actuators=[Thruster(xy=(0, 0.395), **vars(thruster_params)), Thruster(xy=(0, -0.395), **vars(thruster_params))],
        dt=dt,
        tau_X=100
    ),
    guidance=Guidance(desired_heading=100*DEG2RAD),
    actuators=[Thruster(xy=(0, 0.395), **vars(thruster_params)), Thruster(xy=(0, -0.395), **vars(thruster_params))]
)

env = NavEnv(
    own_vessel=otter,
    target_vessels=[],
    obstacles=[],
    dt=dt,
    current=Current(beta=-30.0*DEG2RAD, v=0.3),
    render_mode="human"
)

sim = Simulator(env)
sim.run(100)
