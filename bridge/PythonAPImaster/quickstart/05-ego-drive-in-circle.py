#!/usr/bin/env python3
#
# Copyright (c) 2019-2021 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

from environs import Env
import lgsvl

print("Python API Quickstart #5: Ego vehicle driving in circle")
env = Env()

sim = lgsvl.Simulator(address = "169.254.42.175", port = 8181)



# if sim.current_scene == lgsvl.wise.DefaultAssets.map_borregasave:
#     sim.reset()
# else:
# sim.load(lgsvl.wise.DefaultAssets.map_sanfrancisco)
# sim.load("12da60a7-2fc9-474d-a62a-5cc08cb97fe8") #correct

# sim.load("3a2b5432-98e7-4f97-a852-3e2c65080ba4") #correct2

# sim.load("33b2965f-ebcd-4f9f-8cb0-a89ce20d49a0") #a test

# spawns = sim.get_spawn()

# state = lgsvl.AgentState()
# state.transform = spawns[0]
# forward = lgsvl.utils.transform_to_forward(spawns[0])
# state.transform.position += 5 * forward  # 5m forwards
# ego = sim.add_agent(env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5), lgsvl.AgentType.EGO, state)

# print("Current time = ", sim.current_time)
# print("Current frame = ", sim.current_frame)

# input("Press Enter to start driving for 30 seconds")

# # VehicleControl objects can only be applied to EGO vehicles
# # You can set the steering (-1 ... 1), throttle and braking (0 ... 1), handbrake and reverse (bool)
# c = lgsvl.VehicleControl()
# c.throttle = 0.3
# c.steering = -1.0
# # a True in apply_control means the control will be continuously applied ("sticky"). False means the control will be applied for 1 frame
# ego.apply_control(c, True)

# sim.run(30)


if sim.current_scene == lgsvl.wise.DefaultAssets.map_borregasave:
    sim.reset()
else:
    sim.load(lgsvl.wise.DefaultAssets.map_borregasave)


# if sim.current_scene == "12da60a7-2fc9-474d-a62a-5cc08cb97fe8":
#     sim.reset()
# else:
#     sim.load("12da60a7-2fc9-474d-a62a-5cc08cb97fe8")


# if sim.current_scene == lgsvl.wise.DefaultAssets.map_sanfrancisco:
#     sim.reset()
# else:
#     sim.load(lgsvl.wise.DefaultAssets.map_sanfrancisco)


# sim.load("12da60a7-2fc9-474d-a62a-5cc08cb97fe8") #correct
# sim.load(lgsvl.wise.DefaultAssets.map_borregasave)




spawns = sim.get_spawn()

state = lgsvl.AgentState()
state.transform = spawns[0]

ego = sim.add_agent(lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo6_modular, lgsvl.AgentType.EGO, state)
ego.connect_bridge(
    env.str("LGSVL__AUTOPILOT_0_HOST", "127.0.0.1"),
    env.int("LGSVL__AUTOPILOT_0_PORT", lgsvl.wise.SimulatorSettings.bridge_port)
)

# Dreamview setup
# dv = lgsvl.dreamview.Connection(sim, ego, env.str("LGSVL__AUTOPILOT_0_HOST", "169.254.42.170"))
dv = lgsvl.dreamview.Connection(sim, ego, env.str("LGSVL__AUTOPILOT_0_HOST", "127.0.0.1"))
# dv.set_hd_map('Borregas Ave')

dv.set_hd_map('San Francisco')
dv.set_vehicle('Lincoln2017MKZ_LGSVL')
modules = [
    'Localization',
    'Perception',
    'Transform',
    'Routing',
    'Prediction',
    'Planning',
    'Camera',
    'Traffic Light',
    'Control',
    "GPS"
]
destination = spawns[1].destinations[0]
dv.setup_apollo(destination.position.x, destination.position.z, modules)

sim.run()
