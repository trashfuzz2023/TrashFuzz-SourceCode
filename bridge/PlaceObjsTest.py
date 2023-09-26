#!/usr/bin/env python3

#
# Copyright (c) 2020-2021 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

from environs import Env
import lgsvl

print("Python API Quickstart #28: How to Add/Control Traffic Cone")
env = Env()

sim = lgsvl.Simulator(env.str("LGSVL__SIMULATOR_HOST", lgsvl.wise.SimulatorSettings.simulator_host), env.int("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port))
if sim.current_scene == lgsvl.wise.DefaultAssets.map_borregasave:
    sim.reset()
else:
    sim.load(lgsvl.wise.DefaultAssets.map_borregasave, 42)

spawns = sim.get_spawn()

state = lgsvl.AgentState()
forward = lgsvl.utils.transform_to_forward(spawns[0])
right = lgsvl.utils.transform_to_right(spawns[0])
up = lgsvl.utils.transform_to_up(spawns[0])
state.transform = spawns[0]

ego = sim.add_agent(env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5), lgsvl.AgentType.EGO, state)

# for i in range(10 * 3):
#     # Create controllables in a block
#     start = (
#         spawns[0].position
#         + (5 + (1.0 * (i // 6))) * forward
#         - (2 + (1.0 * (i % 6))) * right
#     )
#     end = start + 10 * forward
#
#     state = lgsvl.ObjectState()
#     state.transform.position = start
#     state.transform.rotation = spawns[0].rotation
#     # Set velocity and angular_velocity
#     state.velocity = 10 * up
#     state.angular_velocity = 6.5 * right
#
#     # add controllable
#     # o = sim.controllable_add("TrafficCone", state)
#     o = sim.controllable_add("Tank", state)



start0 = (
        spawns[0].position
        + (5 + 0) * forward
        - (2 + 0) * right
)

state = lgsvl.ObjectState()
state.transform.position = start0
state.transform.rotation = spawns[0].rotation
# Set velocity and angular_velocity
# state.velocity = 10 * up
# state.angular_velocity = 6.5 * right
state.velocity = 0.000000001 * up
state.angular_velocity = 0.000000001 * right

# add controllable
o1 = sim.controllable_add("TrafficCone", state)


# start1 = (
#         spawns[0].position
#         + (5 + 1) * forward
#         - (2 + 1) * right
# )
# state.transform.position = start1
#
# o2 = sim.controllable_add("Tree0", state)
#
#
#
# start2 = (
#         spawns[0].position
#         + (5 + 1) * forward
#         - (2 + 3) * right
# )
# state.transform.position = start2
#
# o3 = sim.controllable_add("Tree1", state)
#
#
#
# start2 = (
#         spawns[0].position
#         + (5 + 1) * forward
#         - (2 + 5) * right
# )
# state.transform.position = start2
#
# o3 = sim.controllable_add("Tree2", state)


start2 = (
        spawns[0].position
        + (5 + 1) * forward
        - (2 + 3) * right
)
state.transform.position = start2

o3 = sim.controllable_add("BinBlue", state)

start2 = (
        spawns[0].position
        + (5 + 1) * forward
        - (2 + 4) * right
)
state.transform.position = start2

o3 = sim.controllable_add("BinGreen", state)

start2 = (
        spawns[0].position
        + (5 + 1) * forward
        - (2 + 5) * right
)
state.transform.position = start2

o3 = sim.controllable_add("BinRed", state)

start2 = (
        spawns[0].position
        + (5 + 1) * forward
        - (2 + 6) * right
)
state.transform.position = start2

o3 = sim.controllable_add("BinYellow", state)


start2 = (
        spawns[0].position
        + (5 + 1) * forward
        - (2 + 7) * right
)
state.transform.position = start2

o3 = sim.controllable_add("Bin", state)


# start2 = (
#         spawns[0].position
#         + (5 + 1) * forward
#         - (2 + 7) * right
# )
# state.transform.position = start2
#
# o3 = sim.controllable_add("Bench0", state)
#
# start2 = (
#         spawns[0].position
#         + (5 + 1) * forward
#         - (2 + 9) * right
# )
# state.transform.position = start2
#
# o3 = sim.controllable_add("Bench1", state)


# start2 = (
#         spawns[0].position
#         + (5 + 0) * forward
#         - (2 + 3) * right
# )
# state.transform.position = start2
#
# o3 = sim.controllable_add("BusStopPole", state)

# start2 = (
#         spawns[0].position
#         + (4 + 0) * forward
#         - (2 + 5) * right
# )
# state.transform.position = start2
#
# o3 = sim.controllable_add("BigTrashBin", state)


start2 = (
        spawns[0].position
        + (4 + 0) * forward
        - (2 + 2) * right
)
state.transform.position = start2

o3 = sim.controllable_add("Cart", state)

start2 = (
        spawns[0].position
        + (4 + 0) * forward
        - (2 + 1) * right
)
state.transform.position = start2

o3 = sim.controllable_add("WarningStand", state)


start2 = (
        spawns[0].position
        + (4 + 0) * forward
        - (2 + 3) * right
)
state.transform.position = start2

o3 = sim.controllable_add("TrashBag", state)



start2 = (
        spawns[0].position
        + (4 + 0) * forward
        - (2 + 4) * right
)
state.transform.position = start2

o3 = sim.controllable_add("Hydrant", state)


start2 = (
        spawns[0].position
        + (4 + 0) * forward
        - (2 + 5) * right
)
state.transform.position = start2

o3 = sim.controllable_add("Rock5", state)


# start2 = (
#         spawns[0].position
#         + (4 + 0) * forward
#         - (2 + 5) * right
# )
# state.transform.position = start2
#
# o3 = sim.controllable_add("WaterTank0", state)


# print("\nAdded {} Traffic Cones".format(i + 1))

seconds = 10
input("\nPress Enter to run simulation for {} seconds".format(seconds))
print("\nRunning simulation for {} seconds...".format(seconds))
sim.run(seconds)

print("\nDone!")