import os
import lgsvl
from environs import Env

import math
import random

from lgsvl_method import *



class CUS_LGVSL:
    def __init__(self, sim):
        self.sim = sim

    def transform_apollo_coord_to_lgsvl_coord(self, apollo_x, apollo_y):
        point = self.sim.map_from_gps(northing = apollo_y, easting = apollo_x)
        return point

    def find_the_point_on_line_for_lgsvl(self, point):
        sx = point.position.x
        sy = point.position.y
        sz = point.position.z
        adjusted_point = lgsvl.Vector(sx , sy, sz)
        return self.sim.map_point_on_lane(adjusted_point) 

    def Add_heading_to_the_point(self, point):
        gps = self.sim.map_to_gps(point)
        adjuested_point = self.sim.map_from_gps(
                northing=gps.northing,
                easting=gps.easting,
                altitude=gps.altitude,
                orientation=gps.orientation,
            )
        return adjuested_point

    def get_lgsvl_state(self, start_point_x, start_point_y):
        start_point = self.transform_apollo_coord_to_lgsvl_coord(start_point_x,start_point_y)          
        start_point = self.find_the_point_on_line_for_lgsvl(start_point)
        start_point = self.Add_heading_to_the_point(point=start_point)
        return start_point


def main():
    env = Env()
    sim = lgsvl.Simulator(
        env.str("LGSVL__SIMULATOR_HOST", "169.254.42.175"),
        env.int("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port)
    )

    xxx = CUS_LGVSL(sim)

    # if sim.current_scene == "12da60a7-2fc9-474d-a62a-5cc08cb97fe8":
    if sim.current_scene == lgsvl.wise.DefaultAssets.map_borregasave:
    # if sim.current_scene == lgsvl.wise.DefaultAssets.map_cubetown:
        sim.reset()
    else:
        sim.load(lgsvl.wise.DefaultAssets.map_borregasave)
        # sim.load(lgsvl.wise.DefaultAssets.map_cubetown)
        

    spawns = sim.get_spawn()




    #place agent
    state = lgsvl.AgentState()
    state.transform = spawns[0]



    # ego_position = xxx.get_lgsvl_state(587120.77, 4141205.08)
    # ego_position = xxx.get_lgsvl_state(587004.99, 4141387.64)
    # state.transform = ego_position
    ego_position = state.transform

    forward = lgsvl.utils.transform_to_forward(ego_position)
    # print("forward: "+ str(forward))
    right = lgsvl.utils.transform_to_right(ego_position)
    # print("right: "+ str(right))
    up = lgsvl.utils.transform_to_up(ego_position)
    # print("up: "+ str(up))


    ego = sim.add_agent(lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5_full_analysis, lgsvl.AgentType.EGO, state)
    # ego = sim.add_agent(lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo6_modular, lgsvl.AgentType.EGO, state)
    # ego = sim.add_agent('2e966a70-4a19-44b5-a5e7-64e00a7bc5de', lgsvl.AgentType.EGO, state)
    # source: https://github.com/MingfeiCheng/AV-Fuzzer/blob/aa0f96c35088502189f7a9343d9ec7b0cee46b55/simulation/simulator.py#L12
    ego.connect_bridge(
        env.str("LGSVL__AUTOPILOT_0_HOST", "169.254.42.170"),
        env.int("LGSVL__AUTOPILOT_0_PORT", lgsvl.wise.SimulatorSettings.bridge_port)
    )






    #place objects
    state = lgsvl.ObjectState()
    state.transform.rotation = ego_position.rotation
    state.velocity = lgsvl.Vector(0, 0, 0)
    state.angular_velocity = lgsvl.Vector(0, 0, 0)

    # state.transform.rotation = lgsvl.Vector(0,20,0)

    # add controllable
    # o1 = sim.controllable_add("TrafficCone", state)
    # print("Place TrafficCone")

    # start2 = (
    #         ego_position.position
    #         + (5 + 25) * forward
    #         - (1.6) * right
    # )
    # state.transform.position = start2

    # o3 = sim.controllable_add("Bench1", state)
    # print("Place Bench1")



    # ego_position1 = xxx.transform_apollo_coord_to_lgsvl_coord(587004.99, 4141387.64)
    # sx = ego_position1.position.x
    # sy = ego_position1.position.y
    # sz = ego_position1.position.z

    start2 = (
            ego_position.position
            + (10) * forward
            + (2.3) * right
    )


    start3 = sim.map_point_on_lane(start2)
    start2.y = start3.position.y

    state.transform.position = start2

    state.transform.rotation = lgsvl.Vector(0, 0, 0)
    print(start2)

    o3 = sim.controllable_add("BinGreen", state)
    print("Place BinGreen")

    start2 = (
            ego_position.position
            + (10) * forward
            + (5.3) * right
    )
    start2 = sim.map_point_on_lane(start2)
    state.transform.position = start2.position

    print(start2.position)

    state.transform.rotation = lgsvl.Vector(0, 90, 0)

    o3 = sim.controllable_add("Bin", state)
    print("Place Bin")

    # start2 = (
    #         spawns[0].position
    #         + (5 + 12) * forward
    #         - (2 - 2) * right
    # )
    # state.transform.position = start2

    # o3 = sim.controllable_add("BinGreen", state)
    # print("Place BinGreen")


    # start2 = (
    #         spawns[0].position
    #         + (5 + 12) * forward
    #         - (2 - 1) * right
    # )
    # state.transform.position = start2

    # o3 = sim.controllable_add("BinRed", state)
    # print("Place BinRed")



    # add NPCs
    # sx = spawns[0].position.x
    # sy = spawns[0].position.y
    # sz = spawns[0].position.z

    # mindist = 10.0
    # maxdist = 40.0

    # random.seed(0)

    # print("Spawn 10 random NPCs on lanes")
    # for i in range(3):
    #     # input("Press Enter to spawn NPC ({})".format(i + 1))

    #     # Creates a random point around the EGO
    #     angle = random.uniform(0.0, 2 * math.pi)
    #     dist = random.uniform(mindist, maxdist)

    #     point = lgsvl.Vector(sx + dist * math.cos(angle), sy, sz + dist * math.sin(angle))

    #     # Creates an NPC on a lane that is closest to the random point
    #     state = lgsvl.AgentState()
    #     state.transform = sim.map_point_on_lane(point)
    #     sim.add_agent("Sedan", lgsvl.AgentType.NPC, state)


    # Dreamview setup
    dv = lgsvl.dreamview.Connection(sim, ego, env.str("LGSVL__AUTOPILOT_0_HOST", "169.254.42.170"))
    dv.set_hd_map('Borregas Ave')
    # dv.set_hd_map('Cube Town')
    # dv.set_hd_map('Single Lane Road')
    dv.set_vehicle('Lincoln2017MKZ_LGSVL')
    dv.set_setup_mode('Mkz Lgsvl')

    modules = [
        'Localization',
        'Perception',
        'Transform',
        'Routing',
        'Prediction',
        'Planning',
        'Traffic Light',
        'Control'
    ]
    # destination = spawns[1].destinations[0]
    destination = spawns[0].destinations[0]
    dv.setup_apollo(destination.position.x, destination.position.z, modules)
    # dv.setup_apollo(587120.77, 4141205.08, modules)
    # dv.setup_apollo(587064.15, 4141615.15, modules)

    # destination_apollo = [587120.77, 4141205.08]


    # bridge_custom = CyberBridgeInstance()
    # trace_for_process = bridge_custom.register(destination_apollo, self.map)





    sim.run()


if __name__ == '__main__':
    main()