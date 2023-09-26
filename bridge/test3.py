import os
import lgsvl
from environs import Env

import math
import random



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

    if sim.current_scene == lgsvl.wise.DefaultAssets.map_borregasave:
        sim.reset()
    else:
        sim.load(lgsvl.wise.DefaultAssets.map_borregasave)

    spawns = sim.get_spawn()




    #place agent
    state = lgsvl.AgentState()
    # state.transform = spawns[1]



    ego_position = xxx.get_lgsvl_state(587120.77, 4141205.08)
    state.transform = ego_position


    forward = lgsvl.utils.transform_to_forward(ego_position)
    right = lgsvl.utils.transform_to_right(ego_position)
    up = lgsvl.utils.transform_to_up(ego_position)

    ego = sim.add_agent(lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5_full_analysis, lgsvl.AgentType.EGO, state)
    ego.connect_bridge(
        env.str("LGSVL__AUTOPILOT_0_HOST", "169.254.42.170"),
        env.int("LGSVL__AUTOPILOT_0_PORT", lgsvl.wise.SimulatorSettings.bridge_port)
    )






    #place objects
    state = lgsvl.ObjectState()
    state.transform.rotation = 1.7*ego_position.rotation
    state.velocity = 0.000000001 * up
    state.angular_velocity = 0.000000001 * right


    start2 = (
            ego_position.position
            + (5 + 5) * forward
            - (1.6) * right
    )
    state.transform.position = start2

    o3 = sim.controllable_add("Bench0", state)
    print("Place Bench0")


    # Dreamview setup
    dv = lgsvl.dreamview.Connection(sim, ego, env.str("LGSVL__AUTOPILOT_0_HOST", "169.254.42.170"))
    dv.set_hd_map('Borregas Ave')
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
    destination = spawns[1].destinations[0]
    # dv.setup_apollo(destination.position.x, destination.position.z, modules)
    dv.setup_apollo(587120.77, 4141205.08, modules)

    sim.run()


if __name__ == '__main__':
    main()