#!/usr/bin/env python
import os
import json
import asyncio
import websockets
from enum import Enum
import threading


from lgsvl_method import *

import lgsvl
from environs import Env

from lgsvl.simulator import WeatherState

# WeatherState = namedtuple("WeatherState", "rain fog wetness cloudiness damage")
# WeatherState.__new__.__defaults__ = (0,) * len(WeatherState._fields)

from lgsvl.dreamview import CoordType

# import sys
# sys.path.append("..") 
from map_for_bridge import get_map_info
# sys.path.remove("..")

import time




class Server:
    def __init__(self):
        self.SIMULATOR_HOST = os.environ.get("SIMULATOR_HOST", "169.254.42.175")
        self.SIMULATOR_PORT = int(os.environ.get("SIMULATOR_PORT", 8181))
        self.BRIDGE_HOST = os.environ.get("LGSVL__AUTOPILOT_0_HOST", "169.254.42.170")
        self.BRIDGE_PORT = int(os.environ.get("LGSVL__AUTOPILOT_0_PORT", 9090))
        self.sim = 0
        self.ego = 0
        self.destination_apollo = []
        self.AgentNames = []
        self.result = {}
        self.map = None
        # self.ADS_model = 'apollo5'
        self.ADS_model = 'apollo_latest'
        self._Accident = False
        self.exception = False
        # self.map_info = None

    def AVS_on_collision(self, agent1, agent2, contact):
        self._Accident = True
        print("Accident!")


    def start_lgsvl(self):
        # time.sleep(1.5)
        self.sim.run()

    def start_bridge(self):
        bridge_custom = CyberBridgeInstance()
        trace_for_process = bridge_custom.register(self.destination_apollo, self.map)

        self.result["testFailures"] = trace_for_process["testFailures"]
        if self._Accident:
            self.result["testFailures"].append("Accident!")
            self._Accident = False
        if trace_for_process["testFailures"] == []:
            self.result["testResult"] = "PASS"
        else:
            self.result["testResult"] = "Fail"

        self.result["minEgoObsDist"] = trace_for_process["minEgoObsDist"]
        self.result["destinationReached"] = trace_for_process["destinationReached"]
        self.result["trace"] = trace_for_process["trace"]
        self.result["completed"] = trace_for_process["completed"]

        self._Accident = False
        # self.sim.reset()
        # print("reset the LGSVL")
        self.sim.close()


    def transform_apollo_coord_to_lgsvl_coord(self, apollo_x, apollo_y):
        point = self.sim.map_from_gps(northing = apollo_y, easting = apollo_x)
        return point

    def find_the_point_on_line_for_lgsvl(self, point):
        sx = point.position.x
        sy = point.position.y
        sz = point.position.z
        adjusted_point = lgsvl.Vector(sx , sy, sz)

        return self.sim.map_point_on_lane(adjusted_point) 

        # return self.sim.map_point_on_lane(adjusted_point)

    def Add_heading_to_the_point(self, heading, point):
        gps = self.sim.map_to_gps(point)
        adjuested_point = self.sim.map_from_gps(
                northing=gps.northing,
                easting=gps.easting,
                altitude=gps.altitude,
                orientation=gps.orientation,
                # orientation=gps.orientation + heading,
            )
        return adjuested_point


    def process_with_lane_position(self, lane_position):
        _map = self.map
        map_info = get_map_info(_map)
        map_info.get_lane_config()
        # print(map_info)
        lane_point = [lane_position["lane"],lane_position["offset"]]
        p = map_info.get_position(lane_point)
        return p

    def Deal_with_WAYpoint(self,start_point):
        if start_point.__contains__('lane_position'):
            pos = self.process_with_lane_position(start_point["lane_position"])  
            pos = self.transform_apollo_coord_to_lgsvl_coord(apollo_x=pos[0],apollo_y=pos[1])
            pos = self.find_the_point_on_line_for_lgsvl(pos)
            pos = self.Add_heading_to_the_point(heading=start_point["heading"]["ref_angle"],point=pos)
            return pos
        elif start_point.__contains__('position'):
            pos = self.transform_apollo_coord_to_lgsvl_coord(apollo_x=start_point["position"]["x"],apollo_y=start_point["position"]["y"])
            pos = self.find_the_point_on_line_for_lgsvl(pos)
            pos = self.Add_heading_to_the_point(heading=start_point["heading"]["ref_angle"],point=pos)
            return pos
        else:
            return False

    def Add_npc_vehicles_to_Lgsvl(self, npclist):
        # layer_mask = 0
        # layer_mask |= 1 << 0
        for _i in npclist:
            npc_state = None
            npc_state = lgsvl.AgentState()
            if _i.__contains__("name"):
                name_of_npc = _i["name"]
            else:
                name_of_npc = "Sedan"

            waypoints = []
            start_point = _i["start"] 

            start_waypoint_after_process = self.Deal_with_WAYpoint(start_point)

            npc_state.transform = start_waypoint_after_process

            angle = start_waypoint_after_process.rotation
            # hit = self.sim.raycast(start_waypoint_after_process.position, lgsvl.Vector(0, -1, 0), layer_mask)
            speed = start_point["speed"]
            # if hit != None:
            wp = lgsvl.DriveWaypoint(start_waypoint_after_process.position, speed, angle, 0)
            # wp = lgsvl.DriveWaypoint(start_waypoint_after_process.position, speed)
            waypoints.append(wp)
            
            for single_waypoint in _i["motion"]:
                single_waypoint_after_process = None
                single_waypoint_after_process = self.Deal_with_WAYpoint(single_waypoint)
                angle = single_waypoint_after_process.rotation
                # hit = self.sim.raycast(single_waypoint_after_process.position, lgsvl.Vector(0, -1, 0), layer_mask)
                speed = single_waypoint["speed"]
                # if hit != None:
                wp = lgsvl.DriveWaypoint(single_waypoint_after_process.position, speed, angle, 0)
                # wp = lgsvl.DriveWaypoint(single_waypoint_after_process.position, speed)
                waypoints.append(wp)

            destination_point = _i["destination"]
            if destination_point!= None:
                destination_waypoint_after_process = self.Deal_with_WAYpoint(destination_point)
                angle = destination_waypoint_after_process.rotation
                # hit = self.sim.raycast(destination_waypoint_after_process.position, lgsvl.Vector(0, -1, 0), layer_mask)
                speed = destination_point["speed"]
                # if hit != None:
                wp = lgsvl.DriveWaypoint(destination_waypoint_after_process.position, speed, angle, 0)
                # wp = lgsvl.DriveWaypoint(destination_waypoint_after_process.position, speed)
                waypoints.append(wp)           

            color = _i["color"]

            npc = self.sim.add_agent(name=name_of_npc, agent_type=lgsvl.AgentType.NPC, state=npc_state, color=color)

            if waypoints!=[]:
                npc.follow(waypoints)
            else:
                npc.follow_closest_lane(True,5.6)

    def Add_pedestrians_to_Lgsvl(self, pedestrianList):
        for _i in pedestrianList:
            if _i.__contains__("name"):
                name_of_pedestrian = _i["name"]
            else:
                name_of_pedestrian = "Bob"

            pedestrian_state = None
            pedestrian_state = lgsvl.AgentState()
            waypoints = []
            start_point = _i["start"] 

            start_waypoint_after_process = self.Deal_with_WAYpoint(start_point)

            pedestrian_state.transform = start_waypoint_after_process
            angle = start_waypoint_after_process.rotation
            # hit = self.sim.raycast(start_waypoint_after_process.position, lgsvl.Vector(0, -1, 0), layer_mask)
            speed = start_point["speed"]
            # if hit != None:
            wp = lgsvl.DriveWaypoint(start_waypoint_after_process.position, speed, angle, 1)
            waypoints.append(wp)

            for single_waypoint in _i["motion"]:
                single_waypoint_after_process = None
                single_waypoint_after_process = self.Deal_with_WAYpoint(single_waypoint)
                angle = single_waypoint_after_process.rotation
                # hit = self.sim.raycast(single_waypoint_after_process.position, lgsvl.Vector(0, -1, 0), layer_mask)
                speed = single_waypoint["speed"]
                # if hit != None:
                wp = lgsvl.DriveWaypoint(single_waypoint_after_process.position, speed, angle, 1)
                waypoints.append(wp)


            destination_point = _i["destination"]
            if destination_point!= None:
                destination_waypoint_after_process = self.Deal_with_WAYpoint(destination_point)
                angle = destination_waypoint_after_process.rotation
                # hit = self.sim.raycast(destination_waypoint_after_process.position, lgsvl.Vector(0, -1, 0), layer_mask)
                speed = destination_point["speed"]
                # if hit != None:
                wp = lgsvl.DriveWaypoint(destination_waypoint_after_process.position, speed, angle, 1)
                waypoints.append(wp)


            pe = self.sim.add_agent(name_of_pedestrian, lgsvl.AgentType.PEDESTRIAN, pedestrian_state)
            pe.follow(waypoints)
            # print(waypoints)

    def Add_obstacles_to_Lgsvl(self, obstacleList):
        forward = lgsvl.utils.transform_to_forward(self.ego_position)
        right = lgsvl.utils.transform_to_right(self.ego_position)

        for _i in obstacleList:
            pos_forward = _i["forward"]
            pos_right = _i["right"]
            pos_rotation = _i["rotation"]
            obs_type = _i["type"]


            state = lgsvl.ObjectState()
            # state.transform.rotation = pos_rotation* self.ego_position.rotation
            state.transform.rotation = lgsvl.Vector(0, pos_rotation, 0)
            state.velocity = lgsvl.Vector(0, 0, 0)
            state.angular_velocity = lgsvl.Vector(0, 0, 0)



            pos = (
                    self.ego_position.position
                    + pos_forward * forward
                    + pos_right * right
            )

            pos0 = self.sim.map_point_on_lane(pos)
            pos.y = pos0.position.y

            state.transform.position = pos

            obs = self.sim.controllable_add(obs_type, state)

    def process_with_data_lgsvl_part(self, data_to_process:dict):
        map_of_lgsvl = data_to_process['map'] 
        time_of_day = data_to_process['time']
        weather = data_to_process['weather']

        npcList = data_to_process['npcList']
        pedestrianList = data_to_process['pedestrianList']
        obstacleList = data_to_process['obstacleList']

        try:
            self.sim = lgsvl.Simulator(self.SIMULATOR_HOST, self.SIMULATOR_PORT)
            env = Env()

            # print("connect to lgsvl now")

            

            if map_of_lgsvl == 'borregas_ave':
                if self.sim.current_scene == lgsvl.wise.DefaultAssets.map_borregasave:
                    self.sim.reset()
                else:
                    self.sim.load(lgsvl.wise.DefaultAssets.map_borregasave)
                self.map = 'borregas_ave'
            elif map_of_lgsvl == 'singlelaneroad':
                if self.sim.current_scene == lgsvl.wise.DefaultAssets.map_singlelaneroad:
                    self.sim.reset()
                else:
                    self.sim.load(lgsvl.wise.DefaultAssets.map_singlelaneroad)
                self.map = 'singlelaneroad'
            elif map_of_lgsvl == 'san_francisco':
                # if self.sim.current_scene == "12da60a7-2fc9-474d-a62a-5cc08cb97fe8":
                if self.sim.current_scene == lgsvl.wise.DefaultAssets.map_sanfrancisco:
                    self.sim.reset()
                else:
                    # print("take effect")
                    # self.sim.load("12da60a7-2fc9-474d-a62a-5cc08cb97fe8") #san_francisco_correct
                    self.sim.load(lgsvl.wise.DefaultAssets.map_sanfrancisco)
                    # self.sim.load("3a2b5432-98e7-4f97-a852-3e2c65080ba4") #san_francisco_correct2
                self.map = 'san_francisco'
            elif map_of_lgsvl == 'cubetown':
                if self.sim.current_scene == lgsvl.wise.DefaultAssets.map_cubetown:
                    self.sim.reset()
                else:
                    self.sim.load(lgsvl.wise.DefaultAssets.map_cubetown)
                self.map = 'cubetown'
            else:
                print('Map: '+ str(map_of_lgsvl) + ' not defined')



            self.add_scenario_data_to_result(data_to_process)   
            # set the day time
            time_for_lgsvl = time_of_day["hour"] + time_of_day["minute"] / 60
            self.sim.set_time_of_day(time_for_lgsvl, fixed=True)

            w = {}
            if weather.__contains__('rain'):
                w["rain"] = weather["rain"]
            else:
                w["rain"] = 0
            if weather.__contains__('fog'):
                w["fog"] = weather["fog"]
            else:
                w["fog"] = 0
            if weather.__contains__('wetness'):
                w["wetness"] = weather["wetness"]
            else:
                w["wetness"] = 0
            if weather.__contains__('sunny'):
                w["cloudiness"] = weather["sunny"]
            else:
                w["cloudiness"] = 0
            if weather.__contains__('damage'):
                w["damage"] = weather["damage"]
            else:
                w["damage"] = 0      

            self.sim.weather = WeatherState(w["rain"], w["fog"], w["wetness"],  w["cloudiness"], w["damage"])

            ego_state = lgsvl.AgentState()

            if data_to_process["ego"]["start"].__contains__('position'):
                start_point_x = data_to_process["ego"]["start"]["position"]["x"]
                start_point_y = data_to_process["ego"]["start"]["position"]["y"]
                start_point_heading = data_to_process["ego"]["start"]["heading"]["ref_angle"]

                start_point = self.transform_apollo_coord_to_lgsvl_coord(start_point_x,start_point_y)          
                start_point = self.find_the_point_on_line_for_lgsvl(start_point)
                start_point = self.Add_heading_to_the_point(heading=start_point_heading , point=start_point)
            elif data_to_process["ego"]["start"].__contains__('lane_position'):
                pos = self.process_with_lane_position(data_to_process["ego"]["start"]["lane_position"])  
                pos = self.transform_apollo_coord_to_lgsvl_coord(apollo_x=pos[0],apollo_y=pos[1])
                pos = self.find_the_point_on_line_for_lgsvl(pos)
                pos = self.Add_heading_to_the_point(heading=data_to_process["ego"]["start"]["heading"]["ref_angle"],point=pos)
                start_point = pos
            else:
                pass

            # print(start_point)

            
            self.ego_position = start_point
            ego_state.transform = start_point

            if self.ADS_model == 'apollo5':
                if data_to_process["ego"]["groundTruthPerception"]:
                    self.ego = self.sim.add_agent(env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5_full_analysis), lgsvl.AgentType.EGO, ego_state)
                else:
                    self.ego = self.sim.add_agent(env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5_full_analysis), lgsvl.AgentType.EGO, ego_state)
            elif self.ADS_model == 'apollo_latest':
                if data_to_process["ego"]["groundTruthPerception"]:
                    self.ego = self.sim.add_agent(env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5_full_analysis), lgsvl.AgentType.EGO, ego_state)
                else:
                    self.ego = self.sim.add_agent(env.str("LGSVL__VEHICLE_0", lgsvl.wise.DefaultAssets.ego_lincoln2017mkz_apollo5_full_analysis), lgsvl.AgentType.EGO, ego_state)


            self.ego.on_collision(self.AVS_on_collision)
            self.ego.connect_bridge(self.BRIDGE_HOST, self.BRIDGE_PORT)
            self.Add_npc_vehicles_to_Lgsvl(npcList)
            self.Add_pedestrians_to_Lgsvl(pedestrianList)
            self.Add_obstacles_to_Lgsvl(obstacleList)
        except:
            self.exception = True


    def process_with_data_apollo_part(self, data_to_process:dict):

        map_of_lgsvl = data_to_process['map'] 

        dv = lgsvl.dreamview.Connection(self.sim, self.ego, self.BRIDGE_HOST)
        if map_of_lgsvl== 'borregas_ave':
            dv.set_hd_map('Borregas Ave')
        elif map_of_lgsvl == 'san_francisco':
            dv.set_hd_map('San Francisco')
        elif map_of_lgsvl == 'cubetown':
            dv.set_hd_map('Cube Town')
        else:
            print('Unknown map')
        
        if self.ADS_model == 'apollo5':
            dv.set_vehicle('Lincoln2017MKZ')
        elif self.ADS_model == 'apollo_latest':
            dv.set_vehicle('Lincoln2017MKZ_LGSVL')

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


        if data_to_process["ego"]["destination"].__contains__('position'):
            destination_x = data_to_process["ego"]["destination"]["position"]["x"]
            destination_y = data_to_process["ego"]["destination"]["position"]["y"]
            coord_type = CoordType.Northing
        elif data_to_process["ego"]["destination"].__contains__('lane_position'):
            pos = self.process_with_lane_position(data_to_process["ego"]["destination"]["lane_position"])  
            destination_x = pos[0]
            destination_y = pos[1]
            coord_type = CoordType.Northing
        else:
            pass

        mode = "Mkz Lgsvl"

        dv.set_setup_mode(mode)
        dv.setup_apollo(destination_x, destination_y, modules, coord_type=coord_type)

        self.destination_apollo = [destination_x, destination_y]



        t_apollo = threading.Thread(target = self.start_bridge)
        t_lgsvl = threading.Thread(target = self.start_lgsvl)

        controllables = self.sim.get_controllables("signal")
        control_policy = "trigger=300;green=15;yellow=5;red=15;loop"
        for c in controllables:
            if c.type =="signal":
                c.control(control_policy)

        t_apollo.start()
        t_lgsvl.start()

    def convert_position_to_lane_position(self, position):
        _map = self.map
        map_info = get_map_info(_map)
        map_info.get_lane_config()
        # print(map_info)
        # lane_point = [lane_position["lane"],lane_position["offset"]]
        p = map_info.get_position2(position)
        return p

    def add_scenario_data_to_result(self, data_to_process:dict):
        self.AgentNames = data_to_process["AgentNames"]
        self.result = data_to_process
        self.result["groundTruthPerception"] = data_to_process["ego"]["groundTruthPerception"]

        if "lane_position" in self.result["ego"]["start"]:
            pass
        else:
            assert "position" in self.result["ego"]["start"]
            self.result["ego"]["start"]["lane_position"] = self.convert_position_to_lane_position(self.result["ego"]["start"]["position"])

        if "lane_position" in self.result["ego"]["destination"]:
            pass
        else:
            assert "position" in self.result["ego"]["destination"]
            self.result["ego"]["destination"]["lane_position"] = self.convert_position_to_lane_position(self.result["ego"]["destination"]["position"])

        for _i in range(len(self.result["npcList"])):
            if self.result["npcList"][_i]["start"] is not None:
                if "lane_position" in self.result["npcList"][_i]["start"]:
                    pass
                else:
                    assert "position" in self.result["npcList"][_i]["start"]
                    self.result["npcList"][_i]["start"]["lane_position"] = self.convert_position_to_lane_position(self.result["npcList"][_i]["start"]["position"])

            if self.result["npcList"][_i]["destination"] is not None:
                if "lane_position" in self.result["npcList"][_i]["destination"]:
                    pass
                else:
                    assert "position" in self.result["npcList"][_i]["destination"]
                    self.result["npcList"][_i]["destination"]["lane_position"] = self.convert_position_to_lane_position(self.result["npcList"][_i]["destination"]["position"])

            if  "motion" in self.result["npcList"][_i]:
                for _j in range(len(self.result["npcList"][_i]["motion"])):
                    if "lane_position" in self.result["npcList"][_i]["motion"][_j]:
                        pass 
                    else:
                        assert "position" in self.result["npcList"][_i]["motion"][_j]
                        self.result["npcList"][_i]["motion"][_j]["lane_position"] = self.convert_position_to_lane_position(self.result["npcList"][_i]["motion"][_j]["position"])

    def get_port(self):
        return os.getenv('WS_PORT', '8000')

    def get_host(self):
        return os.getenv('WS_HOST', 'localhost')

    def start(self):
        return websockets.serve(self.handler, self.get_host(), self.get_port() ,ping_interval=None)

    async def handler(self, websocket, path):
      async for message in websocket:
        msg = json.loads(message)
        response = {'TYPE': None, 'DATA':None }

        if msg['CMD'] == 'CMD_READY_FOR_NEW_TEST':
            response['TYPE'] = 'READY_FOR_NEW_TEST'
            response['DATA'] = True
            # print('READY_FOR_NEW_TEST')
            try:
                await websocket.send(json.dumps(response))
            except websocket.exceptions as e: #fail
                response['TYPE'] = 'ERROR'
                response['DATA'] = None
                await websocket.send(json.dumps(response))
                print('websockets.exceptions')
        elif msg['CMD'] == 'CMD_NEW_TEST':
            data_to_process = msg['DATA']
            # print('START_FOR_NEW_TEST')

            if self.result =={}:
                self.process_with_data_lgsvl_part(data_to_process) 
                if self.exception:
                    pass 
                else:        
                    self.process_with_data_apollo_part(data_to_process)
            else:
                print('The result is not cleared!')

            if self.result.__contains__('trace'):
                response['TYPE'] = 'TEST_COMPLETED'
                response['DATA'] = self.result  
                # print('Server return Trace')
                self.result = {}            
            else:
                response['TYPE'] = 'KEEP_SERVER_AND_CLIENT_ALIVE'
                response['DATA'] = None

            if self.exception:
                response['TYPE'] = 'ERROR'
                response['DATA'] = None
                self.exception = False
            try:
                await websocket.send(json.dumps(response))
            except websocket.exceptions as e: #fail
                response['TYPE'] = 'ERROR'
                response['DATA'] = None
                await websocket.send(json.dumps(response))
                print('websockets.exceptions')
        elif msg['CMD'] == 'KEEP_SERVER_AND_CLIENT_ALIVE':
            if self.result.__contains__('trace'):
                response['TYPE'] = 'TEST_COMPLETED'
                response['DATA'] = self.result  
                # print('Server return Trace') 
                self.result = {}           
            else:
                response['TYPE'] = 'KEEP_SERVER_AND_CLIENT_ALIVE'
                response['DATA'] = None
            try:
                await websocket.send(json.dumps(response))
            except websocket.exceptions as e: #fail
                response['TYPE'] = 'ERROR'
                response['DATA'] = None
                await websocket.send(json.dumps(response))
                print('websockets.exceptions')
        else:
            print('Request Invaild! Bridge Deny the request from Client !')


def main():
    ws = Server()
    loop = asyncio.get_event_loop()
    loop.run_until_complete(ws.start())
    loop.run_forever()



if __name__ == '__main__':
    main()








