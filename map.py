import copy
import json
import warnings

import numpy as np
from shapely.geometry import Point, LineString
from shapely.geometry import Polygon

# from CusLgsvl import CUS_LGVSL
import os
import lgsvl
from environs import Env

# test for drawing
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.spatial import ConvexHull


directory = 'map/'

_type_of_signals = {"1":"UNKNOWN", \
					"2":"CIRCLE", \
					"3":"ARROW_LEFT", \
					"4":"ARROW_FORWARD", \
					"5":"ARROW_RIGHT", \
					"6":"ARROW_LEFT_AND_FORWARD", \
					"7":"ARROW_RIGHT_AND_FORWARD", \
					"8":"ARROW_U_TURN", \
					 }




# def draw_polygon(vertices):
#     # Ensure the polygon is closed by appending the first vertex to the end
#     vertices.append(vertices[0])

#     # Extract x and y coordinates from the vertices
#     x_coords, y_coords = zip(*vertices)

#     # Create a figure and axis
#     fig, ax = plt.subplots()

#     # Create a Polygon patch and add it to the plot
#     polygon = patches.Polygon(vertices, closed=True, edgecolor='black', facecolor='blue', alpha=0.6)
#     ax.add_patch(polygon)

#     # Set axis labels (optional)
#     ax.set_xlabel('X-axis')
#     ax.set_ylabel('Y-axis')

#     # Set a title for the plot (optional)
#     ax.set_title('Filled Polygon Drawing')

#     # Show the plot
#     plt.show()

def draw_polygon(vertices):
    # Ensure the polygon is closed by appending the first vertex to the end
    vertices.append(vertices[0])

    # Extract x and y coordinates from the vertices
    x_coords, y_coords = zip(*vertices)

    # Create a figure and axis
    fig, ax = plt.subplots()

    # Plot the polygon
    ax.plot(x_coords, y_coords, marker='o')

    # Set axis labels (optional)
    ax.set_xlabel('X-axis')
    ax.set_ylabel('Y-axis')

    # Set a title for the plot (optional)
    ax.set_title('Polygon Drawing')

    # Show the plot
    plt.show()


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

class get_map_info:
	def __init__(self, map_name):
		self.file = directory + map_name + ".json"
		self.lane_config = dict()
		self.lane_waypoints = dict()
		self.crosswalk_config = dict()
		self.lane_predecessor = dict()
		self.lane_successor = dict()

		self.lane_turn =dict()

		self.areas = dict()

		self.crosswalk_config = dict()
		self.traffic_sign = []
		self.traffic_signals = []
		self.Roads = []
		self.original_map = dict()
		self.map_name = map_name


		with open(self.file) as f:
			self.areas["lane_areas"] = dict()
			self.areas["junction_areas"] = dict()
			self.areas["crosswalk_areas"] = dict()
			self.areas["road_areas"] = dict()

			map_config = json.load(f)
			self.original_map = map_config
			lane = map_config['laneList']
			junction = map_config['junctionList']
			crosswalk = map_config['crosswalkList']
			trafficSign = map_config['stopSignList']
			Signals = map_config['signalList']
			Roads = map_config['roadList']
			for i in range(len(lane)):
				lane_id = lane[i]['id']['id']
				lane_length = lane[i]['length']
				self.lane_config[lane_id] = lane_length
				self.lane_waypoints[lane_id] = []
				for _i in range(len(lane[i]['centralCurve']['segmentList'])):
					lane_segment_point = lane[i]['centralCurve']['segmentList'][_i]['lineSegment']['pointList']
					for k in range(len(lane_segment_point)):
						_wp_k = lane_segment_point[k]
						self.lane_waypoints[lane_id].append(np.array([_wp_k['x'], _wp_k['y']]))
				predecessor = lane[i]['predecessorIdList']
				self.lane_predecessor[lane_id] = []
				for j in range(len(predecessor)):
					self.lane_predecessor[lane_id].append(predecessor[j]['id'])
				successor = lane[i]['successorIdList']
				self.lane_successor[lane_id] = []
				for k in range(len(successor)):
					self.lane_successor[lane_id].append(successor[k]['id'])


				area_lane_left = []
				for _ii in range(len(lane[i]['leftBoundary']['curve']['segmentList'])):
					leftBoundary_point = lane[i]['leftBoundary']['curve']['segmentList'][_ii]['lineSegment']['pointList']
					for k in range(len(leftBoundary_point)):
						_wp_k0 = leftBoundary_point[k]
						area_lane_left.append((_wp_k0['x'], _wp_k0['y']))

				area_lane_right = []
				for _iii in range(len(lane[i]['rightBoundary']['curve']['segmentList'])):
					rightBoundary_point = lane[i]['rightBoundary']['curve']['segmentList'][_iii]['lineSegment']['pointList']
					for k in range(len(rightBoundary_point)):
						_wp_k1 = rightBoundary_point[k]
						area_lane_right.append((_wp_k1['x'], _wp_k1['y']))

				area_lane_right.reverse()
				single_lane_area = []
				single_lane_area = area_lane_left + area_lane_right
				# print(single_lane_area)
				self.areas["lane_areas"][lane_id] = single_lane_area

			for _junction in junction:
				junction_id = _junction['id']['id']
				temp = []
				junction_polygon = _junction['polygon']['pointList']
				for _point in junction_polygon:
					temp.append((_point['x'],_point['y']))
				self.areas["junction_areas"][junction_id] = temp

			for _crosswalk in crosswalk:
				crosswalk_id = _crosswalk['id']['id']
				crosswalk_polygon = _crosswalk['polygon']['pointList']
				if len(crosswalk_polygon) != 4:
					print('Needs four points to describe a crosswalk!')
					exit()
				crosswalk_points = [(crosswalk_polygon[0]['x'], crosswalk_polygon[0]['y']),
									(crosswalk_polygon[1]['x'], crosswalk_polygon[1]['y']),
									(crosswalk_polygon[2]['x'], crosswalk_polygon[2]['y']),
									(crosswalk_polygon[3]['x'], crosswalk_polygon[3]['y'])
									]
				self.areas["crosswalk_areas"][crosswalk_id] = crosswalk_points

			# for _i in trafficSign:
			# 	single_element = {}
			# 	single_element["id"] = _i["id"]["id"]
			# 	if single_element["id"].find("stopsign") != -1:
			# 		single_element["type"] = "stopsign"
			# 		stop_line_points = []
			# 		if _i.__contains__("stopLineList"):                        
			# 			stop_line_points = _i["stopLineList"][0]["segmentList"][0]["lineSegment"]["pointList"]
			# 		single_element["stop_line_points"] = stop_line_points

			# 	else:
			# 		single_element["type"] = None
			# 	self.traffic_sign.append(single_element)

			# for _i in Signals:
			# 	single_element = {}
			# 	single_element["id"] = _i["id"]["id"]
			# 	# if single_element["id"].find("stopsign") != -1:
			# 	#     single_element["type"] = "stopsign"
			# 	sub_signal_type_list = []
			# 	if _i.__contains__("subsignalList"):                        
			# 		for _j in _i["subsignalList"]:
			# 			num_type = _j["type"]
			# 			sub_signal_type_list.append(_type_of_signals[str(num_type)])
			# 	single_element["sub_signal_type_list"] = sub_signal_type_list
			# 	if _i.__contains__("stopLineList"):                        
			# 		stop_line_points = _i["stopLineList"][0]["segmentList"][0]["lineSegment"]["pointList"]
			# 	single_element["stop_line_points"] = stop_line_points
			# 	self.traffic_signals.append(single_element)
				
			for _road in Roads:
				road_id = _road["id"]["id"]
				temp = []
				road_polygon = _road["sectionList"][0]['boundary']['outerPolygon']['edgeList']
				for _points in road_polygon:
					for _point in _points['curve']['segmentList'][0]['lineSegment']['pointList']:
						temp.append((_point['x'],_point['y']))
				self.areas["road_areas"][road_id] = temp



	def get_lane_config(self):
		return self.lane_config

	def get_successor_lanes(self, lane_name):
		return self.lane_successor[lane_name]

	def get_predecessor_lanes(self, lane_name):
		return self.lane_predecessor[lane_name]

	def get_crosswalk_config(self):
		return self.crosswalk_config

	def get_position(self, lane_position): #convert lane_position to normal one
		## lane_position = [lane_id, offset]
		lane_id = lane_position[0]
		offset = lane_position[1]
		waypoint = self.lane_waypoints[lane_id]
		_distance = 0
		for i in range(len(waypoint)-1):
			wp1 = waypoint[i]
			wp2 = waypoint[i+1]
			_dis_wp1_2 = np.linalg.norm(wp1 - wp2)
			if _distance + _dis_wp1_2 > offset:
				current_dis = offset - _distance
				k = current_dis / _dis_wp1_2
				x = wp1[0] + (wp2[0] - wp1[0])*k
				y = wp1[1] + (wp2[1] - wp1[1])*k
				return (x, y, 0)
			_distance += _dis_wp1_2
		if i == len(waypoint)-2:
			warnings.warn("The predefined position is out of the given lane, set to the end of the lane.")
			return (wp2[0], wp2[1], 0)


	def get_position2(self, position): #convert normal one to lane_position
		position2 = np.array([position['x'], position['y']])

		point = Point(position2)
		temp = 100000
		result = {}
		for key in self.areas["lane_areas"]:
			points = []
			points = self.areas["lane_areas"][key]
			the_area = Polygon(points)                 
			if point.distance(the_area) < temp:
				temp = point.distance(the_area)
				result["lane"] = key
				waypoints = self.lane_waypoints[key]
				dsiatance = []
				dis0 = np.linalg.norm(position2 - waypoints[0])
				nearest_point = 0
				for _i in range(len(waypoints)):
					if np.linalg.norm(position2 - waypoints[_i]) < dis0:
						nearest_point = _i
						dis0 = np.linalg.norm(position2 - waypoints[_i])
				offset = 0

				tt = nearest_point

				if tt > 0:
					offset += np.linalg.norm(waypoints[tt] - waypoints[tt-1])
					tt -= 1

				result["offset"] = offset + np.linalg.norm(position2 - waypoints[nearest_point])
		return result

	def position2lane(self, position):
		_dis = float('inf')
		_point = Point(position)
		for item in self.lane_waypoints.keys():
			_line = LineString(self.lane_waypoints[item])
			_current_dis = _point.distance(_line)
			if _current_dis < _dis:
				_dis = _current_dis
				_lane_name = item
		return _lane_name


	def get_global_position(self, position, local_position):
		_point = Point(position)
		# for item in self.lane_waypoints.keys():
		#     _line = LineString(self.lane_waypoints[item])
		#     _current_dis = _point.distance(_line)
		#     if _current_dis < _dis:
		#         _dis = _current_dis
		#         _lane_name = item
		_lane_name = self.position2lane(position)
		_lane_waypoint = self.lane_waypoints[_lane_name]
		_in_lane_dis = float('inf')
		_waypoint_index = 0
		for i in range(len(_lane_waypoint)-1):
			_segment_line = LineString([_lane_waypoint[i], _lane_waypoint[i+1]])
			_current_dis = _point.distance(_segment_line)
			if _current_dis < _in_lane_dis:
				_in_lane_dis = _current_dis
				_waypoint_index = i

		direction = _lane_waypoint[_waypoint_index+1] - _lane_waypoint[_waypoint_index]
		cos_theta = direction[0] / np.linalg.norm(direction)
		sin_theta = direction[1] / np.linalg.norm(direction)
		x1 = local_position[0] * cos_theta - local_position[1] * sin_theta
		y1 = local_position[0] * sin_theta + local_position[1] * cos_theta
		return (x1 + position[0], y1 + position[1])

	def check_whether_in_road_area(self, x, y):
		point = Point(x,y)
		for key in self.areas["lane_areas"]:
			points = []
			points = self.areas["lane_areas"][key]
			the_area = Polygon(points)       
			if the_area.contains(point):
				return True

		for key in self.areas["junction_areas"]:
			points = []
			points = self.areas["junction_areas"][key]
			the_area = Polygon(points)       
			if the_area.contains(point):    
				return True

		for key in self.areas["crosswalk_areas"]:
			points = []
			points = self.areas["crosswalk_areas"][key]
			the_area = Polygon(points)       
			if the_area.contains(point):    
				return True

		return False

	def check_whether_in_lane_area(self, x, y):
		point = Point(x,y)
		for key in self.areas["lane_areas"]:
			points = []
			points = self.areas["lane_areas"][key]
			the_area = Polygon(points)       
			if the_area.contains(point):
				return True
		return False

	def dist_to_lanes(self, x, y, length):
		point = Point(x,y)
		for key in self.areas["lane_areas"]:
			points = []
			points = self.areas["lane_areas"][key]
			the_area = Polygon(points)      
			if point.distance(the_area) <= length:
				return True
		return False

	def check_whether_in_junction_area(self, x, y):
		point = Point(x,y)
		for key in self.areas["junction_areas"]:
			points = []
			points = self.areas["junction_areas"][key]
			the_area = Polygon(points)       
			if the_area.contains(point):    
				return True
		return False

	def dist_to_junctions(self, x, y, length):
		point = Point(x,y)
		for key in self.areas["junction_areas"]:
			points = []
			points = self.areas["junction_areas"][key]
			the_area = Polygon(points)             
			if point.distance(the_area) <= length:
				return True
		return False

	def dist_to_roads(self, x, y):
		length = 1000
		point = Point(x,y)
		for key in self.areas["junction_areas"]:
			points = []
			points = self.areas["junction_areas"][key]
			the_area = Polygon(points)             
			if point.distance(the_area) <= length:
				length = point.distance(the_area)

		for key in self.areas["lane_areas"]:
			points = []
			points = self.areas["lane_areas"][key]
			the_area = Polygon(points)      
			if point.distance(the_area) <= length:
				length = point.distance(the_area)
		return length


	def lgsvl2apollo(self, sim, vector):
		transform0 = lgsvl.Transform(
			vector, lgsvl.Vector(0, 0, 0)
		)
		gps = sim.map_to_gps(transform0)
		dest_x = gps.easting
		dest_y = gps.northing
		return (dest_x, dest_y)


	def relative2position(self, start_x, start_y, forward, right):
		env = Env()
		sim = lgsvl.Simulator(
			env.str("LGSVL__SIMULATOR_HOST", "169.254.42.175"),
			env.int("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port)
		)

		map_of_lgsvl = self.map_name

		if map_of_lgsvl == 'borregas_ave':
			if sim.current_scene == lgsvl.wise.DefaultAssets.map_borregasave:
				pass
			else:
				sim.load(lgsvl.wise.DefaultAssets.map_borregasave)
		elif map_of_lgsvl == 'singlelaneroad':
			if sim.current_scene == lgsvl.wise.DefaultAssets.map_singlelaneroad:
				pass
			else:
				sim.load(lgsvl.wise.DefaultAssets.map_singlelaneroad)
		elif map_of_lgsvl == 'san_francisco':
			# if self.sim.current_scene == "12da60a7-2fc9-474d-a62a-5cc08cb97fe8":
			if sim.current_scene == lgsvl.wise.DefaultAssets.map_sanfrancisco:
				pass
			else:
				sim.load(lgsvl.wise.DefaultAssets.map_sanfrancisco)
		elif map_of_lgsvl == 'cubetown':
			if sim.current_scene == lgsvl.wise.DefaultAssets.map_cubetown:
				pass
			else:
				sim.load(lgsvl.wise.DefaultAssets.map_cubetown)
		else:
			print('Map: '+ str(map_of_lgsvl) + ' not defined')

		xxx = CUS_LGVSL(sim)
		ego_position = xxx.get_lgsvl_state(start_x, start_y)
		# state.transform = ego_position

		forward0 = lgsvl.utils.transform_to_forward(ego_position)
		right0 = lgsvl.utils.transform_to_right(ego_position)

		start2 = (
			ego_position.position
			+ forward * forward0
			+ right * right0
		)

		
		result = self.lgsvl2apollo(sim, start2)

		sim.close()

		return result

	def check_whether_in_attack_area(self, position, type0):
		# If the obstacle is a trash bin/movable object
		if type0 == "Bin" or type0 == "BinGreen" or type0 == "BinRed" or type0 =="BinYellow" or type0 == "BigTrashBin":
			if self.check_whether_in_road_area(position[0], position[1]) == False:
				return True
			# if self.dist_to_roads(position[0], position[1]) > 0.3:
				# if self.dist_to_lanes(position[0], position[1], 5) and self.dist_to_junctions(position[0], position[1], 5):
				# 	return True 
				# else:
				# 	return False
			else:
				return False
		# If the obstacle is a bench/fixed-pos object
		else:
			# The dist to road should be larger than 0.6
			if self.dist_to_lanes(position[0], position[1], 0.6) or self.dist_to_junctions(position[0], position[1], 0.6):
				return False 
			else:
				return True

		if self.check_whether_in_road_area(position[0], position[1]) == False:
			return True 
		else:
			return False





if __name__ == "__main__":
	# map = "san_francisco"
	# map_info = get_map_info(map)
	# map_info.get_lane_config()
	# lane_point = ["lane_231", 100]
	# p = map_info.get_position(lane_point)
	# print(p)


	map = "borregas_ave"
	map_info = get_map_info(map)

	k = map_info.areas["road_areas"]
	# k = map_info.areas["lane_areas"]
	# k = map_info.areas["junction_areas"]

	# print(k)

	coord = Polygon(k["road_0"])


	draw_polygon(k["road_0"])

