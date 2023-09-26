import socket
import json
import numpy
import math 
import time

# import lgsvl
from environs import Env

from shapely.geometry import LineString, Point
from shapely.geometry import Polygon

from modules.control.proto import control_cmd_pb2
from modules.drivers.gnss.proto import imu_pb2
from modules.drivers.gnss.proto import ins_pb2
from modules.drivers.gnss.proto import gnss_best_pose_pb2

from modules.localization.proto import imu_pb2 as corrected_imu
from modules.localization.proto import gps_pb2
from modules.perception.proto import perception_obstacle_pb2
from modules.perception.proto import traffic_light_detection_pb2
from modules.planning.proto import planning_pb2

from modules.routing.proto import routing_pb2

from modules.canbus.proto import chassis_pb2
from modules.perception.proto import perception_lane_pb2
from lgsvl_pkgs.lgsvl_msgs.proto import detection2darray_pb2
from modules.drivers.proto import conti_radar_pb2
from modules.drivers.proto import pointcloud_pb2
from modules.localization.proto import pose_pb2


from map_for_bridge import get_map_info


# obstacle_type = {"UNKNOWN": 0, "UNKNOWN_MOVABLE": 1,"UNKNOWN_UNMOVABLE": 2,"PEDESTRIAN": 3,"BICYCLE": 4,"VEHICLE": 5}


class CyberBridgeInstance1:
	def __init__(self):
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.connect_status = 'disconnected'
		self.setup = []
		self.sim = None


	def connect(self):
		if self.connect_status == 'disconnected':
			HOST = '127.0.0.1'  # The server's hostname or IP address
			PORT = 9090        # The port used by the server

			# with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
				
			self.connect_status = 'connecting'
			print('connecting')

			self.sock.setsockopt(socket.SOL_TCP, socket.TCP_NODELAY, 1)

			SEND_BUF_SIZE = 1024*1024
			RECV_BUF_SIZE = 1024*1024
			self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_SNDBUF,SEND_BUF_SIZE)
			self.sock.setsockopt(socket.SOL_SOCKET,socket.SO_RCVBUF,RECV_BUF_SIZE)
			self.sock.settimeout(10)

			try:
				self.sock.connect((HOST, PORT))
				self.connect_status = 'connected'
				print('Connected to Apollo Port 9090 successfully!')
				for i in self.setup:
					self.send(i)

				while True:                  
					response = self.receive()
					if response:
						self.receive_publish(response)
					else:
						self.testFailures.append("Receive message is None")
						print('Error!')
						break
					# if self.reach_destinaton == True or self.Check_The_vehicle_is_stuck_or_not == "Stuck!":
					# 	break
			except socket.error as err:
				self.testFailures.append("Couldnt connect with the socket-server: " + str(err))
				print ("Couldnt connect with the socket-server: %s\n terminating program" % err)  
		else:
			print('The status of connection is: ' + str(connect_status))
			pass


	def disconnect(self):
		self.sock.close()
		self.connect_status = 'disconnected'


	def send(self, data):
		if self.connect_status == 'connected':
			try:
				self.sock.sendall(data)
			except socket.error as err:
				self.testFailures.append("Send Error: "+str(err))
				print ("Send Error: %s\n terminating program" % err)
				self.disconnect()
		else:
			ptint('Not Connected! Send Fail!')


	def receive(self):
		if self.connect_status == 'connected':
			try:
				response = self.sock.recv(1024*1024)
				# response = response.decode("ascii") 
				if response[0] != 4:
					# print('Receive Error:  Not from Publish')
					# self.disconnect()
					return response
				else:
					return response
			except socket.error as err:
				self.testFailures.append("Receive Error: "+str(err))
				print ("Receive Error: %s\n terminating program" % err)
				self.disconnect()
		else:
			print('Not Connected! Receive Fail!') 

	def parse_message_of_pose(self, pose):
		temp = {}

		position = {}
		position["x"] = pose.localization.position.x
		position["y"] = pose.localization.position.y
		position["z"] = pose.localization.position.z
		temp["position"] = position

		orientation = {}
		orientation["qx"] = pose.localization.orientation.qx
		orientation["qy"] = pose.localization.orientation.qy
		orientation["qz"] = pose.localization.orientation.qz
		orientation["qw"] = pose.localization.orientation.qw
		temp["orientation"] = orientation

		linear_velocity = {}
		linear_velocity["x"] = pose.localization.linear_velocity.x
		linear_velocity["y"] = pose.localization.linear_velocity.y
		linear_velocity["z"] = pose.localization.linear_velocity.z
		temp["linearVelocity"] = linear_velocity

		linear_acceleration = {}
		linear_acceleration["x"] = pose.localization.linear_acceleration.x
		linear_acceleration["y"] = pose.localization.linear_acceleration.y
		linear_acceleration["z"] = pose.localization.linear_acceleration.z
		temp["linearAcceleration"] = linear_acceleration

		angular_velocity = {}
		angular_velocity["x"] = pose.localization.angular_velocity.x
		angular_velocity["y"] = pose.localization.angular_velocity.y
		angular_velocity["z"] = pose.localization.angular_velocity.z
		temp["angularVelocity"] = angular_velocity

		heading = pose.localization.heading
		temp["heading"] = heading


		linear_acceleration_vrf = {}
		linear_acceleration_vrf["x"] = pose.localization.linear_acceleration_vrf.x
		linear_acceleration_vrf["y"] = pose.localization.linear_acceleration_vrf.y
		linear_acceleration_vrf["z"] = pose.localization.linear_acceleration_vrf.z
		temp["linearAccelerationVrf"] = linear_acceleration_vrf


		angular_velocity_vrf = {}
		angular_velocity_vrf["x"] = pose.localization.angular_velocity_vrf.x
		angular_velocity_vrf["y"] = pose.localization.angular_velocity_vrf.y
		angular_velocity_vrf["z"] = pose.localization.angular_velocity_vrf.z
		temp["angularVelocityVrf"] = angular_velocity_vrf

		euler_angles = {}
		euler_angles["x"] = pose.localization.euler_angles.x
		euler_angles["y"] = pose.localization.euler_angles.y
		euler_angles["z"] = pose.localization.euler_angles.z
		temp["eulerAngles"] = euler_angles

		self.Ego = {}
		self.Ego["pose"] = temp
		self.Ego["size"] = { "length": 4.7, "width": 2.06 }

	# we should know which lane the obstacle is in and classify it.
	def classify_oblist(self, oblist):
		assert self.Ego != {}
		x = self.Ego["pose"]["position"]["x"]
		y = self.Ego["pose"]["position"]["y"]
		point = (x,y)
		the_result_after_classification = dict()

		result = self.map_info.find_which_area_the_point_is_in(point)
		if result != None:
			if result[0].__contains__("lane_id"):
				ego_lane_id = result[0]["lane_id"]
			else:
				ego_lane_id = result[0]["junction_id"]
		else:
			ego_lane_id = None

		same_list = []
		different_list = []
		junction_list = []

		fourth_list = []
		fivth_list = []
		unknown_list = []
		for ob in oblist:
			temp = dict()
			x = ob["position"]["x"]
			y = ob["position"]["y"]
			point = (x,y)
			result = self.map_info.find_which_area_the_point_is_in(point)
			if result != None:
				if result[0].__contains__("lane_id"):
					oblist_lane_id = result[0]["lane_id"]
				else:
					oblist_lane_id = result[0]["junction_id"]
			else:
				oblist_lane_id = None
			if ego_lane_id != None and oblist_lane_id!= None:
				if "lane" in ego_lane_id:
					# print('???')
					if "lane" in oblist_lane_id:
						if self.map_info.check_whether_two_lanes_are_in_the_same_road(ego_lane_id, oblist_lane_id):
							# print("ego and "+ob["name"] +" on the same Road")
							temp["name"] = ob["name"]
							temp["laneId"] = oblist_lane_id
							temp["turn"] = result[0]["turn"]
							same_list.append(temp)
						else:
							# print("ego and "+ob["name"] +" on the different Road")
							temp["name"] = ob["name"]
							temp["laneId"] = oblist_lane_id
							temp["turn"] = result[0]["turn"]
							different_list.append(temp)
					elif "J_" in oblist_lane_id or "junction" in oblist_lane_id:
						# print("ego on lane and "+ob["name"] +" on the junction")
						temp["name"] = ob["name"]
						temp["junctionId"] = oblist_lane_id
						junction_list.append(temp)
				elif "J_" in ego_lane_id or "junction" in oblist_lane_id:
					# print('!!!!')
					if "lane" in oblist_lane_id:
						temp["name"] = ob["name"]
						temp["laneId"] = oblist_lane_id
						temp["turn"] = result[0]["turn"]
						fourth_list.append(temp)
					elif "J_" in ego_lane_id or "junction" in oblist_lane_id:
						temp["name"] = ob["name"]
						temp["junctionId"] = oblist_lane_id
						fivth_list.append(temp)
			else:
				temp["name"] = ob["name"]
				unknown_list.append(temp)

		#When ego on lane
		the_result_after_classification["NextToEgo"] = same_list
		the_result_after_classification["OntheDifferentRoad"] = different_list
		the_result_after_classification["IntheJunction"] = junction_list

		#when ego on junction
		the_result_after_classification["EgoInjunction_Lane"] = fourth_list
		the_result_after_classification["EgoInjunction_junction"] = fivth_list


		return the_result_after_classification
		# print(oblist_lane_position)

	def convert_velocity_to_speed(self, velocity):
		x = velocity["x"]
		y = velocity["y"]
		z = velocity["z"]

		return math.sqrt(x*x+y*y+z*z)

	def process_with_angle_pi(self, angle_pi):
		if angle_pi<0:
			return angle_pi + 2*math.pi
		else:
			return angle_pi

# enum Type {
#   UNKNOWN = 0;
#   UNKNOWN_MOVABLE = 1;
#   UNKNOWN_UNMOVABLE = 2;
#   PEDESTRIAN = 3;  // Pedestrian, usually determined by moving behavior.
#   BICYCLE = 4;     // bike, motor bike
#   VEHICLE = 5;     // Passenger car or truck.
# };
	def parse_message_of_obstacles(self, obstacles):
		oblist = []
		if True:
			# print("********")
			for _i in obstacles.perception_obstacle:
				print("The obstacle: " + str(_i.id) + "  " + str(_i.type))
				oblist.append({})
				if hasattr(_i,'id'):
					theta = _i.id
					oblist[-1]["id"] = theta

				if hasattr(_i,'position'):
					position = {}
					position["x"] = _i.position.x
					position["y"] = _i.position.y
					position["z"] = _i.position.z
					oblist[-1]["position"] = position

				if hasattr(_i,'theta'):
					theta = _i.theta
					oblist[-1]["theta"] = theta

				if hasattr(_i,'velocity'):
					velocity = {}
					velocity["x"] = _i.velocity.x
					velocity["y"] = _i.velocity.y
					velocity["z"] = _i.velocity.z
					oblist[-1]["velocity"] = velocity

				speed = self.convert_velocity_to_speed(velocity)
				oblist[-1]["speed"] = speed

				if hasattr(_i,'length'):
					length = _i.length
					oblist[-1]["length"] = length   

				if hasattr(_i,'width'):
					width = _i.width
					oblist[-1]["width"] = width

				if hasattr(_i,'height'):
					height = _i.height
					oblist[-1]["height"] = height    

				if hasattr(_i,"polygon_point"):
					temp0 = []        
					for _j in _i.polygon_point:
						temp = {}
						temp["x"] = _j.x
						temp["y"] = _j.y
						temp["z"] = _j.z
						temp0.append(temp)
					oblist[-1]["polygonPointList"] = temp0   

				if hasattr(_i,"tracking_time"):
					trackingTime = _i.tracking_time
					oblist[-1]["trackingTime"] = trackingTime  

				if hasattr(_i,"type"):
					type0 = _i.type
					oblist[-1]["type"] = type0

				if hasattr(_i,"timestamp"):
					timestamp = _i.timestamp
					oblist[-1]["timestamp"] = timestamp

				if hasattr(_i,"pointCloudList"):
					temp0 = []
					temp = {}
					for _j in _i.pointCloudList:
						temp["x"] = _j.x
						temp["y"] = _j.y
						temp["z"] = _j.z
						temp0.append(temp)
					oblist[-1]["pointCloudList"] = temp0  
				else:
					oblist[-1]["pointCloudList"] = []   

				if hasattr(_i,"dropsList"):
					temp0 = []
					temp = {}
					for _j in _i.dropsList:
						temp["x"] = _j.x
						temp["y"] = _j.y
						temp["z"] = _j.z
						temp0.append(temp)
					oblist[-1]["dropsList"] = temp0  
				else:
					oblist[-1]["dropsList"] = [] 

				if hasattr(_i,"acceleration"):
					acceleration = {}
					acceleration["x"] = _i.acceleration.x
					acceleration["y"] = _i.acceleration.y
					acceleration["z"] = _i.acceleration.z
					oblist[-1]["acceleration"] = acceleration   

				if hasattr(_i,"anchor_point"):
					anchorPoint = {}
					anchorPoint["x"] = _i.anchor_point.x
					anchorPoint["y"] = _i.anchor_point.y
					anchorPoint["z"] = _i.anchor_point.z
					oblist[-1]["anchorPoint"] = anchorPoint

				if hasattr(_i,"bbox2d"):
					bbox2d = {}
					bbox2d["xmin"] = _i.bbox2d.xmin
					bbox2d["ymin"] = _i.bbox2d.ymin
					bbox2d["xmax"] = _i.bbox2d.xmax
					bbox2d["ymax"] = _i.bbox2d.ymax
					oblist[-1]["bbox2d"] = bbox2d

				if hasattr(_i,"sub_type"):
					subType = _i.sub_type
					oblist[-1]["subType"] = subType

				if hasattr(_i,"measurements"):
					temp0 = []
					for _j in _i.measurements:
						temp = {}
						temp["sensorId"] = _j.sensor_id
						temp["id"] = _j.id

						temp1 = {}
						temp1["x"] = _j.position.x
						temp1["y"] = _j.position.y
						temp1["z"] = _j.position.z

						temp["position"] = temp1
						temp["theta"] = _j.theta
						temp["length"] = _j.length
						temp["width"] = _j.width
						temp["height"] = _j.height

						temp1 = {}
						temp1["x"] = _j.velocity.x
						temp1["y"] = _j.velocity.y
						temp1["z"] = _j.velocity.z
						temp["velocity"] = temp1

						temp["type"] = _j.type

						if hasattr(_j,"sub_type"):
							subType = _j.sub_type
							temp["subType"] = subType

						temp["timestamp"] = _j.timestamp

						temp0.append(temp)
					oblist[-1]["measurementsList"] = temp0  

				if hasattr(_i,"height_above_ground"):
					height_above_ground = _i.height_above_ground
					oblist[-1]["heightAboveGround"] = height_above_ground


				if hasattr(_i,"position_covariance"):
					temp0 = []
					for _j in _i.position_covariance:
						temp0.append(_j)
					oblist[-1]["positionCovarianceList"] = temp0

				if hasattr(_i,"velocity_covariance"):
					temp0 = []
					for _j in _i.velocity_covariance:
						temp0.append(_j)
					oblist[-1]["velocityCovarianceList"] = temp0

				if hasattr(_i,"acceleration_covariance"):
					temp0 = []
					for _j in _i.acceleration_covariance:
						temp0.append(_j)
					oblist[-1]["accelerationCovarianceList"] = temp0

				if hasattr(_i,"type_name"):
					type_name = _i.type_name
					oblist[-1]["typeName"] = type_name   

				if hasattr(_i,"sub_type_name"):
					sub_type_name = _i.sub_type_name
					oblist[-1]["subTypeName"] = sub_type_name 

				# if hasattr(_i,"name"):
				#     name = _i.name
				#     oblist[-1]["name"] = name 
				# else:
				# num = oblist[-1]["id"]
				# oblist[-1]["name"] = self.AgentNames[num-2]

				# print(str(self.AgentNames[num-2])+ "  " +str(theta))

				# if self.Ego != {}:
				# 	distToEgo = self.calculate_distToEgo(oblist[-1])
				# 	oblist[-1]["distToEgo"] = distToEgo		

	def parse_message_of_traffic_light(self, TrafficLight):
		if self.Ego!={}:
			result = {}
			if hasattr(TrafficLight, "header"):
				pass
				# print("TrafficLight" + str(TrafficLight.header.sequence_num))

			if hasattr(TrafficLight, "contain_lights"):
				result["containLights"] = TrafficLight.contain_lights
				if TrafficLight.contain_lights:
					if hasattr(TrafficLight, "traffic_light"):
						traffic_light = []
						for _i in TrafficLight.traffic_light:
							temp = {}
							if hasattr(_i,"color"):
								temp["color"] = _i.color
							if hasattr(_i,"id"):
								temp["id"] = _i.id
							# if hasattr(_i,"confidence"):
							#     temp["confidence"] = _i.confidence
							# if hasattr(_i,"tracking_time"):
							#     temp["tracking_time"] = _i.tracking_time
							if hasattr(_i,"blink"):
								temp["blink"] = _i.blink
							# if hasattr(_i,"remaining_time"):
							#     temp["remaining_time"] = _i.remaining_time
							traffic_light.append(temp)
						result["trafficLightList"] = traffic_light                   
						result["trafficLightStopLine"] = self.calculate_distance_to_traffic_light_stop_line(temp["id"])
					else:
						print("No information for traffic lights found!")

			self.traffic_lights = result
			# print(self.traffic_lights)
		else:
			pass

	def parse_message_of_canbus(self, chassis):
		result = {}
		
		result["lowBeamOn"] = chassis.signal.low_beam
		result["highBeamOn"] = chassis.signal.high_beam
		result["turnSignal"] = chassis.signal.turn_signal

		# print("turn_signal:"+ str(result["turnSignal"]))

		result["hornOn"] = chassis.signal.horn
		result["speed"] = chassis.speed_mps

		result["engineOn"] = chassis.engine_started
		result["gear"] = chassis.gear_location
		result["brake"] = chassis.brake_percentage

		result["day"] = chassis.chassis_gps.day
		result["hours"] = chassis.chassis_gps.hours
		result["minutes"] = chassis.chassis_gps.minutes
		result["seconds"] = chassis.chassis_gps.seconds

		result["error_code"] = chassis.error_code
		self.Canbus = result




	def Get32le(self, offset, response):
		ret = response[offset + 0] | response[offset + 1]<<8 | response[offset + 2]<<16 | response[offset + 3]<<24
		return ret

	def receive_publish(self, response:bytes):
		if len(response) < 1+2*4:
			return False

		offset = 1
		channelsize = self.Get32le(offset,response)
		offset = offset + 4
		if len(response) < offset + channelsize:
			return False

		channeloffset = offset
		offset = channelsize + offset

		if len(response) < offset + 4:
			return False

		messagesize = self.Get32le(offset,response)
		offset = offset + 4

		if len(response) < offset + messagesize:
			return False

		messageoffset = offset
		offset = messagesize + offset

		legal_flag = True
		for _iii in response[channeloffset:channeloffset+channelsize]:
			if _iii >= 128:
				legal_flag = False 

		if legal_flag!= False:
			channel = response[channeloffset:channeloffset+channelsize].decode("ascii") 
		else:
			return False 

		message = response[messageoffset:messageoffset+messagesize]


		if channel == '/apollo/perception/obstacles':
			PerceptionObstacles = perception_obstacle_pb2.PerceptionObstacles()
			PerceptionObstacles.ParseFromString(message)
			self.parse_message_of_obstacles(PerceptionObstacles)
		elif channel == '/apollo/canbus/chassis':
			chassis = chassis_pb2.Chassis()
			chassis.ParseFromString(message)
			self.parse_message_of_canbus(chassis)
		elif channel == '/simulator/ground_truth/2d_detections':
			Detection2DArray = detection2darray_pb2.Detection2DArray()
			Detection2DArray.ParseFromString(message)
			print('Detection2DArray')
			print(Detection2DArray)       
		elif channel == 'Lane_lane':
			PerceptionLanes = perception_lane_pb2.PerceptionLanes() 
			PerceptionLanes.ParseFromString(message)
			print('PerceptionLanes') 
			# print(PerceptionLanes) 
		elif channel == '/apollo/sensor/conti_radar':
			ContiRadar = conti_radar_pb2.ContiRadar()
			ContiRadar.ParseFromString(message)
			print('ContiRadar') 
			print(ContiRadar)
		elif channel == '/apollo/sensor/lidar128/compensator/PointCloud2':
			PointCloud = pointcloud_pb2.PointCloud() 
			PointCloud.ParseFromString(message)
			print('PointCloud') 
			print(PointCloud)
		else:
			pass
		
	def AddSubscriber(self, BridgeType, Topic: str):
		channelBytes = Topic.encode('ascii')
		typeBytes = (str(BridgeType)).encode('ascii')
		bytes_for_set = []
		bytes_for_set.append('2'.encode('ascii'))
		bytes_for_set.append(str(len(channelBytes)>>0).encode('ascii'))
		bytes_for_set.append(str(len(channelBytes)>>8).encode('ascii'))
		bytes_for_set.append(str(len(channelBytes)>>16).encode('ascii'))
		bytes_for_set.append(str(len(channelBytes)>>24).encode('ascii'))
		for i in channelBytes:
			bytes_for_set.append(str(i).encode('ascii'))
		bytes_for_set.append(str(len(typeBytes)>>0).encode('ascii'))
		bytes_for_set.append(str(len(typeBytes)>>8).encode('ascii'))
		bytes_for_set.append(str(len(typeBytes)>>16).encode('ascii'))
		bytes_for_set.append(str(len(typeBytes)>>24).encode('ascii'))
		for i in typeBytes:
			bytes_for_set.append(str(i).encode('ascii'))
			# bytes_for_set.append(i)

		# print(len(bytes([len(channelBytes)>>0])))

		data = bytes([2]) + \
				bytes([len(channelBytes)>>0]) +\
				bytes([len(channelBytes)>>8]) + \
				bytes([len(channelBytes)>>16]) + \
				bytes([len(channelBytes)>>24]) + \
				channelBytes + \
				bytes([len(typeBytes)>>0]) +\
				bytes([len(typeBytes)>>8]) + \
				bytes([len(typeBytes)>>16]) + \
				bytes([len(typeBytes)>>24]) + \
				typeBytes


		self.setup.append(data)                


	def register(self):
		# The control -useful!
		# self.AddSubscriber("apollo.control.ControlCommand","/apollo/control")

		# Canbus -useful!
		self.AddSubscriber("apollo.canbus.Chassis","/apollo/canbus/chassis")


		#The PerceptionObstacles -Key!
		self.AddSubscriber("apollo.perception.PerceptionObstacles","/apollo/perception/obstacles")

		#The Traffic lights -Key!
		# self.AddSubscriber("apollo.perception.TrafficLightDetection","/apollo/perception/traffic_light")
		
		# self.AddSubscriber("apollo.perception.TrafficLightDetection","/simulator/ground_truth/signals")

		#Lane -Key!
		# self.AddSubscriber("apollo.perception.PerceptionLanes","Lane_lane")    




		# self.connect()

		return



def main():
	print("Init...")
	bridge_custom = CyberBridgeInstance1()
	bridge_custom.register()
	bridge_custom.connect()


if __name__ == '__main__':
    main()