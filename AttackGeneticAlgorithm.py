import gc
import math

import numpy as np
import json
import copy

import numpy.random


from EXtraction import ExtractAll
from monitor import Monitor
from TestCaseRandom import TestCaseRandom
from TracePreprocess import Trace
import random
import ast
from itertools import chain, islice
from random import gauss
import operator
from map import get_map_info
from pedestrian_motion_checking import pedestrian_in_crosswalk
from config import get_obs_list

obs_list = get_obs_list()


forward_max = 250
forward_min = 100

right_max = 10
right_min = -10

rotation_max = 360
rotation_min = 0


def get_testcase(trace_scneario):
	testcase_ = {}
	testcase_['ScenarioName'] = trace_scneario['ScenarioName']
	testcase_['MapVariable'] = trace_scneario['MapVariable']
	testcase_['map'] = trace_scneario['map']
	testcase_['time'] = trace_scneario['time']
	testcase_['weather'] = trace_scneario['weather']
	testcase_['ego'] = trace_scneario['ego']
	testcase_['npcList'] = trace_scneario['npcList']
	testcase_['pedestrianList'] = trace_scneario['pedestrianList']
	testcase_['obstacleList'] = trace_scneario['obstacleList']
	testcase_['AgentNames'] = trace_scneario['AgentNames']
	# remove the vector positions in ego's start and destination states
	# testcase_['ego']['start'].pop('position', None)
	# testcase_['ego']['start']['heading'].pop('ref_point', None)
	# testcase_['ego']['destination'].pop('position', None)
	# testcase_['ego']['destination']['heading'].pop('ref_point', None)

	map_info = get_map_info(trace_scneario['map'])
	lane_config = map_info.get_lane_config()
	crosswalk_config = map_info.get_crosswalk_config()

	for _i in range(len(testcase_['npcList'])):
		testcase_['npcList'][_i]['start'].pop('position', None)
		testcase_['npcList'][_i]['start']['heading'].pop('ref_point', None)
		_offset_s = testcase_['npcList'][_i]['start']['lane_position']['offset']
		_lane_name_s = testcase_['npcList'][_i]['start']['lane_position']['lane']
		testcase_['npcList'][_i]['start']['lane_position']['offset'] = float(np.clip(_offset_s, offset_offset, lane_config[_lane_name_s] - offset_offset))
		testcase_['npcList'][_i]['start']['heading']['ref_lane_point'] = testcase_['npcList'][_i]['start']['lane_position']
		if testcase_['npcList'][_i]['destination'] is not None:
			testcase_['npcList'][_i]['destination'].pop('position', None)
			testcase_['npcList'][_i]['destination']['heading'].pop('ref_point', None)
			_offset_s = testcase_['npcList'][_i]['destination']['lane_position']['offset']
			_lane_name_s = testcase_['npcList'][_i]['destination']['lane_position']['lane']
			testcase_['npcList'][_i]['destination']['lane_position']['offset'] = float(np.clip(_offset_s, offset_offset, lane_config[_lane_name_s] - offset_offset))
			testcase_['npcList'][_i]['destination']['heading']['ref_lane_point'] = testcase_['npcList'][_i]['destination']['lane_position']
		for _j in range(len(testcase_['npcList'][_i]['motion'])):
			testcase_['npcList'][_i]['motion'][_j].pop('position', None)
			testcase_['npcList'][_i]['motion'][_j]['heading'].pop('ref_point', None)
			_offset_s = testcase_['npcList'][_i]['motion'][_j]['lane_position']['offset']
			_lane_name_s = testcase_['npcList'][_i]['motion'][_j]['lane_position']['lane']
			testcase_['npcList'][_i]['motion'][_j]['lane_position']['offset'] = float(np.clip(_offset_s, offset_offset, lane_config[_lane_name_s] - offset_offset))
			testcase_['npcList'][_i]['motion'][_j]['heading']['ref_lane_point'] = testcase_['npcList'][_i]['motion'][_j]['lane_position']

	# for _i in range(len(testcase_['pedestrianList'])):
	#     # testcase_['pedestrianList'][_i]['start'].pop('position', None)
	#     testcase_['pedestrianList'][_i]['start']['heading'].pop('ref_point', None)
	#     if testcase_['pedestrianList'][_i]['destination'] is not None:
	#         # testcase_['pedestrianList'][_i]['destination'].pop('position', None)
	#         testcase_['pedestrianList'][_i]['destination']['heading'].pop('ref_point', None)
	#     for _j in range(len(testcase_['pedestrianList'][_i]['motion'])):
	#         # testcase_['pedestrianList'][_i]['motion'][_j].pop('position', None)
	#         testcase_['pedestrianList'][_i]['motion'][_j]['heading'].pop('ref_point', None)
	return testcase_

def testcase_encode(testcase):
	chrm = {'forward': [], 'right': [], 'rotation': [], 'type': []}
	_obs_number = len(testcase['obstacleList'])
	for _i in range(_obs_number):
		obs_i = testcase['obstacleList'][_i]
		chrm['forward'].append(obs_i['forward'])
		chrm['right'].append(obs_i['right'])
		chrm['rotation'].append(obs_i['rotation'])
		chrm['type'].append(obs_i['type'])
	return chrm

class EncodedTestCase:
	def __init__(self, trace, spec):
		self.trace = copy.deepcopy(trace)
		self.testcase = get_testcase(trace)
		self.chromosome = testcase_encode(self.testcase)
		self.spec = spec
		self.robustness = []
		self.fitness = float('inf')
		self.muti_fitness = dict()
		self.compute_fitness()

	def compute_fitness(self):
		if self.spec != {}:
			monitor = Monitor(self.trace, self.spec)
			self.fitness = monitor.continuous_monitor()

	def compute_muti_fitness(self):
		monitor = Monitor(self.trace, self.spec)
		self.muti_fitness = monitor.continuous_monitor_for_muti_traffic_rules()
		# print(self.muti_fitness)

class DecodedTestCase:
	def __init__(self, population_para):
		self.population = copy.deepcopy(population_para)

	def Decode_POP(self, p):
		new_testcase = copy.deepcopy(p.testcase)

		# decode obstacles
		try:
			obs_no = len(new_testcase['obstacleList'])
			for j in range(obs_no):
				new_testcase['obstacleList'][j]['forward'] = p.chromosome['forward'][j]
				new_testcase['obstacleList'][j]['right'] = p.chromosome['right'][j]
				new_testcase['obstacleList'][j]['rotation'] = p.chromosome['rotation'][j]
				new_testcase['obstacleList'][j]['type'] = p.chromosome['type'][j]
		except IndexError:
			print('Error!')
		return new_testcase

	def decoding(self):
		newTestCases = []
		for i in range(len(self.population)):
			_testcase = self.Decode_POP(self.population[i])
			newTestCases.append(_testcase)
		return newTestCases


class GAGeneration:
	def __init__(self, population_para, crossover_prob=1.0, mutation_prob=1.0):
		self.population = copy.deepcopy(population_para) # a list of EncodedTestCase
		self.p_cross = crossover_prob
		self.p_mutation = mutation_prob
		self.population_size = len(self.population)

	def selection(self, pop_size):
		selected_population = []
		sorted_pop = sorted(self.population, key=operator.attrgetter('fitness'), reverse=False)
		for i in range(pop_size):
			first_int = random.sample(range(0, math.ceil(self.population_size/2)), 1)[0]
			second_int = random.sample(range(0, self.population_size), 1)[0]
			# two_int = random.sample(range(0, self.population_size), 2)
			# p1 = copy.deepcopy(self.population[two_int[0]])
			# p2 = copy.deepcopy(self.population[two_int[1]])
			p1 = copy.deepcopy(sorted_pop[first_int])
			p2 = copy.deepcopy(sorted_pop[second_int])
			if p1.fitness < p2.fitness:
				selected_population.append(p1)
			else:
				selected_population.append(p2)
			del p1, p2
			gc.collect()
		return selected_population


	def selection2(self, pop_size):
		selected_population = []
		sorted_pop = sorted(self.population, key=operator.attrgetter('fitness'), reverse=True)
		for i in range(pop_size):
			first_int = random.sample(range(0, math.ceil(self.population_size/2)), 1)[0]
			second_int = random.sample(range(0, self.population_size), 1)[0]
			# two_int = random.sample(range(0, self.population_size), 2)
			# p1 = copy.deepcopy(self.population[two_int[0]])
			# p2 = copy.deepcopy(self.population[two_int[1]])
			p1 = copy.deepcopy(sorted_pop[first_int])
			p2 = copy.deepcopy(sorted_pop[second_int])
			if p1.fitness > p2.fitness:
				selected_population.append(p1)
			else:
				selected_population.append(p2)
			del p1, p2
			gc.collect()
		return selected_population

	def crossover(self, p1, p2, r_cross = 1): #range of r_cross is [0, 1], 0 indicates no crossover, 1 indicates crossover for sure
		# children are copies of parents by default
		new_p1 = copy.deepcopy(p1)
		new_p2 = copy.deepcopy(p2)

		chm01 = p1.chromosome['forward']
		chm02 = p2.chromosome['forward']

		chm11 = p1.chromosome['right']
		chm12 = p2.chromosome['right']

		chm21 = p1.chromosome['rotation']
		chm22 = p2.chromosome['rotation']

		chm31 = p1.chromosome['type']
		chm32 = p2.chromosome['type']

		if random.random() < r_cross and len(chm01) > 1:
			# select crossover point that is not on the end of the string
			crossover_point = random.randint(1, len(chm01)-1)
			# perform crossover
			new_p1.chromosome['forward'] = chm01[:crossover_point] + chm02[crossover_point:]
			new_p2.chromosome['forward'] = chm02[:crossover_point] + chm01[crossover_point:]

			crossover_point = random.randint(1, len(chm01)-1)
			new_p1.chromosome['right'] = chm11[:crossover_point] + chm12[crossover_point:]
			new_p2.chromosome['right'] = chm12[:crossover_point] + chm11[crossover_point:]

			crossover_point = random.randint(1, len(chm01)-1)
			new_p1.chromosome['rotation'] = chm21[:crossover_point] + chm22[crossover_point:]
			new_p2.chromosome['rotation'] = chm22[:crossover_point] + chm21[crossover_point:]

			crossover_point = random.randint(1, len(chm01)-1)
			new_p1.chromosome['type'] = chm31[:crossover_point] + chm32[crossover_point:]
			new_p2.chromosome['type'] = chm32[:crossover_point] + chm31[crossover_point:]

		return new_p1, new_p2

	def mutation(self, p, r_mut = 1): #range of r_mut is [0, 1], 0 indicates no crossover, 1 indicates crossover for sure
		new_p = copy.deepcopy(p)


		chm0 = p.chromosome['forward']
		chm1 = p.chromosome['right']
		chm2 = p.chromosome['rotation']
		chm3 = p.chromosome['type']

		assert len(chm0) == len(chm1) == len(chm2) == len(chm3)

		for i in range(len(chm0)): # check for a mutation
			if random.random() < r_mut:
				temp = gauss(chm0[i], 1)
				new_p.chromosome['forward'][i] = float(np.clip(temp, forward_min, forward_max))
			if random.random() < r_mut:
				temp = gauss(chm1[i], 1)
				new_p.chromosome['right'][i] = float(np.clip(temp, right_min, right_max))
			if random.random() < r_mut:
				temp = gauss(chm2[i], 1)
				new_p.chromosome['rotation'][i] = float(np.clip(temp, rotation_min, rotation_max))
			if random.random() < r_mut:
				new_p.chromosome['type'][i] = random.choice(obs_list)

		return new_p

	def check_distance_to_other_objects(self, positionx, type0, map_info, new_p, seq):
		x0 = positionx[0]
		y0 = positionx[1]

		start_x = new_p.testcase['ego']['start']['position']['x']
		start_y = new_p.testcase['ego']['start']['position']['y']

		if type0 == "Bin" or type0 == "BinGreen" or type0 == "BinRed" or type0 =="BinYellow" or type0 == "BigTrashBin":
			for _i in range(len(new_p.chromosome['forward'])):
				if (_i != seq):
					pos_forward = new_p.chromosome['forward'][_i]
					pos_right = new_p.chromosome['right'][_i]

					pos = map_info.relative2position(start_x, start_y, pos_forward, pos_right)
					x1 = pos[0]
					y1 = pos[1]

					if math.sqrt((x1 - x0)**2 + (y1 - y0)**2) < 0.5:
						return False
					if math.sqrt((x1 - x0)**2 + (y1 - y0)**2)+ map_info.dist_to_roads(x1, y1) < map_info.dist_to_roads(x0, y0):
						return False


		return True

	def check_validity(self, positionx, type0, map_info, new_p, seq):
		if map_info.check_whether_in_attack_area(positionx, type0) == False:
			return False
		if self.check_distance_to_other_objects(positionx, type0, map_info, new_p, seq) == False:
			return False

		return True




	def validation_check(self, p):
		new_p = copy.deepcopy(p)

		map_name = p.testcase['map']
		map_info = get_map_info(map_name)
		start_x = p.testcase['ego']['start']['position']['x']
		start_y = p.testcase['ego']['start']['position']['y']

		chm0 = p.chromosome['forward']
		chm1 = p.chromosome['right']
		chm3 = p.chromosome['type']

		assert len(chm0) == len(chm1)

		for i in range(len(chm0)):
			pos_forward = chm0[i]
			pos_right = chm1[i]
			positionx = map_info.relative2position(start_x, start_y, pos_forward, pos_right)
			type0 = chm3[i]

			while self.check_validity(positionx, type0, map_info, new_p, i) == False:
				pos_forward = gauss(pos_forward, 1)
				pos_right = gauss(pos_right, 1)
				pos_forward = float(np.clip(pos_forward, forward_min, forward_max))
				pos_right = float(np.clip(pos_right, right_min, right_max))

				positionx = map_info.relative2position(start_x, start_y, pos_forward, pos_right)
				new_p.chromosome['forward'][i] = pos_forward
				new_p.chromosome['right'][i] = pos_right

		return new_p

	def one_generation(self):
		# always keep the individual with the minimal fitness
		sorted_pop = sorted(self.population, key=operator.attrgetter('fitness'), reverse=True)
		try:
			_top1_pop = copy.deepcopy(sorted_pop[-1])
		except IndexError:
			print(len(sorted_pop))
		_new_population = [_top1_pop]
		selected_pop = self.selection(self.population_size-1)  # only need to select n-1 individual and perform mutation.
		for i in range(0, self.population_size - 1, 2):  # i = 0,2,4,...,n-3
			if i + 1 <= self.population_size - 2:
				p1 = copy.deepcopy(selected_pop[i])
				p2 = copy.deepcopy(selected_pop[i+1])
				p1, p2 = self.crossover(p1, p2, self.p_cross)
				p1 = self.mutation(p1, self.p_mutation)
				p2 = self.mutation(p2, self.p_mutation)
				p1 = self.validation_check(p1)
				p2 = self.validation_check(p2)
				_new_population.append(p1)
				_new_population.append(p2)
			else:
				new_p = copy.deepcopy(selected_pop[i])
				_new_population.append(new_p)

		return _new_population

	def one_generation_law_breaking(self, population_size):
		# always keep the individual with the minimal fitness
		# map_name = self.population[0].testcase['map']
		# map_info = get_map_info(map_name)
		# lane_info = map_info.get_lane_config()
		# crosswalk_info = map_info.get_crosswalk_config()
		sorted_pop = sorted(self.population, key=operator.attrgetter('fitness'), reverse=True)
		try:
			_top1_pop = copy.deepcopy(sorted_pop[-1])
		except IndexError:
			print(len(sorted_pop))
		_new_population = [_top1_pop]
		selected_pop = self.selection2(population_size-1)  # only need to select n-1 individual and perform mutation.
		for i in range(0, population_size - 1, 2):  # i = 0,2,4,...,n-3
			if i + 1 <= population_size - 2:
				p1 = copy.deepcopy(selected_pop[i])
				p2 = copy.deepcopy(selected_pop[i+1])
				p1, p2 = self.crossover(p1, p2, self.p_cross)
				p1 = self.mutation(p1, self.p_mutation)
				p2 = self.mutation(p2, self.p_mutation)
				p1 = self.validation_check(p1)
				p2 = self.validation_check(p2)
				# if random.random() < self.p_cross:
				#     p1, p2 = self.crossover(p1, p2)
				# if random.random() < self.p_mutation:
				#     p1 = self.mutation(p1, lane_info, crosswalk_info)
				# if random.random() < self.p_mutation:
				#     p2 = self.mutation(p2, lane_info, crosswalk_info)
				_new_population.append(p1)
				_new_population.append(p2)
			else:
				new_p = copy.deepcopy(selected_pop[i])
				_new_population.append(new_p)

		return _new_population



