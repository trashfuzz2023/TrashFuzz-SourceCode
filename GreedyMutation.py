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

def get_sequence_value(value):
    value_to_number = {val: num for num, val in enumerate(obs_list)}
    return value_to_number.get(value)


class list2d_convert:
    def __init__(self, list_2d):
        self.list2d = list_2d
        self.list1d = []
        self.len_list = []
        self.len = 0
        self.to_1d()
        self.len_compute()

    def to_1d(self):
        self.list1d = list(chain.from_iterable(self.list2d))
        self.len = len(self.list1d)

    def len_compute(self):
        for _i in range(len(self.list2d)):
            element_len = len(self.list2d[_i])
            self.len_list.append(element_len)

    def to_2d(self):
        _it = iter(self.list1d)
        return [list(islice(_it, i)) for i in self.len_list]


def encoding_quantify(scenario):
    result = 0
    chm0 = scenario.chromosome['forward']
    chm1 = scenario.chromosome['right']
    chm2 = scenario.chromosome['rotation']
    chm3 = scenario.chromosome['type']

    for i in range(len(chm0)):
        result = result + chm0[i] + chm1[i] + chm2[i] + get_sequence_value(chm3[i])

    return result


class GreedyMute:
    def __init__(self, scenario_msg):
        self.initial = scenario_msg # an initial case of EncodedTestCase

    # def random_mutation(self, p, category, sequence): #range of r_mut is [0, 1], 0 indicates no crossover, 1 indicates crossover for sure
    #     new_p = copy.deepcopy(p)

    #     if category == 'type':   
    #         new_p.chromosome['type'][sequence] = random.choice(obs_list)
    #     elif category == 'rotation':
    #         temp = gauss(new_p.chromosome[category][sequence], 1)
    #         new_p.chromosome[category][sequence] = float(np.clip(temp, rotation_min, rotation_max))
    #     elif category == 'right':
    #         temp = gauss(new_p.chromosome[category][sequence], 1)
    #         new_p.chromosome[category][sequence] = float(np.clip(temp, right_min, right_max))
    #     elif category == 'foward':
    #         temp = gauss(new_p.chromosome[category][sequence], 1)
    #         new_p.chromosome[category][sequence] = float(np.clip(temp, forward_min, forward_max))

    #     return new_p

    def rand_mutation(self, p, r_mut = 1): #range of r_mut is [0, 1], 0 indicates no crossover, 1 indicates crossover for sure
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


    def mutation(self, p, sequence): #range of r_mut is [0, 1], 0 indicates no crossover, 1 indicates crossover for sure
        new_p = copy.deepcopy(p)

        chm0 = p.chromosome['forward']
        chm1 = p.chromosome['right']
        chm2 = p.chromosome['rotation']
        chm3 = p.chromosome['type']

        assert len(chm0) == len(chm1) == len(chm2) == len(chm3)

        r_mut = 0.7
        # for i in range(len(chm0)): # check for a mutation
        if random.random() < r_mut:
            temp = gauss(chm0[sequence], 1)
            new_p.chromosome['forward'][sequence] = float(np.clip(temp, forward_min, forward_max))
        if random.random() < r_mut:
            temp = gauss(chm1[sequence], 1)
            new_p.chromosome['right'][sequence] = float(np.clip(temp, right_min, right_max))
        if random.random() < r_mut:
            temp = gauss(chm2[sequence], 1)
            new_p.chromosome['rotation'][sequence] = float(np.clip(temp, rotation_min, rotation_max))
        if random.random() < r_mut:
            new_p.chromosome['type'][sequence] = random.choice(obs_list)

        r_mut = 0.3
        for i in range(len(chm0)): # check for a mutation
            if i != sequence:
                if random.random() < r_mut:
                    # temp = gauss(chm0[i], 1)
                    # new_p.chromosome['forward'][i] = float(np.clip(temp, forward_min, forward_max))
                    new_p.chromosome['forward'][i] = random.uniform(forward_min, forward_max)
                if random.random() < r_mut:
                    # temp = gauss(chm1[i], 1)
                    # new_p.chromosome['right'][i] = float(np.clip(temp, right_min, right_max))
                    new_p.chromosome['right'][i] = random.uniform(right_min, right_max)
                if random.random() < r_mut:
                    # temp = gauss(chm2[i], 1)
                    # new_p.chromosome['rotation'][i] = float(np.clip(temp, rotation_min, rotation_max))
                    new_p.chromosome['rotation'][i] = random.uniform(rotation_min, rotation_max)
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

            patience = 100

            while self.check_validity(positionx, type0, map_info, new_p, i) == False:
                patience = patience - 1
                if patience > 0:
                    pos_forward = gauss(pos_forward, 1)
                    pos_right = gauss(pos_right, 1)
                    pos_forward = float(np.clip(pos_forward, forward_min, forward_max))
                    pos_right = float(np.clip(pos_right, right_min, right_max))

                    positionx = map_info.relative2position(start_x, start_y, pos_forward, pos_right)
                    new_p.chromosome['forward'][i] = pos_forward
                    new_p.chromosome['right'][i] = pos_right
                else:
                    pos_forward = random.uniform(forward_min, forward_max)
                    pos_right = random.uniform(right_min, right_max)

                    positionx = map_info.relative2position(start_x, start_y, pos_forward, pos_right)
                    new_p.chromosome['forward'][i] = pos_forward
                    new_p.chromosome['right'][i] = pos_right
                    print("Out of patience!")
                    patience = 100
        return new_p

    def one_generation_greedy(self):
        initial_one = copy.deepcopy(self.initial)
        _new_population = []

        chm0 = initial_one.chromosome['forward']
        for i in range(len(chm0)):
            p1 = self.mutation(initial_one, i)
            # p2 = self.mutation(initial_one, 'forward', i)
            # p3 = self.mutation(initial_one, 'right', i)
            # p4 = self.mutation(initial_one, 'rotation', i)

            p1 = self.validation_check(p1)
            # p2 = self.validation_check(p2)
            # p3 = self.validation_check(p3)
            # p4 = self.validation_check(p4)

            _new_population.append(p1)
            # _new_population.append(p2)
            # _new_population.append(p3)
            # _new_population.append(p4)

        return _new_population
                
