import copy
import gc
import random
import math
from typing import Any, Dict

from TestCaseExtraction import AllTestCase
from EXtraction import ExtractAll
from TestCaseExtraction import TestCase
import json
from random import gauss
import numpy as np
from map import get_map_info
from pedestrian_motion_checking import nearest

from config import get_obs_list

obs_list = get_obs_list()




class TestCaseRandom:
    def __init__(self, msg):
        testcase_ = {}
        testcase_['ScenarioName'] = msg['ScenarioName']
        testcase_['MapVariable'] = msg['MapVariable']
        testcase_['map'] = msg['map']
        testcase_['time'] = msg['time']
        testcase_['weather'] = msg['weather']
        testcase_['ego'] = msg['ego']
        testcase_['npcList'] = msg['npcList']
        testcase_['pedestrianList'] = msg['pedestrianList']
        testcase_['obstacleList'] = msg['obstacleList']
        testcase_['AgentNames'] = msg['AgentNames']

        for _i in range(len(testcase_['npcList'])):
            testcase_['npcList'][_i]['start'].pop('position', None)
            testcase_['npcList'][_i]['start']['heading'].pop('ref_point', None)
            if testcase_['npcList'][_i]['destination'] is not None:
                testcase_['npcList'][_i]['destination'].pop('position', None)
                testcase_['npcList'][_i]['destination']['heading'].pop('ref_point', None)
            for _j in range(len(testcase_['npcList'][_i]['motion'])):
                testcase_['npcList'][_i]['motion'][_j].pop('position', None)
                testcase_['npcList'][_i]['motion'][_j]['heading'].pop('ref_point', None)

        for _i in range(len(testcase_['pedestrianList'])):
            # testcase_['pedestrianList'][_i]['start'].pop('position', None)
            testcase_['pedestrianList'][_i]['start']['heading'].pop('ref_point', None)
            if testcase_['pedestrianList'][_i]['destination'] is not None:
                # testcase_['pedestrianList'][_i]['destination'].pop('position', None)
                testcase_['pedestrianList'][_i]['destination']['heading'].pop('ref_point', None)
            for _j in range(len(testcase_['pedestrianList'][_i]['motion'])):
                # testcase_['pedestrianList'][_i]['motion'][_j].pop('position', None)
                testcase_['pedestrianList'][_i]['motion'][_j]['heading'].pop('ref_point', None)


        self.original = testcase_
        self.cases = []

    def check_distance_to_other_objects(self, positionx, type0, map_info):
        x0 = positionx[0]
        y0 = positionx[1]

        if type0 == "Bin" or type0 == "BinGreen" or type0 == "BinRed" or type0 =="BinYellow" or type0 == "BigTrashBin":
            for case in self.new_obs_list:
                x1 = case['positionx'][0]
                y1 = case['positionx'][1]
                if math.sqrt((x1 - x0)**2 + (y1 - y0)**2) < 0.5:
                    print('F1')
                    return False
                if math.sqrt((x1 - x0)**2 + (y1 - y0)**2)+ map_info.dist_to_roads(x1, y1) < map_info.dist_to_roads(x0, y0):
                    print('F2')
                    return False


        return True

    def check_validity(self, positionx, type0, map_info):
        if map_info.check_whether_in_attack_area(positionx, type0) == False:
            # print("F0")
            return False
        if self.check_distance_to_other_objects(positionx, type0, map_info) == False:
            return False

        return True


    def testcase_random(self, num, num_of_obs = 30):
        for _i in range(num):
            _new_case = copy.deepcopy(self.original)
            # if not _new_case['ego']['groundTruthPerception']:
            map_name = _new_case['map']
            map_info = get_map_info(map_name)

            start_x = _new_case['ego']['start']['position']['x']
            start_y = _new_case['ego']['start']['position']['y']

            forward_max = 250
            forward_min = 100

            right_max = 10
            right_min = -10

            rotation_max = 360
            rotation_min = 0

            # num_of_obs = random.randint(1, max_obs_num)

            self.new_obs_list = []

            for i in range(num_of_obs):
                new_obs = {}
                new_obs['type'] = random.choice(obs_list)


                pos_forward = random.uniform(forward_min, forward_max)
                pos_right = random.uniform(right_min, right_max)
                positionx = map_info.relative2position(start_x, start_y, pos_forward, pos_right)


                while self.check_validity(positionx, new_obs['type'], map_info) == False:
                    pos_forward = random.uniform(forward_min, forward_max)
                    # pos_forward = gauss(pos_forward, 1)
                    pos_right = random.uniform(right_min, right_max)
                    # pos_right = gauss(pos_right, 1)
                    positionx = map_info.relative2position(start_x, start_y, pos_forward, pos_right)

                
                new_obs['forward'] = pos_forward
                new_obs['right'] = pos_right
                
                new_obs['rotation'] = random.uniform(rotation_min, rotation_max)

                new_obs['positionx'] = positionx
                

                _new_case['obstacleList'].append(new_obs)

                self.new_obs_list.append(new_obs)
                

            assert num_of_obs == len(_new_case['obstacleList'])
            self.cases.append(_new_case)



if __name__ == "__main__":
    # file_name = 'result1.json'
    # with open(file_name) as f:
    #     data = json.load(f)
    #     testcase = TestCaseRandom(data)
    #     testcase.testcase_random(5, 20)
        # print(testcase.cases[i])

    input_file = 'codes_testing/law46_2.txt'
    isGroundTruth = True
    extracted_script = ExtractAll(input_file,isGroundTruth)
    origin_case = extracted_script.Get_TestCastINJsonList()

    testcase = TestCaseRandom(origin_case[0])
    testcase.testcase_random(5, 20)

    for i in range(len(testcase.cases)):
        print(len(testcase.cases[i]['obstacleList']))
