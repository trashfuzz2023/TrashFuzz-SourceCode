import copy
import os
import select
import shutil
import signal
import sys
import time
import warnings
from pathlib import Path
from threading import Timer

import websockets
import json
import asyncio
from EXtraction import ExtractAll
from AttackGeneticAlgorithm import GAGeneration, EncodedTestCase, DecodedTestCase
from TestCaseRandom import TestCaseRandom
from datetime import datetime
from AssertionExtraction import SingleAssertion
from map import get_map_info
from monitor import Monitor
import logging
from spec_coverage import failure_statement

from GreedyMutation import GreedyMute, encoding_quantify





async def hello(scenario_msg, single_spec, max_num_of_cases = 100, max_obs_num=5, directory=None) -> object:
    # print(single_spec)
    maximun = len(single_spec.sub_violations)
    remaining_sub_violations = single_spec.sub_violations


    # mapping = dict()
    # for item in remaining_sub_violations:
    #     mapping[item] = -1000
    #     seed[item] = None


    uri = "ws://localhost:8000"
    async with websockets.connect(uri,  max_size= 300000000, ping_interval=None) as websocket:
        init_msg = json.dumps({'CMD': "CMD_READY_FOR_NEW_TEST"})
        await websocket.send(init_msg)
        msg = await websocket.recv()
        msg = json.loads(msg)
        if msg['TYPE'] == 'READY_FOR_NEW_TEST' and msg['DATA']:
            now = datetime.now()
            dt_string = now.strftime("%d/%m/%Y %H:%M:%S")
            with open(directory + '/Incompleted.txt', 'a') as f:
                f.write('Time: {} \n'.format(dt_string))
            with open(directory + '/bugTestCase.txt', 'a') as f:
                f.write('Time: {} \n'.format(dt_string))
            with open(directory + '/NoTrace.txt', 'a') as f:
                f.write('Time: {} \n'.format(dt_string))
            new_testcases = []
            init_msg = json.dumps({'CMD': "CMD_READY_FOR_NEW_TEST"})
            await websocket.send(init_msg)

            # get the initial test case (random)
            test_case = scenario_msg[0]
            testcase = TestCaseRandom(test_case)
            testcase.testcase_random(1, max_obs_num)

            initial_scenario = testcase.cases[0]
            scenario_msg = []
            scenario_msg.append(testcase.cases[0])

            i = 0;

            initial_sub_robustness = dict()
            for item in remaining_sub_violations:
                initial_sub_robustness[item] = 0
                # seed[item] = None

            initial_encoded_testcase = 0
            while i < max_num_of_cases:
                # run the initial test case
                assert len(scenario_msg) == 1
                while True:
                    msg = await websocket.recv()
                    msg = json.loads(msg)
                    if msg['TYPE'] == 'READY_FOR_NEW_TEST':
                        if msg['DATA']:
                            logging.info('Individual: {}'.format(i + 1))
                            send_msg = {'CMD': "CMD_NEW_TEST", 'DATA': scenario_msg[0]}
                            await websocket.send(json.dumps(send_msg))
                        else:
                            asyncio.sleep(3)
                            init_msg = json.dumps({'CMD': "CMD_READY_FOR_NEW_TEST"})
                            await websocket.send(init_msg)
                    elif msg['TYPE'] == 'KEEP_SERVER_AND_CLIENT_ALIVE':
                        send_msg = {'CMD': "KEEP_SERVER_AND_CLIENT_ALIVE", 'DATA': None}
                        await websocket.send(json.dumps(send_msg))
                    elif msg['TYPE'] == 'TEST_TERMINATED' or msg['TYPE'] == 'ERROR':
                        print("Try to reconnect!")
                        asyncio.sleep(3)
                        init_msg = json.dumps({'CMD': "CMD_READY_FOR_NEW_TEST"})
                        await websocket.send(init_msg)
                    elif msg['TYPE'] == 'TEST_COMPLETED':
                        i = i + 1
                        output_trace = msg['DATA']
                        now = datetime.now()
                        dt_string = now.strftime("%d-%m-%Y-%H-%M-%S")
                        file = directory + '/data/result' + dt_string + '.json'
                        with open(file, 'w') as outfile:
                            json.dump(output_trace, outfile, indent=2)
                        if not output_trace['destinationReached']:
                            logging.info("Not reach the destination")
                            with open(directory + '/Incompleted.txt', 'a') as f:
                                json.dump(scenario_msg[0], f, indent=2)
                                f.write('\n')
                        if len(output_trace['trace']) > 1:
                            encoded_testcase = EncodedTestCase(output_trace, single_spec)   
                            if encoded_testcase.muti_fitness == {}:
                                encoded_testcase.compute_muti_fitness()              
                            if 'Accident!' in output_trace["testFailures"]: 
                                encoded_testcase.compute_muti_fitness()
                                with open(directory + '/AccidentTestCase.txt', 'a') as bug_file:
                                    now = datetime.now()
                                    dt_string = now.strftime("%d-%m-%Y-%H-%M-%S")
                                    string_index = "Time:" + dt_string +" Individual: " + str(i + 1) +", Bug: " + str(output_trace["testFailures"]) +'\n'
                                    bug_file.write(string_index)
                                    string_index2 = "The detailed fitness values:" + str(encoded_testcase.muti_fitness) + '\n'
                                    bug_file.write(string_index2)
                                    json.dump(output_trace, bug_file, indent=2)
                                    bug_file.write('\n')   
                            covered = []
                            # if encoded_testcase.fitness <= 0.0:
                            monitor = Monitor(output_trace, 0)
                            for spec in remaining_sub_violations:                                  
                                fitness0 = monitor.continuous_monitor2(spec) 
                                initial_sub_robustness[spec] = fitness0                                                                        
                                if fitness0 >= 0.0:
                                    covered.append(spec)
                                    # logging.info("Coverage rate is: {}/{}, Covered Predicates are: {}".format(len(coverage_statement), len(all_predicates), coverage_statement))
                                    with open(directory + '/improvedTestCase.txt', 'a') as bug_file:
                                        now = datetime.now()
                                        dt_string = now.strftime("%d-%m-%Y-%H-%M-%S")
                                        string_index = "Time:" + dt_string +" Individual: " + str(i + 1) +'\n'
                                        bug_file.write(string_index)
                                        string_index2 = "The detailed fitness values:" + str(encoded_testcase.muti_fitness) + '\n'
                                        bug_file.write(string_index2)
                                        coverage_rate = 1- len(remaining_sub_violations) / maximun
                                        string_index3 = "total coverage rate: {}/{} = {}, new covered predicates: {}\n".format((maximun - len(remaining_sub_violations)), maximun, coverage_rate, spec)
                                        bug_file.write(string_index3)
                                        # bug_file.write(spec)
                                        json.dump(output_trace, bug_file, indent=2)
                                        bug_file.write('\n')                                           
                            for itme in covered:
                                remaining_sub_violations.remove(itme)
                                initial_sub_robustness.pop(itme, None)
                            del encoded_testcase.trace 
                            initial_encoded_testcase = encoded_testcase
                        elif len(output_trace['trace']) == 1:
                            testcase = TestCaseRandom(scenario_msg[0])
                            testcase.testcase_random(1, max_obs_num)
                            new_testcases.append(testcase.cases[-1])
                        else:
                            logging.info("No trace for the test cases")
                            with open(directory + '/NoTrace.txt', 'a') as f:
                                json.dump(scenario_msg[0], f, indent=2)
                                f.write('\n')
                            testcase = TestCaseRandom(scenario_msg[0])
                            testcase.testcase_random(1, max_obs_num)
                            new_testcases.append(testcase.cases[-1])
                        init_msg = json.dumps({'CMD': "CMD_READY_FOR_NEW_TEST"})
                        await websocket.send(init_msg)

                        with open("remaining_list.txt", "w") as filex:
                            for itemx in remaining_sub_violations:
                                filex.write(f"{itemx}\n")

                        asyncio.sleep(5)
                        break


                next_exploration = GreedyMute(initial_encoded_testcase)
                scenario_msg2 = next_exploration.one_generation_greedy()

                decoder = DecodedTestCase(scenario_msg2)
                next_new_testcases = decoder.decoding()
                scenario_msg = copy.deepcopy(next_new_testcases)

                assert len(scenario_msg) == max_obs_num


                gradient = []
                for j in range(len(scenario_msg)):
                    temp_case = dict()
                    for item in remaining_sub_violations:
                        temp_case[item] = 0
                    while True:
                        msg = await websocket.recv()
                        msg = json.loads(msg)
                        if msg['TYPE'] == 'READY_FOR_NEW_TEST':
                            if msg['DATA']:
                                logging.info('Individual: {}'.format(i + 1))
                                send_msg = {'CMD': "CMD_NEW_TEST", 'DATA': scenario_msg[j]}
                                await websocket.send(json.dumps(send_msg))
                            else:
                                asyncio.sleep(3)
                                init_msg = json.dumps({'CMD': "CMD_READY_FOR_NEW_TEST"})
                                await websocket.send(init_msg)
                        elif msg['TYPE'] == 'KEEP_SERVER_AND_CLIENT_ALIVE':
                            send_msg = {'CMD': "KEEP_SERVER_AND_CLIENT_ALIVE", 'DATA': None}
                            await websocket.send(json.dumps(send_msg))
                        elif msg['TYPE'] == 'TEST_TERMINATED' or msg['TYPE'] == 'ERROR':
                            print("Try to reconnect!")
                            asyncio.sleep(3)
                            init_msg = json.dumps({'CMD': "CMD_READY_FOR_NEW_TEST"})
                            await websocket.send(init_msg)
                        elif msg['TYPE'] == 'TEST_COMPLETED':
                            i = i + 1
                            output_trace = msg['DATA']
                            now = datetime.now()
                            dt_string = now.strftime("%d-%m-%Y-%H-%M-%S")
                            file = directory + '/data/result' + dt_string + '.json'
                            with open(file, 'w') as outfile:
                                json.dump(output_trace, outfile, indent=2)
                            if not output_trace['destinationReached']:
                                logging.info("Not reach the destination")
                                with open(directory + '/Incompleted.txt', 'a') as f:
                                    json.dump(scenario_msg[j], f, indent=2)
                                    f.write('\n')
                            if len(output_trace['trace']) > 1:
                                encoded_testcase = EncodedTestCase(output_trace, single_spec)     
                                if encoded_testcase.muti_fitness == {}:
                                    encoded_testcase.compute_muti_fitness()              
                                if 'Accident!' in output_trace["testFailures"]: 
                                    encoded_testcase.compute_muti_fitness()
                                    with open(directory + '/AccidentTestCase.txt', 'a') as bug_file:
                                        now = datetime.now()
                                        dt_string = now.strftime("%d-%m-%Y-%H-%M-%S")
                                        string_index = "Time:" + dt_string +" Individual: " + str(i + 1) +", Bug: " + str(output_trace["testFailures"]) +'\n'
                                        bug_file.write(string_index)
                                        string_index2 = "The detailed fitness values:" + str(encoded_testcase.muti_fitness) + '\n'
                                        bug_file.write(string_index2)
                                        json.dump(output_trace, bug_file, indent=2)
                                        bug_file.write('\n')   
                                covered = []
                                # if encoded_testcase.fitness <= 0.0:
                                monitor = Monitor(output_trace, 0)
                                for spec in remaining_sub_violations:                                  
                                    fitness0 = monitor.continuous_monitor2(spec) 
                                    if (encoding_quantify(encoded_testcase) - encoding_quantify(initial_encoded_testcase)) != 0:
                                        temp_case[spec] = -(fitness0 - initial_sub_robustness[spec]) / (encoding_quantify(encoded_testcase) - encoding_quantify(initial_encoded_testcase))
                                    else:
                                        print("It's the same!")
                                        temp_case[spec] = 0
                                    if fitness0 >= 0.0:
                                        covered.append(spec)
                                        # logging.info("Coverage rate is: {}/{}, Covered Predicates are: {}".format(len(coverage_statement), len(all_predicates), coverage_statement))
                                        with open(directory + '/improvedTestCase.txt', 'a') as bug_file:
                                            now = datetime.now()
                                            dt_string = now.strftime("%d-%m-%Y-%H-%M-%S")
                                            string_index = "Time:" + dt_string +" Individual: " + str(i + 1) +'\n'
                                            bug_file.write(string_index)
                                            string_index2 = "The detailed fitness values:" + str(encoded_testcase.muti_fitness) + '\n'
                                            bug_file.write(string_index2)
                                            coverage_rate = 1- len(remaining_sub_violations) / maximun
                                            string_index3 = "total coverage rate: {}/{} = {}, new covered predicates: {}\n".format((maximun - len(remaining_sub_violations)), maximun, coverage_rate, spec)
                                            bug_file.write(string_index3)
                                            # bug_file.write(spec)
                                            json.dump(output_trace, bug_file, indent=2)
                                            bug_file.write('\n')                                           
                                for itme in covered:
                                    remaining_sub_violations.remove(itme)
                                    temp_case.pop(itme, None)
                                del encoded_testcase.trace 
                            elif len(output_trace['trace']) == 1:
                                testcase = TestCaseRandom(scenario_msg[j])
                                testcase.testcase_random(1, max_obs_num)
                                new_testcases.append(testcase.cases[-1])
                            else:
                                logging.info("No trace for the test cases")
                                with open(directory + '/NoTrace.txt', 'a') as f:
                                    json.dump(scenario_msg[j], f, indent=2)
                                    f.write('\n')
                                testcase = TestCaseRandom(scenario_msg[j])
                                testcase.testcase_random(1, max_obs_num)
                                new_testcases.append(testcase.cases[-1])
                            init_msg = json.dumps({'CMD': "CMD_READY_FOR_NEW_TEST"})
                            await websocket.send(init_msg)

                            gradient.append(temp_case)

                            with open("remaining_list.txt", "w") as filex:
                                for itemx in remaining_sub_violations:
                                    filex.write(f"{itemx}\n")

                            asyncio.sleep(5)
                            break

                
                chosen_value = 0
                for spec in remaining_sub_violations:  
                    for xx in range(len(gradient)):
                        if abs(chosen_value) < abs(gradient[xx][spec]):
                            chosen_one = scenario_msg[xx]
                            chosen_value = gradient[xx][spec]
                scenario_msg = []
                scenario_msg.append(chosen_one)
                

            coverage_rate = 1- len(remaining_sub_violations) / maximun
            logging.info("total coverage rate: {}/{} = {}, uncovered predicates: {}\n".format((maximun - len(remaining_sub_violations)), maximun, coverage_rate, remaining_sub_violations))
            mapping = sorted(mapping.items(), key=lambda item: item[1], reverse=True)
            mapping = dict(mapping)




def spec_scenario(spec, testcase, max_num_of_case = 100, max_obs_num=5, file_directory=None):
    loop = asyncio.get_event_loop()
    scenario_specification = copy.deepcopy(spec)
    scenario_testcase = copy.deepcopy(testcase)
    msgs = [scenario_testcase]
    loop.run_until_complete(
        asyncio.gather(hello(msgs, scenario_specification, max_num_of_cases = max_num_of_case, max_obs_num = max_obs_num,
                             directory=file_directory)))



def test_scenario(input_file, num2):
    file = input_file

    log_direct ='The_Results/'+'num_'+ str(num2)
    if not os.path.exists(log_direct):
        os.makedirs(log_direct)
    else:
        shutil.rmtree(log_direct)

    if not os.path.exists(log_direct + '/data'):
        os.makedirs(log_direct + '/data')

    logging_file = log_direct + '/test.log'
    file_handler = logging.FileHandler(logging_file, mode='w')
    stdout_handler = logging.StreamHandler(sys.stdout)
    logging.basicConfig(level=logging.INFO, handlers=[stdout_handler, file_handler],
                        format='%(asctime)s, %(levelname)s: %(message)s', datefmt="%Y-%m-%d %H:%M:%S")
    # logging.info("Current Test Case: {}".format(item))
    isGroundTruth = True
    extracted_data = ExtractAll(file, isGroundTruth)
    origin_case = extracted_data.Get_TestCastINJsonList()
    # print(origin_case)
    all_specifications = extracted_data.Get_Specifications()
    maps = extracted_data.Get_AllMaps()

    for i in range(len(origin_case)):
        test_case = origin_case[i]
        scenario_name = test_case['ScenarioName']
        logging.info("Current scenario is {}.\n".format(scenario_name))
        try:
            specifications_in_scenario = all_specifications[scenario_name]
            current_map = maps[scenario_name]
            ego_init_start = test_case['ego']['start']
            map_info = get_map_info(current_map)
            if "lane_position" in ego_init_start.keys():
                lane_position = ego_init_start['lane_position']
                ego_position = map_info.get_position([lane_position['lane'], lane_position['offset']])
            else:
                ego_position = (ego_init_start['position']['x'], ego_init_start['position']['y'], ego_init_start['position']['z'])
            for spec_index in range(len(specifications_in_scenario)):
                first_specification = specifications_in_scenario[spec_index]
                single_specification = SingleAssertion(first_specification, current_map, ego_position)
                logging.info("\n Evaluate Scenario {} with Assertion {}: {} \n ".format(scenario_name, spec_index, single_specification.specification))
                # spec_scenario(spec=single_specification, testcase=bug_cases, generations=25, pop_size=20,
                              # file_directory=log_direct)
                spec_scenario(spec=single_specification, testcase=test_case, max_num_of_case = 600, max_obs_num = num2, file_directory=log_direct)
                # print(testcase)
        except KeyError:
            spec_scenario(spec={}, testcase=test_case)


if __name__ == "__main__":
    # direct = 'test_cases/traffic_rule_tests/'
    # arr = ['Double-Direction-1.txt']

    # for item in arr:
    #     test_scenario(direct, item)

    input_file = 'codes_testing/laws.txt'

    test_scenario(input_file, 1)

    # for num in range(2):
    #     num2 = 2 - num
    #     test_scenario(input_file, num2)
    


