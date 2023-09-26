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
from ScenarioParse import Add_obs_to_obslist

import numpy as np

import logging






async def fuzzing(scenario_msg, single_spec, generation_number=1, population_size=20, max_obs_num = 15, directory=None) -> object:
    # print(single_spec)
    maximun = len(single_spec.sub_violations)
    remaining_sub_violations = single_spec.sub_violations

    start_time = 0
    end_time = 0

    robustness_value_list = []
    runningtime_value_list = []

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
            population = []
            population_temp = []
            init_msg = json.dumps({'CMD': "CMD_READY_FOR_NEW_TEST"})
            await websocket.send(init_msg)

            if len(population) < population_size: # get the initial population (random)
                test_case = scenario_msg[0]
                testcase = TestCaseRandom(test_case)
                testcase.testcase_random(population_size, max_obs_num)
                for kk in range(len(testcase.cases)):
                    population.append(testcase.cases[kk])


            #process with the generations
            for generation in range(generation_number):
                assert len(population) == population_size
                for i in range(population_size):
                    while True:
                        msg = await websocket.recv()
                        msg = json.loads(msg)
                        if msg['TYPE'] == 'READY_FOR_NEW_TEST':
                            if msg['DATA']:
                                logging.info('Running Generation: {}, Individual: {}'.format(generation, i + 1))
                                send_msg = {'CMD': "CMD_NEW_TEST", 'DATA': population[i]}
                                await websocket.send(json.dumps(send_msg))
                                start_time = time.time()
                            else:
                                time.sleep(3)
                                init_msg = json.dumps({'CMD': "CMD_READY_FOR_NEW_TEST"})
                                await websocket.send(init_msg)
                        elif msg['TYPE'] == 'KEEP_SERVER_AND_CLIENT_ALIVE':
                            send_msg = {'CMD': "KEEP_SERVER_AND_CLIENT_ALIVE", 'DATA': None}
                            await websocket.send(json.dumps(send_msg))
                        elif msg['TYPE'] == 'TEST_TERMINATED' or msg['TYPE'] == 'ERROR':
                            print(msg['TYPE'] + "! Try to reconnect!")
                            time.sleep(3)
                            init_msg = json.dumps({'CMD': "CMD_READY_FOR_NEW_TEST"})
                            await websocket.send(init_msg)
                        elif msg['TYPE'] == 'TEST_COMPLETED':
                            # Add duration time for each test case
                            end_time = time.time()
                            duration_period = 0
                                
                            try: # record the time spent
                                duration_period = end_time - start_time
                                runningtime_value_list.append(duration_period)
                                average_running_time = np.sum(runningtime_value_list)/len(runningtime_value_list)
                                with open(directory + '/DurationTimeRecord.txt', 'a') as bug_file:
                                    now = datetime.now()
                                    dt_string = now.strftime("%d-%m-%Y-%H-%M-%S")
                                    string_index = "Time:" + dt_string + ", TestCase: " + str(i + 1)+'\n'
                                    bug_file.write(string_index)
                                    string_index2 = "Duration_period: "+ str(duration_period) + "s" +'\n'
                                    bug_file.write(string_index2)
                                    bug_file.write('\n')
                                start_time = 0
                                end_time = 0
                            except:
                                print("Record Fail!!!")
                                with open(directory + '/DurationTimeRecord.txt', 'a') as bug_file:
                                    now = datetime.now()
                                    dt_string = now.strftime("%d-%m-%Y-%H-%M-%S")
                                    string_index = "Time:" + dt_string + ", TestCase: " + str(i + 1)+'\n'
                                    bug_file.write(string_index)
                                    string_index2 = "Record Fail!!!"+ '\n'
                                    bug_file.write(string_index2)
                                    bug_file.write('\n')  
                                end_time = 0

                            output_trace = msg['DATA']
                            now = datetime.now()
                            dt_string = now.strftime("%d-%m-%Y-%H-%M-%S")
                            file = directory + '/data/result' + dt_string + '.json'
                            with open(file, 'w') as outfile:
                                json.dump(output_trace, outfile, indent=2)
                            if not output_trace['destinationReached']:
                                logging.info("Not reach the destination")
                                with open(directory + '/Incompleted.txt', 'a') as f:
                                    json.dump(population[i], f, indent=2)
                                    f.write('\n')
                            if len(output_trace['trace']) > 1:                            
                                encoded_testcase = EncodedTestCase(output_trace, single_spec)  
                                if encoded_testcase.muti_fitness == {}:
                                    encoded_testcase.compute_muti_fitness()  
                                with open(directory + '/OverallResult.txt', 'a') as bug_file:
                                    now = datetime.now()
                                    dt_string = now.strftime("%d-%m-%Y-%H-%M-%S")
                                    string_index = "Time:" + dt_string + "Generation: " + str(generation) +", TestCase: " + str(i + 1) +", Bug: " + str(output_trace["testFailures"]) +'\n'
                                    bug_file.write(string_index)
                                    string_index1 = "The overall fitness value:" + str(encoded_testcase.fitness) + '\n'
                                    bug_file.write(string_index1)
                                    string_index2 = "The detailed fitness values:" + str(encoded_testcase.muti_fitness) + '\n'
                                    bug_file.write(string_index2)
                                    bug_file.write('\n') 
                                if encoded_testcase.fitness <= 0.0: # fitness 
                                    print("Bingo!!!")                                                                                                        
                                    with open(directory + '/RecordedTestCase.txt', 'a') as bug_file:
                                        now = datetime.now()
                                        dt_string = now.strftime("%d-%m-%Y-%H-%M-%S")
                                        string_index = "Time:" + dt_string + "Generation: " + str(generation) + ", Individual: " + str(i + 1) +'\n'
                                        bug_file.write(string_index)
                                        string_index2 = "The detailed fitness values:" + str(encoded_testcase.muti_fitness) + '\n'
                                        bug_file.write(string_index2)
                                        json.dump(output_trace, bug_file, indent=2)
                                        bug_file.write('\n')                                                                                    
                                if 'Accident!' in output_trace["testFailures"]: 
                                    with open(directory + '/AccidentTestCase.txt', 'a') as bug_file:
                                        now = datetime.now()
                                        dt_string = now.strftime("%d-%m-%Y-%H-%M-%S")
                                        string_index = "Time:" + dt_string + "Generation: " + str(0) + ", Individual: " + str(i + 1) +", Bug: " + str(output_trace["testFailures"]) +'\n'
                                        bug_file.write(string_index)
                                        string_index2 = "The detailed fitness values:" + str(encoded_testcase.muti_fitness) + '\n'
                                        bug_file.write(string_index2)
                                        json.dump(output_trace, bug_file, indent=2)
                                        bug_file.write('\n')                                       
                                del encoded_testcase.trace 
                                population_temp.append(encoded_testcase)
                            else:
                                testcase = TestCaseRandom(scenario_msg[0])
                                testcase.testcase_random(1, max_obs_num)
                                population_temp.append(testcase.cases[-1])
                                logging.info("No trace for the test cases")
                            time.sleep(3)
                            init_msg = json.dumps({'CMD': "CMD_READY_FOR_NEW_TEST"})
                            await websocket.send(init_msg)
                            break

                if len(population_temp): 
                    assert len(population_temp) == population_size
                    new_population_obj = GAGeneration(population_temp, crossover_prob = 0.5, mutation_prob = 0.2)
                    new_population = new_population_obj.one_generation()
                    decoder = DecodedTestCase(new_population)
                    ga_new_testcases = decoder.decoding()
                    # population_temp2.extend(ga_new_testcases)
                population = copy.deepcopy(ga_new_testcases)
                del population_temp
                population_temp = []
           
            
def spec_scenario(spec, testcase, generations=0, pop_size=1, max_obs_num = 5, file_directory=None):
    loop = asyncio.get_event_loop()
    scenario_specification = copy.deepcopy(spec)
    scenario_testcase = copy.deepcopy(testcase)
    msgs = [scenario_testcase]

    with open(file_directory + '/InitTestCase.txt', 'w') as f:
        json.dump(scenario_testcase, f, indent=2)
    loop.run_until_complete(
        asyncio.gather(fuzzing(msgs, scenario_specification, generation_number=generations, population_size=pop_size, max_obs_num = max_obs_num, directory=file_directory)))


def test_scenario(log_direct, test_case, scenario_spec):
    # log_direct ='/data/Gradient-REDriver/ForData/ModifiedApollo7.0-0.9/' + Path(item).stem

    if not os.path.exists(log_direct):
        os.makedirs(log_direct)
    else:
        shutil.rmtree(log_direct)

    if not os.path.exists(log_direct + '/data'):
        os.makedirs(log_direct + '/data')

    logging_file = log_direct + '/test.log'
    file_handler = logging.FileHandler(logging_file, mode='w')
    stdout_handler = logging.StreamHandler(sys.stdout)
    logging.basicConfig(level=logging.INFO, handlers=[stdout_handler, file_handler], format='%(asctime)s, %(levelname)s: %(message)s', datefmt="%Y-%m-%d %H:%M:%S")


    scenario_name = test_case['ScenarioName']
    try:
        specifications_in_scenario = scenario_spec[scenario_name]
        current_map = test_case['map']
        # print(current_map)
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
            spec_scenario(spec=single_specification, testcase=test_case, generations=30, pop_size=20, max_obs_num = 0, file_directory=log_direct)
    except KeyError:
        spec_scenario(spec={}, testcase=test_case)



if __name__ == "__main__":
    # input_file = 'codes_testing/law38.txt'
    # input_file = 'codes_testing/law51_4_5.txt'
    input_file = 'codes_testing/law38_test2.txt'
    # input_file = 'codes_testing/law38_sub_1.txt'
    isGroundTruth = True
    extracted_script = ExtractAll(input_file,isGroundTruth)
    origin_case = extracted_script.Get_TestCastINJsonList()
    scenario_spec = extracted_script.Get_Specifications()


    direct = '/data/Attack-Of-ADS/Apollo7.0.0/law38/num_0_5ss'

    test_scenario(direct, origin_case[0], scenario_spec)




