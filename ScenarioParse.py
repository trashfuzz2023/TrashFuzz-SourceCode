#!/usr/bin/python

def Add_obs_to_obslist(scenario, forward, right, rotation, type0):
	obs = {}
	obs["forward"] = forward
	obs["right"] = right
	obs["rotation"] = rotation
	obs["type"] = type0

	scenario["obstacleList"].append(obs)

	return