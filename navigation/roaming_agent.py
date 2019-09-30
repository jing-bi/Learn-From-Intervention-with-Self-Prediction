#!/usr/bin/env python

# Copyright (c) 2018 Intel Labs.
# authors: German Ros (german.ros@intel.com)
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

""" This module implements an robot that roams around a track following random waypoints and avoiding other vehicles.
The robot also responds to traffic lights. """

from navigation.auto_agent import Agent, AgentState
from navigation.local_planner import LocalPlanner


class RoamingAgent(Agent):
    """
    RoamingAgent implements a basic robot that navigates scenes making random
    choices when facing an intersection.

    This robot respects traffic lights and other vehicles.
    """

    def __init__(self, vehicle):
        """

        :param vehicle: actor to apply to local planner logic onto
        """
        super(RoamingAgent, self).__init__(vehicle)
        self._proximity_threshold = 12.0  # meters
        self._state = AgentState.NAVIGATING
        self._local_planner = LocalPlanner(self._vehicle)
        self._previous_roadoption=None
        self.new_plan=False
    def run_step(self,debug=False):
        """
        Execute one step of navigation.
        :return: carla.VehicleControl
        """

        # is there an obstacle in front of us?
        hazard_detected = False

        # retrieve relevant elements for safe navigation, i.e.: traffic lights
        # and other vehicles
        actor_list = self._world.get_actors()
        vehicle_list = actor_list.filter("*vehicle*")
        lights_list = actor_list.filter("*traffic_light*")
        stopsign_list=actor_list.filter("*traffic.stop*")
        # check possible obstacles
        vehicle_state, vehicle = self._is_vehicle_hazard(vehicle_list)
        if vehicle_state:
            if debug:
                print('!!! VEHICLE BLOCKING AHEAD [{}])'.format(vehicle.id))

            self._state = AgentState.BLOCKED_BY_VEHICLE
            hazard_detected = True

        # check for the state of the traffic lights
        # light_state, traffic_light = self._is_light_red(lights_list)
        # if light_state:
        #     if debug:
        #         print('=== RED LIGHT AHEAD [{}])'.format(traffic_light.id))
        #
        #     self._state = AgentState.BLOCKED_RED_LIGHT
        #     hazard_detected = True
        # check for the state of stop sign
        # stopsign_state = self._is_stop_sign(stopsign_list)
        # if stopsign_state:
        #     if debug:
        #         print('*** STOP SIGN AHEAD')
        #
        #     self._state = AgentState.BLOCKED_STOP_SIGN
        #     # hazard_detected = True


        result = self._local_planner.run_step(new_plan=self.new_plan)
        # print(self.new_plan)
        if hazard_detected:
            result['control'] = self.emergency_stop()
        return result
