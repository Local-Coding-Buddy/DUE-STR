import os
import sys
import optparse
from core.Estimation import *
from xml.dom.minidom import parse, parseString
from core.Util import *
from core.target_vehicles_generation_protocols import *

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("No environment variable SUMO_HOME!")

import traci
import sumolib
from controller.RouteController import *

"""
SUMO Selfless Traffic Routing (STR) Testbed
"""

MAX_SIMULATION_STEPS = 2000

# TODO: decide which file to put these in. Right now they're also defined in RouteController!!
STRAIGHT = "s"
TURN_AROUND = "t"
LEFT = "l"
RIGHT = "r"
SLIGHT_LEFT = "L"
SLIGHT_RIGHT = "R"

class StrSumo:
    def __init__(self, route_controller, connection_info, controlled_vehicles):
        """
        :param route_controller: object that implements the scheduling algorithm for controlled vehicles
        :param connection_info: object that includes the map information
        :param controlled_vehicles: a dictionary that includes the vehicles under control
        """
        self.missed = []
        self.finished = 0
        self.max_travel_time = 0
        self.direction_choices = [STRAIGHT, TURN_AROUND, SLIGHT_RIGHT, RIGHT, SLIGHT_LEFT, LEFT]
        self.connection_info = connection_info
        self.route_controller = route_controller
        self.controlled_vehicles =  controlled_vehicles # dictionary of Vehicles by id

    def run(self):
        """
        Runs the SUMO simulation
        At each time-step, cars that have moved edges make a decision based on user-supplied scheduler algorithm
        Decisions are enforced in SUMO by setting the destination of the vehicle to the result of the
        :returns: total time, number of cars that reached their destination, number of deadlines missed
        """
        total_time = 0
        end_number = 0
        deadlines_missed = []
        deadline_overtime = 0

        step = 0
        vehicles_to_direct = [] #  the batch of controlled vehicles passed to make_decisions()
        

        try:
            while traci.simulation.getMinExpectedNumber() > 0:
                vehicle_ids = set(traci.vehicle.getIDList())
                #print("souless:",vehicle_ids)
                # store edge vehicle counts in connection_info.edge_vehicle_count
                self.get_edge_vehicle_counts()
                #initialize vehicles to be directed
                vehicles_to_direct = []
                # iterate through vehicles currently in simulation
                for vehicle_id in vehicle_ids:

                    
                    

                    # handle newly arrived controlled vehicles
                    if vehicle_id not in vehicle_IDs_in_simulation and vehicle_id in self.controlled_vehicles:
                        #print("what is the problem?")
                        vehicle_IDs_in_simulation.append(vehicle_id)
                        traci.vehicle.setColor(vehicle_id, (255, 0, 0)) # set color so we can visually track controlled vehicles
                        self.controlled_vehicles[vehicle_id].start_time = float(step)#Use the detected release time as start time

                    if vehicle_id in self.controlled_vehicles.keys():
                        current_edge = traci.vehicle.getRoadID(vehicle_id)

                        if current_edge not in self.connection_info.edge_index_dict.keys():
                            continue
                        elif current_edge == self.controlled_vehicles[vehicle_id].destination:
                            continue

                        
                        if current_edge != self.controlled_vehicles[vehicle_id].current_edge:
                            self.controlled_vehicles[vehicle_id].current_edge = current_edge
                            self.controlled_vehicles[vehicle_id].current_speed = traci.vehicle.getSpeed(vehicle_id)
                            vehicles_to_direct.append(self.controlled_vehicles[vehicle_id])
                
                vehicle_decisions_by_id = self.route_controller.make_decisions(vehicles_to_direct, self.connection_info)
                
                #{id: direction} {4: R}
                for vehicle_id, local_target_edge in vehicle_decisions_by_id.items():
                   
                    if vehicle_id in traci.vehicle.getIDList():
                        # UPDATE VEHICLE DIRECTION
                        traci.vehicle.changeTarget(vehicle_id, local_target_edge)
                        
                        self.controlled_vehicles[vehicle_id].local_destination = local_target_edge
                        
                arrived_at_destination = traci.simulation.getArrivedIDList()

                for vehicle_id in arrived_at_destination:
                    if vehicle_id in self.controlled_vehicles:
                        vehicle_IDs_in_simulation.remove(vehicle_id)
                        #print the raw result out to the terminal
                        arrived_at_destination = False
                        if self.controlled_vehicles[vehicle_id].local_destination == self.controlled_vehicles[vehicle_id].destination:
                            arrived_at_destination = True
                            self.finished+=1
                        
                            
                        time_span = step - self.controlled_vehicles[vehicle_id].start_time
                        if time_span > self.max_travel_time:
                            self.max_travel_time =time_span 
                        total_time += time_span
                        miss = False
                        dl = self.controlled_vehicles[vehicle_id].deadline
                        ST = self.controlled_vehicles[vehicle_id].start_time
                        if step > self.controlled_vehicles[vehicle_id].deadline:
                            deadlines_missed.append(vehicle_id)
                            deadline_overtime += step - dl
                            miss = True
                        end_number += 1
                        

                traci.simulationStep()
                step += 1

                if step > MAX_SIMULATION_STEPS:
                    print('Ending due to timeout.')
                    break

        except ValueError as err:
            print('Exception caught.')
            print(err)
        global Num_vehicles_route_change_realtime
        if Num_vehicles_route_change_realtime > 0:
            print("Vehicles that have switched",Num_vehicles_route_change_realtime)
        num_deadlines_missed = len(deadlines_missed)
        #print("vehicles with deadline misses:",self.missed)
        return step, total_time, end_number, num_deadlines_missed, deadline_overtime, self.max_travel_time,self.finished

    def get_edge_vehicle_counts(self):
        for edge in self.connection_info.edge_list:
            self.connection_info.edge_vehicle_count[edge] = traci.edge.getLastStepVehicleNumber(edge)

