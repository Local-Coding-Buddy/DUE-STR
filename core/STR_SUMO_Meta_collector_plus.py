import os
import sys
import optparse
#from core.Estimation import *
from xml.dom.minidom import parse, parseString
from core.Util import *
from core.target_vehicles_generation_protocols import *
import core.Run_id

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("No environment variable SUMO_HOME!")

import traci
import sumolib
from controller.RouteController import *

from datetime import datetime
from lxml import etree
from history.data_recorder import *

"""
SUMO Selfless Traffic Routing (STR) Testbed
"""

MAX_SIMULATION_STEPS = 2500

# TODO: decide which file to put these in. Right now they're also defined in RouteController!!
STRAIGHT = "s"
TURN_AROUND = "t"
LEFT = "l"
RIGHT = "r"
SLIGHT_LEFT = "L"
SLIGHT_RIGHT = "R"

class StrSumo_Meta_collector_plus:
    def __init__(self, route_controller, connection_info, controlled_vehicles,Round_name):
        """
        :param route_controller: object that implements the scheduling algorithm for controlled vehicles
        :param connection_info: object that includes the map information
        :param controlled_vehicles: a dictionary that includes the vehicles under control
        """
        self.finished = 0
        self.max_travel_time = 0
        self.direction_choices = [STRAIGHT, TURN_AROUND, SLIGHT_RIGHT, RIGHT, SLIGHT_LEFT, LEFT]
        self.connection_info = connection_info
        self.route_controller = route_controller
        self.controlled_vehicles =  controlled_vehicles # dictionary of Vehicles by id
        self.fPaper_pushing = {}
        self.trips = {}
        self.Round_name = Round_name
        self.vehicles_that_missed=[]
        self.trips = core.Run_id.trips
        self.fPaper_pushing =core.Run_id.route_information
        self.vehicle_options_num = core.Run_id.vehicle_num_options
        self.missed_by_options=[0,0,0,0,0] #keeping track of vehicles that missed by # of shared edges
        self.total_by_option=[0,0,0,0,0] #keepint track of the total number of vehicles by # of shared edges
        

    def run(self):
        """
        Runs the SUMO simulation
        At each time-step, cars that have moved edges make a decision based on user-supplied scheduler algorithm
        Decisions are enforced in SUMO by setting the destination of the vehicle to the result of the
        :returns: total time, number of cars that reached their destination, number of deadlines missed
        """
        run_id =core.Run_id.run_id
      
        
        total_time = 0
        end_number = 0
        deadlines_missed = []
        deadline_overtime = 0

        step = 0
        vehicles_to_direct = [] #  the batch of controlled vehicles passed to make_decisions()
        vehicle_IDs_in_simulation = []

        try:
            while traci.simulation.getMinExpectedNumber() > 0:
                vehicle_ids = set(traci.vehicle.getIDList())
               
                # store edge vehicle counts in connection_info.edge_vehicle_count
                self.get_edge_vehicle_counts()
                #initialize vehicles to be directed
                vehicles_to_direct = []
                # iterate through vehicles currently in simulation
                for vehicle_id in vehicle_ids:
                    

                    #should not be added because there is no corresponding -1, this makes edge_vehicle_count becomes the total number of vehicles that used to be on this edge.
                    #self.connection_info.edge_vehicle_count[traci.vehicle.getRoadID(vehicle_id)] += 1
                    
                    

                    # handle newly arrived controlled vehicles
                    if vehicle_id not in vehicle_IDs_in_simulation and vehicle_id in self.controlled_vehicles:
                        vehicle_IDs_in_simulation.append(vehicle_id)
                        traci.vehicle.setColor(vehicle_id, (255, 0, 0)) # set color so we can visually track controlled vehicles
                        self.controlled_vehicles[vehicle_id].start_time = float(step)#Use the detected release time as start time
                        

                    if vehicle_id in self.controlled_vehicles.keys():
                        current_edge = traci.vehicle.getRoadID(vehicle_id)

                        if current_edge not in self.connection_info.edge_index_dict.keys():
                            continue
                        elif current_edge == self.controlled_vehicles[vehicle_id].destination:
                            continue

                        #print("{} now on: {}, records on {}; {} ".format(vehicle_id, current_edge, self.controlled_vehicles[vehicle_id].current_edge, current_edge!=self.controlled_vehicles[vehicle_id].current_edge))
                        if current_edge != self.controlled_vehicles[vehicle_id].current_edge:
                            self.controlled_vehicles[vehicle_id].current_edge = current_edge
                            self.controlled_vehicles[vehicle_id].current_speed = traci.vehicle.getSpeed(vehicle_id)
                            vehicles_to_direct.append(self.controlled_vehicles[vehicle_id])
                #print(len(vehicles_to_direct))
                vehicle_decisions_by_id = self.route_controller.make_decisions(vehicles_to_direct, self.connection_info)
                
                for vehicle_id, local_target_edge in vehicle_decisions_by_id.items():
                
                    if vehicle_id in traci.vehicle.getIDList():
                        traci.vehicle.changeTarget(vehicle_id, local_target_edge)
                        self.controlled_vehicles[vehicle_id].local_destination = local_target_edge

                arrived_at_destination = traci.simulation.getArrivedIDList()

                for vehicle_id in arrived_at_destination:
                    if vehicle_id in self.controlled_vehicles:
                        #print the raw result out to the terminal
                        arrived_at_destination = False
                        if self.controlled_vehicles[vehicle_id].local_destination == self.controlled_vehicles[vehicle_id].destination:
                            arrived_at_destination = True
                            self.finished +=1
                            self.controlled_vehicles[vehicle_id].finished = True
                        time_span = step - self.controlled_vehicles[vehicle_id].start_time
                        if time_span > self.max_travel_time:
                            self.max_travel_time =time_span 
                        #pushing[vehicle_id].append(time_span)
                        self.fPaper_pushing[vehicle_id].append(time_span)
                        total_time += time_span
                        miss = False

                        if vehicle_id in self.vehicle_options_num.keys():
                            with_options = self.vehicle_options_num[vehicle_id]
                            #self.total_by_options
                            if with_options > 4:
                                    with_options = 4
                            self.total_by_option[with_options] +=1

                        if step > self.controlled_vehicles[vehicle_id].deadline:
                            deadlines_missed.append(vehicle_id)
                            deadline_overtime += step - self.controlled_vehicles[vehicle_id].deadline
                            miss = True
                            self.vehicles_that_missed.append(vehicle_id)
                            if vehicle_id in self.vehicle_options_num.keys():
                                self.missed_by_options[with_options] +=1
                        end_number += 1
                        if arrived_at_destination == False:
                            print("destination:",self.controlled_vehicles[vehicle_id].destination,\
                                "did not arrive now at:",self.controlled_vehicles[vehicle_id].current_edge)
                        print("Vehicle {} reaches the destination: {}, timespan: {}, deadline missed: {}"\
                            .format(vehicle_id, arrived_at_destination, time_span, miss))
                        #if not arrived_at_destination:
                            #print("{} - {}".format(self.controlled_vehicles[vehicle_id].local_destination, self.controlled_vehicles[vehicle_id].destination))

                traci.simulationStep()
                step += 1

                if step > MAX_SIMULATION_STEPS:
                    print('Ending due to timeout.')
                    break

        except ValueError as err:
            print('Exception caught.')
            print(err)

        num_deadlines_missed = len(deadlines_missed)

       
        csv2Data('./History/With_start_time.csv')
        temp = []
        overall= []
        for vid in self.fPaper_pushing.keys():
            print("uh what:",self.fPaper_pushing[vid])
            temp=self.fPaper_pushing[vid]
            temp.insert(0,self.trips[vid])
            temp.insert(0,self.controlled_vehicles[vid].destination)
            temp.insert(0,str(self.controlled_vehicles[vid].start_edge))
            temp.insert(0,vid)
            temp.insert(0,run_id)
            temp.insert(0,self.Round_name)
            
            overall.append(temp)
       # print("data selected:",overall)
        data2Csv_general(overall,'./History/Travel_time_prediction_factor.csv')

        csv2Data('./History/Vehicle_switch_overall_performance.csv')
        overall= []
        temp = []
        temp.extend(self.missed_by_options)
        temp.extend(self.total_by_option)
        
        temp.insert(0,core.Run_id.Num_switched)
        if num_deadlines_missed > 0:
            temp.insert(0,deadline_overtime/num_deadlines_missed)
        else:
            temp.insert(0,0)
        temp.insert(0,num_deadlines_missed)
        temp.insert(0,self.finished)
        temp.insert(0,end_number)
        temp.insert(0,self.max_travel_time)
        if end_number > 0:
            temp.insert(0,str(total_time/end_number))
        else:
            temp.insert(0,0)
        temp.insert(0,step)
        temp.insert(0,core.Run_id.Controller_version)
        temp.insert(0,str(run_id))
        temp.insert(0,str(self.Round_name))
        
        overall.append(temp)
        data2Csv_general(overall,'./History/Vehicle_switch_overall_performance.csv')
        core.Run_id.Num_switched = 0
        print("vehicles that missed:",self.vehicles_that_missed)
        return step, total_time, end_number, num_deadlines_missed, deadline_overtime, self.max_travel_time,self.finished
        
    def get_edge_vehicle_counts(self):
        for edge in self.connection_info.edge_list:
            self.connection_info.edge_vehicle_count[edge] = traci.edge.getLastStepVehicleNumber(edge)

