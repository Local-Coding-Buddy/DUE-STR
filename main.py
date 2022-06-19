'''
This test file needs the following files:
STR_SUMO.py, RouteController.py, Util.py, test.net.xml, test.rou.xml, myconfig.sumocfg and corresponding SUMO libraries.
'''
print("what")
from core.STR_SUMO import StrSumo
from core.STR_SUMO_Meta_collector_plus import StrSumo_Meta_collector_plus
import os
import sys
import copy
import random
import time
from lxml import etree
from xml.dom.minidom import parse, parseString
from core.Util import *
from controller.RouteController import *
from controller.DijkstraController import DijkstraPolicy
import core.Run_id
from controller.New_Tom_Reader_Controller import NewPolicy_Reader
from controller.STRBE_Controller import NewPolicy_Reader_STRBE
from controller.STRBE_Collector import STRBE_Collector
from Route_setup.Make_Trips_file import *

from core.target_vehicles_generation_protocols import *
#from history.Historical_Prediction import *
from datetime import datetime
from core.Time_estimation.Timer_estimator_new import Time_predictors

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("No environment variable SUMO_HOME!")

from sumolib import checkBinary
import traci

    
# net_map = root.xpath('./input/net-file/@value')[0]
Round_name_base ="Eric"#"analyze"#'for_paper'##"analyze-two-"#'dua_iterate_analysis'#'for_paper'##"tests"#
sufix = 0
round_min = 0
round_max = 1

even = False # if true then we will send control_v number of vehicles down each start edge

control_v_min = 300 #number of controlled cars
control_v_max = 500
uncontrol_v = 0 
start_edge = "-2"#-1 # for random generation
end_edge = "-2"# for random generation
pattern_var = 3
refine = True # use duaiterate.py
#already_routed = True
Using_reading = True# if false then use target_vehicles_generation and if false with already routed then back to 

#how the original code work.
trips = {}

start_edges = ["597602756#0","-260998164#3","105681660#12","597602753#0","422688973","918280985","422685671#2",\
"318908683#0"]
end_edges = ["-597602756#0","260998164#0","-105681660#20","-597602753#1","422688976#1","918280984",\
"15374898#0-AddedOffRampEdge","-318908683#1"]
edge_dependencies = {"597602756#0":["-597602756#0"],"-260998164#3":["260998164#0"],"105681660#12":["-105681660#20"],\
"597602753#0":["-597602753#1"],"422688973":["422688976#1","918280984"],"918280985":["422688976#1","918280984"],\
"422685671#2":["15374898#0-AddedOffRampEdge","-318908683#1"],"318908683#0":["15374898#0-AddedOffRampEdge","-318908683#1"]}


trip_file_name = "static_trip.rou.xml"
file_name = "str_sumo.rou.xml" #should be from duarouter
file_name2 = "str_sumo_099.rou.xml"#"static2_099.rou.xml"#should be from duaIterate 
file_name3 = "One-shot.rou.xml"
num_iterations = 100


# use vehicle generation protocols to generate vehicle list
# not the primary method of use at this point
def get_controlled_vehicles(route_filename, connection_info, \
    num_controlled_vehicles=control_v_max, num_uncontrolled_vehicles=uncontrol_v, pattern = pattern_var):
    '''
    :param @route_filename <str>: the name of the route file to generate
    :param @connection_info <object>: an object that includes the map inforamtion q
    :param @num_controlled_vehicles <int>: the number of vehicles controlled by the route controller
    :param @num_uncontrolled_vehicles <int>: the number of vehicles not controlled by the route controller
    :param @pattern <int>: one of four possible patterns. FORMAT:
            -- CASES BEGIN --
                #1. one start point, one destination for all target vehicles
                #2. ranged start point, one destination for all target vehicles
                #3. ranged start points, ranged destination for all target vehicles
                #4. starting_edges and ending_edges considered random trips similar to pattern 3
            -- CASES ENDS --
    '''
    vehicle_dict = {}
    print(connection_info.net_filename)
    if pattern_var != 4:
        start_and_end_edges = []
    generator = target_vehicles_generator(connection_info.net_filename)
    
    # list of target vehicles is returned by generate_vehicles
    vehicle_list = generator.generate_vehicles(num_controlled_vehicles, num_uncontrolled_vehicles, \
        pattern, route_filename, connection_info.net_filename,start_edge,end_edge)#, \
            #edge_dependencies)#gneE0")#"15420484#3") #set start edge -1 for random

    for vehicle in vehicle_list:
        vehicle_dict[str(vehicle.vehicle_id)] = vehicle
    
    return vehicle_dict

def test_dijkstra_policy(vehicles):
    print("Testing Dijkstra's Algorithm Route Controller")
    scheduler = DijkstraPolicy(init_connection_info)
    run_simulation(scheduler, vehicles)


def test_new_policy_Meta_Plus(vehicles,fiuela): #reader using meta SUMO 
    print("Testing New Algorithm meta data collection Reader Route Controller")
    scheduler = NewPolicy_Reader(init_connection_info,fiuela,Round_name)#going to add file name as another argument
    run_simulation_Meta_Plus(scheduler, vehicles)
    del scheduler

def test_new_policy_STRBE_Meta_Plus(vehicles,time_difference,predictors):# STRBE Controller
    print("Testing New Algorithm meta data collection Reader Route Controller")
    scheduler = NewPolicy_Reader_STRBE(init_connection_info,file_name,file_name2,Round_name,vehicles,time_difference,predictors)#going to add file name as another argument
    run_simulation_Meta_Plus(scheduler, vehicles)
    del scheduler

def test_new_policy_STRBE_collector_Meta_Plus(vehicles,time_difference): # STRBE collector
    print("Testing New Algorithm meta data collection Reader Route Controller")
    scheduler = STRBE_Collector(init_connection_info,file_name,file_name2,vehicles,time_difference)#going to add file name as another argument
    run_simulation_Meta_Plus(scheduler, vehicles)
    del scheduler





def run_simulation(scheduler, vehicles):

    simulation = StrSumo(scheduler, init_connection_info, vehicles)

    traci.start([sumo_binary, "-c", "./configurations/myconfig.sumocfg", \
                 "--tripinfo-output", "./configurations/trips.trips.xml", \
                 "--fcd-output", "./configurations/testTrace.xml","--quit-on-end"])

    step, total_time, end_number, deadlines_missed,deadline_ogretime,max_time,fin = simulation.run()

    if end_number != 0:
        print("Total timespan: {}, Average timespan: {}, Max Travel Time {}, total vehicle number: {}, Total Vehicles finished: {}\
            ".format(step,str(total_time/end_number),max_time,str(end_number),str(fin)))
        if deadlines_missed == 0:
            print(str(deadlines_missed) + ' deadlines missed.' + " average time over deadline:"+ str(0))
        else: 
            print(str(deadlines_missed) + ' deadlines missed.' + " average time over deadline:"+str(deadline_ogretime/deadlines_missed+1))

    print("TRACI CLOSEING")
    traci.close()


def run_simulation_Meta_Plus(scheduler, vehicles): # used in STRBE non-real time

    simulation = StrSumo_Meta_collector_plus(scheduler, init_connection_info, vehicles,Round_name_base+str(sufix))
    #print(Round_name_base+str(sufix))
    traci.start([sumo_binary, "-c", "./configurations/myconfig.sumocfg", \
                 "--tripinfo-output", "./configurations/trips.trips.xml", \
                 "--fcd-output", "./configurations/testTrace.xml","--quit-on-end"])

    step, total_time, end_number, deadlines_missed,deadline_ogretime,max_time,fin = simulation.run()

    if end_number != 0:
        print("Total timespan: {}, Average timespan: {}, Max Travel Time {}, total vehicle number: {}, Total Vehicles finished: {}\
            ".format(step,str(total_time/end_number),max_time,str(end_number),str(fin)))
        if deadlines_missed == 0:
            print(str(deadlines_missed) + ' deadlines missed.' + " average time over deadline:"+ str(0))
        else: 
            print(str(deadlines_missed) + ' deadlines missed.' + " average time over deadline:"+str(deadline_ogretime/deadlines_missed+1))

    print("TRACI CLOSEING")
    traci.close()

if __name__ == "__main__":
    #print("hello?")
    #global already_routed
    dom = parse("./configurations/myconfig.sumocfg")
    print("AAAAAAAAAAAAAAAAAAAAAAA")
    net_file_node = dom.getElementsByTagName('net-file')
    net_file_attr = net_file_node[0].attributes

    net_file = net_file_attr['value'].nodeValue

    mapsqm = str(net_file).find("/maps/")
    if mapsqm != -1: 
        slash_pos = net_file.find("/",mapsqm+1)
        net_file = net_file[slash_pos+1:]
        #I want to move all the map files into configurations/maps but the config file wants me to specifify /maps/{map_name}
        #I would just remove the /maps/ from below but I want to get just the file name meaning I would have to do this anyways
    

    init_connection_info = ConnectionInfo("./configurations/maps/"+net_file)
    print("net_file:",net_file)
    route_file_node = dom.getElementsByTagName('route-files')
    route_file_attr = route_file_node[0].attributes
    route_file = "./configurations/maps/"+route_file_attr['value'].nodeValue

    
    #global Round_name
    #this is where a loop may occur in case of increaseing number of vehicles
    
    predictors = Time_predictors(net_file)
    sumo_binary = checkBinary('sumo')#'sumo-gui' or # 'sumo'
        
    for increment in range(round_min,round_max): #,control_v,
        sufix = increment
        core.Run_id.Round_sufix =Round_name_base+str(sufix)
        print("NUMBA ",increment)
        Round_name=Round_name_base
        Round_name = net_file +"-"+Round_name+"-"+str(increment)
        #debug
        print("Round name:",Round_name)
        print("connection info file = ",init_connection_info)
        print("Net_file = ", net_file)
        print("route file = ",route_file)


        if Using_reading == False:  
            # using the code in a more traditional sense to how it was written by guangli and pavan.
            #that is to say that the routes are updated after a vehicle gets on a new edge and is met with a new
            #edge node pair
           
            num_v =control_v_max#= random.randint(control_v/2,control_v)
            vehicles = get_controlled_vehicles(route_file, init_connection_info,num_v, uncontrol_v,3)
        else:
            
            
            if os.path.isdir("./configurations/rounds/"+Round_name):
                
                vehicles = read_vehicle_file(trip_file_name,Round_name,net_file)
            else:
                num_v = random.randint(control_v_min, control_v_max)#control_v
                vehicles = Create_trips(start_edges,end_edges,edge_dependencies,num_v,uncontrol_v,Round_name,even,net_file,refine,num_iterations)
        core.Run_id.Controller_version = "N/A"
            
        

        tom_vehicles = copy.deepcopy(vehicles)
        #iterator = 0
        tom_vehicles2 = copy.deepcopy(tom_vehicles)

     

        #DUE-STR
        # core.Run_id.run_id = "STRBE_DUE-STR"
        # test_new_policy_STRBE_Meta_Plus(tom_vehicles2,100000,predictors)
        # tom_vehicles2 = copy.deepcopy(tom_vehicles)

        #for testing dijkstras 
        # core.Run_id.run_id = "str_sumo.rou.xml"
        # test_new_policy_Meta_Plus(tom_vehicles2,file_name)
        # tom_vehicles2 = copy.deepcopy(tom_vehicles)


        # core.Run_id.run_id = "static2_099.rou.xml"
        # test_new_policy_Meta_Plus(tom_vehicles2,file_name2)
        # tom_vehicles2 = copy.deepcopy(tom_vehicles)

        # core.Run_id.run_id = "STRBE_Collector"
        # test_new_policy_STRBE_collector_Meta_Plus(tom_vehicles2,file_name2)
        # tom_vehicles2 = copy.deepcopy(tom_vehicles)

    if os.path.isdir("./0"):
        for x in range(0,num_iterations):
            shutil.rmtree('./'+str(x))
            #if this errors out due to there not being an iteration present it's not really a problem.
            #if there are more duaiterate folders than num_iterations then it is a problem.


    