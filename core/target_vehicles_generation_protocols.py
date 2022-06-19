"""
    This file contains the function protocols
    for the generation of target vehicles.
    
    Directions: straight (s), turnaround (t), left (L), slightly left (l),
                right (R), slightly right (r).
"""


import random
import os
import sys
import xml.dom.minidom 
from core import Util
from core import network_map_data_structures
from xml.dom import minidom
import pandas as pd


# CHECK VERSION INFORMATION AND SET UP VERSION REFERENCE VARIABLES:
CURRENT_PY_VERSION = None
PY_VERSION3 = 3
PY_VERSION2 = 2.7
if sys.version_info.major == 3:
    print("Python 3...")    # TODO: After testing, comment out this line, if appropriate.
    CURRENT_PY_VERSION = PY_VERSION3
elif sys.version_info.major == 2 and sys.version_info.minor == 7:
    print("Python 2.7...")  # TODO: After testing, comment out this line, if appropriate.
    CURRENT_PY_VERSION = PY_VERSION2
else:
    sys.exit("This python version is outdated for the project! Upgrade to python 2.7 or higher!")

# !!! Code borrowed from Guangli !!!
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("No environment variable SUMO_HOME!")

from sumolib import checkBinary
import traci
import sumolib





class target_vehicles_generator:
    """
        static var @target_vehicles_output_dict <dict>: A dictionary that records the number
                                                    of target-vehicles generated for
                                                    each target-xml output file. FORMAT:
                                                    {
                                                        target_xml_file1 <str> : count1 <int>,
                                                        target_xml_file2 <str> : count2 <int>,
                                                        target_xml_file3 <str> : count3 <int>,
                                                        ...
                                                    }
    """
    target_vehicles_output_dict = {}
    VEHICLES_INFO = "vehicles info"
    __ERROR_MESSAGE__ = "error message"
    

    def __init__(self, net_file):
        """
            :param @net_file<str>: The name of the network file (in the form of XML)
        """
        self.length_dict = None
        self.out_dict = None
        self.index_dict = None
        self.edge_list = None
        self.net = network_map_data_structures.getNetInfo(net_file)
        [self.length_dict, self.out_dict, self.index_dict, self.edge_list] = network_map_data_structures.getEdgesInfo(self.net)
        #print("all edges 2:",self.edge_list)
        self.__current_target_xml_file__ = ""



    def generate_target_vehicles(self, num_vehicles, target_xml_file, pattern=None):
        """
            param @num_vehicles <int>: the number of target-vehicles desired.
            param @target_xml_file <str>: name of the target xml file.
            param @pattern <tuple>: one of four possible patterns. FORMAT:
            -- CASES BEGIN --
                1. (one_start_point <sumolib.net.edge.Edge>, one_destination <sumolib.net.edge.Edge>)
                2. (random_ranged_start-points <list<sumolib.net.edge.Edge>>, one_destination <sumolib.net.edge.Edge>)
                3. (random_ranged_start-points <list<sumolib.net.edge.Edge>>, random_ranged_destinations <list<sumolib.net.edge.Edge>>)
                4. None
            -- CASES END --.
            
            var @vehicles_info <list>: the target-vehicle information to return from this
                                       function.
            
            The function generates target-vehicle information based on the desired
            number, output xml file, generation pattern specified by the input
            parameters. If the value passed to pattern is None, or is not specified,
            the function will interpret it as a generation pattern of @num_vehicles pairs
            of random start-points and random destinations from the whole of
            @target_vehicles_generator.edge_list. The start point(s) are written into
            the output xml file, and the list of the generated vehicles' information
            (in tuples) is returned.
        """
        
        vehicles_info = []
        self.__current_target_xml_file__ = target_xml_file
        if target_xml_file not in target_vehicles_generator.target_vehicles_output_dict:
            target_vehicles_generator.target_vehicles_output_dict[target_xml_file] = 0
        
        __error_message__ = None
        # Call appropriate member functions according to the pattern specified:
        if type(pattern) is tuple:
            if type(pattern[0]) is sumolib.net.edge.Edge:
                if type(pattern[1]) is sumolib.net.edge.Edge:
                    # -- CASE 1. --
                    vehicles_info = None
                    while vehicles_info is None:
                        vehicles_info = self.generate_with_one_start_one_dest(num_vehicles, pattern[0], pattern[1])
                else:
                    __error_message__ = "Invalid pattern for generating random vehicles: The 1st element of " + str(pattern) + " is not an instance of sumolib.net.edge.Edge!"
            elif type(pattern[0]) is list:
                if type(pattern[1]) is sumolib.net.edge.Edge:
                    # -- CASE 2. --
                    vehicles_info = self.generate_with_ranged_starts_one_dest(num_vehicles, pattern[0], pattern[1])
                elif type(pattern[1]) is list:
                    # -- CASE 3. --
                    #print("oy I'm buying here")
                    vehicles_info = self.generate_with_ranged_starts_ranged_dests(num_vehicles, pattern[0], pattern[1])
                else:
                    __error_message__ = "Invalid pattern for generating random vehicles: The 1st element of " + str(pattern) + " is not an instance of sumolib.net.edge.Edge or a list of such instances!"
            else:
                __error_message__ = "Invalid pattern for generating random vehicles: The 0th element of " + str(pattern) + " is not an instance of sumolib.net.edge.Edge or a list of such instances!"
        elif pattern == None:
            # -- Case 4. --
            #print("oy I'm dying here")
            vehicles_info = self.generate_with_rand_starts_rand_dests(num_vehicles)
        else:
            __error_message__ = "Invalid pattern for generating random vehicles: " + str(pattern) + " is not a tuple!"
        
        # TODO: Write the start-point(s) of the vehicle information into the output xml file (Why?):
        
        # Update the generated vehicle count:
        if __error_message__ != None:
            target_vehicles_generator.target_vehicles_output_dict[target_xml_file] += num_vehicles
        
        # TODO: The tuple elements for the information of a vehicle are to be determined.
        return {
            target_vehicles_generator.VEHICLES_INFO: vehicles_info,
            target_vehicles_generator.__ERROR_MESSAGE__: __error_message__
        }
    
    
    def generate_with_one_start_one_dest(self, num_vehicles, start_point, destination):
        """
            param @num_vehicles <int>: the number of target-vehicles desired.
            param @start_point <sumolib.net.edge.Edge>: the start-point of each target-vehicle.
            param @destination <sumolib.net.edge.Edge>: the destination of each target-vehicle.
            
            var @vehicles_info <list>: the target-vehicle information to return from this
                                       function.
            
            Function to generate @num_vehicles sets of target-vehicle information,
            stored in @vehicles_info. Each target-vehicle is generated with the start-point
            @start_point and the destination @destination. If the returned value is None,
            that means there is no path from @start_point to @destination.
            
            IMPORTANT: This function should only be called in contexts that assign a file
            name (type <str>) to @target_vehicles_generator.__current_target_xml_file__.
            If the variable @target_vehicles_generator.__current_target_xml_file__ is not
            assigned or updated before the function call, an error caused by the file name
            not being found may occur, or the user may be writing target-vehicle information
            to the wrong target-xml output file.
        """
        vehicles_info = []
        # TODO: Generate vehicle ID's:
        current_ID = target_vehicles_generator.target_vehicles_output_dict[self.__current_target_xml_file__]
        end_ID = current_ID + num_vehicles
        if not validate_path(self.net, start_point, destination):
            
            ### UNCOMMENT TO DEBUG ###
            #print("No path from", start_point.getID(), "to", destination.getID())
            
            return None
        while current_ID < end_ID:
            vehicles_info.append( (current_ID, (start_point, destination), True) )
            current_ID += 1
        
        # TODO: The tuple elements for the information of a vehicle are to be determined.
        return vehicles_info
        
        
    def generate_with_ranged_starts_one_dest(self, num_vehicles, start_point_lst, destination):
        """
            param @num_vehicles <int>: the number of target-vehicles desired.
            param @start_point_lst <list>: a list of start-points, from which one for each
                                           target-vehicle is randomly selected.
            param @destination <sumolib.net.edge.Edge>: the destination of each target-vehicle.
            
            var @vehicles_info <list>: the target-vehicle information to return from this
                                       function.
            
            Function to generate @num_vehicles sets of target-vehicle information,
            stored in @vehicles_info. Each target-vehicle is generated with a randomly
            selected start-point from @start_point_lst and with the destination @destination.
            
            IMPORTANT: This function should only be called in contexts that assign a file
            name (type <str>) to @target_vehicles_generator.__current_target_xml_file__.
            If the variable @target_vehicles_generator.__current_target_xml_file__ is not
            assigned or updated before the function call, an error caused by the file name
            not being found may occur, or the user may be writing target-vehicle information
            to the wrong target-xml output file.
        """
        vehicles_info = []
        # Generate @num_vehicle start-points using a random choice function:
        assigned_start_point_lst = None
        if CURRENT_PY_VERSION == PY_VERSION3:
            assigned_start_point_lst = random.choices(start_point_lst, k=num_vehicles)
        else: # CURRENT_PY_VERSION == PY_VERSION2
            assigned_start_point_lst = __random_choices_with_rp__(start_point_lst, num_vehicles)
        
        # TODO: Generate vehicle ID's:
        current_ID = target_vehicles_generator.target_vehicles_output_dict[self.__current_target_xml_file__]
        i = 0
        while i < num_vehicles:
            valid_pair = True
            if not validate_path(self.net, assigned_start_point_lst[i], destination):
                valid_pair = False
                
                ### UNCOMMENT TO DEBUG ###
                #print("No path from", assigned_start_point_lst[i].getID(), "to", destination.getID())
                
                assigned_start_point_lst[i] = random.choice(start_point_lst)
                continue
            
            vehicles_info.append( (current_ID + i, (assigned_start_point_lst[i], destination), valid_pair) )
            i += 1
        
        # TODO: The tuple elements for the information of a vehicle are to be determined.
        return vehicles_info


    def generate_with_ranged_starts_ranged_dests(self, num_vehicles, start_point_lst, destination_lst):
        """
            param @num_vehicles <int>: the number of target-vehicles desired.
            param @start_point_lst <list>: a list of start-points, from which one for each
                                           target-vehicle is randomly selected.
            param @destination_lst <list>: a list of the destinations, from which one for each
                                           target-vehicle is randomly selected.
            
            var @vehicles_info <list>: the target-vehicle information to return from this
                                       function.
            
            Function to generate @num_vehicles sets of target-vehicle information,
            stored in @vehicles_info. Each target-vehicle is generated with a randomly
            selected start-point from @start_point_lst and with a randomly selected
            destination from @destination_lst.
            
            IMPORTANT: This function should only be called in contexts that assign a file
            name (type <str>) to @target_vehicles_generator.__current_target_xml_file__.
            If the variable @target_vehicles_generator.__current_target_xml_file__ is not
            assigned or updated before the function call, an error caused by the file name
            not being found may occur, or the user may be writing target-vehicle information
            to the wrong target-xml output file.
        """
        vehicles_info = []
        # Generate @num_vehicle start-points and destinations using a random choice function:
        assigned_start_point_lst = None
        assigned_destination_lst = None
        if CURRENT_PY_VERSION == PY_VERSION3:
            assigned_start_point_lst = random.choices(start_point_lst, k=num_vehicles)
            assigned_destination_lst = random.choices(destination_lst, k=num_vehicles)
        else:
            assigned_start_point_lst = __random_choices_with_rp__(start_point_lst, num_vehicles)
            assigned_destination_lst = __random_choices_with_rp__(destination_lst, num_vehicles)
        
        # TODO: Generate vehicle ID's:
        current_ID = target_vehicles_generator.target_vehicles_output_dict[self.__current_target_xml_file__]
        i = 0
        while i < num_vehicles:
            valid_pair = True
            if not validate_path(self.net, assigned_start_point_lst[i], assigned_destination_lst[i]):
                valid_pair = False
                
                ### UNCOMMENT TO DEBUG ###
                #print("No path from", assigned_start_point_lst[i].getID(), "to", assigned_destination_lst[i].getID())
                
                assigned_start_point_lst[i] = random.choice(start_point_lst)
                assigned_destination_lst[i] = random.choice(destination_lst)
                continue
            
            vehicles_info.append( (current_ID + i, (assigned_start_point_lst[i], assigned_destination_lst[i]), valid_pair) )
            i += 1
        
        # TODO: The tuple elements for the information of a vehicle are to be determined.
        return vehicles_info
        
        
    def generate_with_rand_starts_rand_dests(self, num_vehicles):
        """
            param @num_vehicles <int>: the number of target-vehicles desired.
            
            var @vehicles_info <list>: the target-vehicle information to return from this
                                       function.
            
            Function to generate @num_vehicles sets of target-vehicle information,
            stored in @vehicles_info. Each target-vehicle is generated with a randomly
            selected start-point from @target_vehicles_generator.edge_list and with a
            randomly selected destination from @target_vehicles_generator.edge_list.
            
            IMPORTANT: This function should only be called in contexts that assign a file
            name (type <str>) to @target_vehicles_generator.__current_target_xml_file__.
            If the variable @target_vehicles_generator.__current_target_xml_file__ is not
            assigned or updated before the function call, an error caused by the file name
            not being found may occur, or the user may be writing target-vehicle information
            to the wrong target-xml output file.
        """
        vehicles_info = []
        
        # Generate @num_vehicle tuple-pairs of start_points and destinations:
        # TODO: Generate vehicle ID's:
        current_ID = target_vehicles_generator.target_vehicles_output_dict[self.__current_target_xml_file__]
        i = 0
        while i < num_vehicles:
            pair = random.sample(self.edge_list, 2)
            if validate_path(self.net, pair[0], pair[1]):
                vehicles_info.append( (current_ID + i, pair, True) )
                i += 1
            ### UNCOMMENT TO DEBUG ###
            #else:
                #print("No path from", pair[0].getID(), "to", pair[1].getID())
                
        return vehicles_info
    
    
    def random_select_edge_IDs(self, num_of_edges):
        """
            param @num_of_edges <int>: the number of distinct edge ID's desired.
            
            Returns @num_of_edges randomly selected and distinct edge ID's from the
            list of edges stored in the member variable @target_vehicles_generator.edge_list.
            If @num_of_edges is greater than the total number of edges available in
            @target_vehicles_generator.edge_list, the function will return a value of 'None'.
        """
        edge_count = len(self.edge_list)
        if (edge_count < num_of_edges):
            print("Warning: Number of edges to select exceeds the maximum of " + str(edge_count) + "! Function 'select_edge_IDs' returns a 'None' value...")
            return None
        else:
            edge_indices = random.sample( self.edge_list, num_of_edges )
            return edge_indices
    

    def reset_vehicle_info(self, target_xml_file):
        """
            param @target_xml_file <str>: name of the target xml file.
            
            Function to clear the vehicle information in the file specified by
            @target_xml_file, and reset the generated-vehicle count associated
            with this file, static @target_vehicles_generator.target_vehicles_output_dict
            [@target_xml_file], to 0.
        """
        self.__current_target_xml_file__ = target_xml_file
        # TODO: Clear the vehicle information in the target-xml output file.
        
        target_vehicles_generator.target_vehicles_output_dict[target_xml_file] = 0

    def generate_vehicles(self, num_target_vehicles, num_random_vehicles, pattern, target_xml_file, net_xml_file,start_edge\
        ,end_edge):
        """
            param @num_target_vehicles <int>: The number of target vehicles.
            param @num_random_vehicles <int>: The number of uncontrolled vehicles.
            param @pattern <tuple>: one of three possible patterns. FORMAT:
            -- CASES BEGIN --
                #1. one start point, one destination for all target vehicles
                #2. ranged start point, one destination for all target vehicles
                #3. ranged start points, ranged destination for all target vehicles
                #4. set of edges considered random trips similar to pattern 3 discontinued moved
            -- CASES ENDS --

            Returns the list of target vehicles if succeeds.
            Returns None if the generation fails with error infromation output to the console.
            The result will be written into the target_xml_file.
            There is no guaratnee on the contents in target_xml_file if the generation fails, i.e., returns None
        """
        #set the start time as 0 (by default) and the end time as 50
        #calculate the density of vehicles accordingly
        latest_release_time = 50.0 #a constant number for the latest release time of all vehicles
        num_random_vehicles *= 2 # this is done to compensate the loss when generating using scripts. Need to solve this later.
        density = 1
        if num_random_vehicles >0:
            density =  latest_release_time / float(num_random_vehicles)
            density = int(density * 100)/100.0
        #copy the file randomTrips.py to the current directory
        '''
        command_str = "copy $SUMO_HOME/tools/randomTrips.py ./"
        if os.system(command_str) != 0:
            print("ERROR: Failed to copy randomTrips.py to current directory.")
            return None
        '''
        #invoke randomTrips.py
        print("net_xml_file:",net_xml_file)
        print("what's our target",target_xml_file)
        command_str = "python randomTrips.py -n "+net_xml_file+" -e 50 -p "+str(density) +" -r "+target_xml_file
        if os.system(command_str) != 0:
            print("ERROR: Failed to invoke randomTrips.py.")
            return None
        #delete randomTrips.py
        '''
        command_str = "rm ./randomTrips.py"
        if os.system(command_str) != 0:
            print("ERROR: Failed to remove randomTrips.py.")
            return None
        '''
        #insert the generated vehicles into the xml file
        #use id to find the vehicles and modify their information directly
        result_dict = None
        if pattern==1:
            param_start = random.choice(self.edge_list)
            param_dest = random.choice(self.edge_list)
            while not validate_path(self.net, param_start, param_dest):
                param_start = random.choice(self.edge_list)
                param_dest = random.choice(self.edge_list)
                ### UNCOMMENT TO DEBUG ###
                #print("DEBUG: pattern 1 regenerating.")
            result_dict = self.generate_target_vehicles(num_target_vehicles, target_xml_file, (param_start, param_dest) )
        elif pattern==2:
            param_start = __random_choices_with_rp__(self.edge_list, num_target_vehicles*2)
            param_dest = random.choice(self.edge_list)
            #all pairs must be valid
            while not validate_path_start_points(self.net, param_start, param_dest):
                param_start = __random_choices_with_rp__(self.edge_list, num_target_vehicles*2)
                param_dest = random.choice(self.edge_list)
                ### UNCOMMENT TO DEBUG ###
                #print("DEBUG: pattern 2 regenerating.")
            result_dict = self.generate_target_vehicles(num_target_vehicles, target_xml_file, (param_start, param_dest) )
        elif pattern==3:
            param_start = []
            param_dest = []
            x = 0
            while x < num_target_vehicles:
                #while False:
                param_start_temp = random.choice(self.edge_list)
                param_dest_temp = random.choice(self.edge_list)
                #print(param_start_temp)
                if param_dest_temp == param_start_temp:
                    continue
                if self.net.getShortestPath(param_start_temp,param_dest_temp) == None:
                    continue
                else:
                    param_start.append(param_start_temp)
                    param_dest.append(param_dest_temp)
                    x+=1

            
            # param_start = __random_choices_with_rp__(self.edge_list, num_target_vehicles*2)
            # param_dest = __random_choices_with_rp__(self.edge_list, num_target_vehicles*2)
            #at least one group of start points and one destination is valid towards each other
            # print(type(param_start))
            # print(type(param_dest))
            # print(param_start)
            # print(param_dest)
            # while not validate_path_starts_ends(self.net, param_start, param_dest):
            #     print("this really is the issue")
            #     param_start = __random_choices_with_rp__(self.edge_list, num_target_vehicles*2)
            #     param_dest = __random_choices_with_rp__(self.edge_list, num_target_vehicles*2)
            #     ### UNCOMMENT TO DEBUG ###
            #     #print("DEBUG: pattern 3 regenerating.")
            result_dict = self.generate_target_vehicles(num_target_vehicles, target_xml_file, (param_start, param_dest))
        # elif pattern == 4:
        #     print(self.edge_list)
        #     print(type(self.edge_list))
        #     param_start = []
        #     param_dest = []
        #     #self.start_edges_using = None
        #     #self.end_edges_using = None
        #     #print("ruh rho",self.start_edges_using)
        #     print(len(self.end_edges_using))
        #     print(self.end_edges_using)
        #     #print(edge_dependencies.keys())
        #     for x in range(0,num_target_vehicles):
        #         start_index = random.randint(0,len(edge_dependencies.keys())-1)
        #         start_e = list(edge_dependencies.keys())[start_index]
        #         while True:
        #             end_index = random.randint(0,len(edge_dependencies.keys())-1)
        #             temp_edge =  list(edge_dependencies.keys())[end_index]
        #             if end_index == start_index:
        #                 continue
        #             elif type(edge_dependencies[temp_edge]) is list:
        #                 end_index2 = random.randint(0,len(edge_dependencies[temp_edge])-1)
        #                 end_e = edge_dependencies[temp_edge][end_index2]

        #                 if type(edge_dependencies[start_e]) is list:
        #                     if end_e in edge_dependencies[start_e]:
        #                         continue
        #                     else:
        #                         break
        #                 else:
        #                     break
                    
                      
        #         param_start.append(self.start_edges_using[start_index])#start_e)
        #         print("end_index:",end_index)
        #         print("end_index2:",end_index2) #use index of to get this set up
        #         param_dest.append(self.end_edges_using[end_index])
                
            #result_dict = self.generate_target_vehicles(num_target_vehicles, target_xml_file, (param_start, param_dest) )
        else:
            print("ERROR: Unknown pattern type.")
            return None
        error_message = result_dict[self.__ERROR_MESSAGE__]
        result_lst = result_dict[self.VEHICLES_INFO]
        if error_message != None:
            print(error_message)
            return None
        #put the vehicle information into a list of Vehicle objects
        vehicle_list = []
        release_time = 0
        release_period = .2#latest_release_time/float(num_target_vehicles)#TODO most likely want to flip this division
        #reason: in order to increase the amount of time between releases I think This is going to increase very quickly
        #otherwise

        #read the xml file
        #print("what's our target: ",target_xml_file)
        doc = xml.dom.minidom.parse(target_xml_file)
        root = doc.documentElement
        vs = root.getElementsByTagName("vehicle")
        index = 0
        #print("das right")
        #print(len(vs))
        #print("das right")
        id_now = int(vs.item(len(vs) - 1).getAttribute('id')) + 1
        #deadline set arbitrarily between a certain range
        #random_s_edge = True
        # multi_s_edge = False
        # if start_edge == "-1":
        #     random_s_edge = True
        # if start_edge == "-2":
        #     multi_s_edge = True
        # iterator = 0

        trips = {}
        deadlines = {}
        # if pattern == 4:
        #     # file_name = "static_trip.rou.xml"
        #     # doc = minidom.parse("./configurations/"+file_name)

        #     # veh = doc.getElementsByTagName("trip")
        #     # #self.
        #     # for t in veh:
        #     #     #print(t)
        #     #     vid = t.getAttribute("id")
        #     #     start1 = t.getAttribute("from")
        #     #     dest1 = t.getAttribute("to")
        #     #     trips[vid]  = [start1,dest1]

        #     working_set = pd.read_csv("./history/SUMO_Single_Trip_Data.csv") #retriev
        #     x = working_set["vehicle_id"].values
        #     y = working_set["deadline"].values
        #     for hero in range(len(x)):
        #         deadlines [x[hero]] = y[hero] 
                #print(vid," : ",[start1,dest1])

                
        
        for r in result_lst:
            #find the vehicle slot based on the depart time (departure time must be sorted in xml)
            for i in range(index, len(vs)):
                if float(vs.item(i).getAttribute('depart')) <= release_time:
                    index = i
                else:
                    break
            
            #insert the vehicle into the xml file
            #print(index)
            temp_v = doc.createElement('vehicle')
            temp_v.setAttribute('depart', str(release_time))
            temp_v.setAttribute('id', str(id_now))
            temp_r = doc.createElement('route')

            #print("start_edge"+str(start_edge))
        
                #print("yes it works thanks")
            #if random_s_edge == True:
            start_edge = r[1][0].getID()
            end_edge = r[1][1].getID()
            #start ="15420484#3"#r[1][0].getID() #"-gneE23"
            #print("START:".format(start))
            #print(start_edge)
            temp_r.setAttribute('edges', start_edge)#r[1][0].getID()) #set the start edge as the route
            #start edges
            #iterator = random.randint(0,7)
            # if pattern == 4:
            #     edge_dupe_map = {}
            #     e=0
            #     et = 0
            #     #this is responsible for setting start edges to not use end edges by index comparing start and end edge lists
            #     # considers the lists in form of a list of strings with some lists in between to represent intersections with more than
            #     # 1 start edge attached to it. See 4 corners top right intersection.
            #     for x in range(len(start_edges)):
            #         edge_dupe_map[x] = []
            #         if type(end_edge[e]) is str:
            #             edge_dupe_map[x].append(e)
            #             e+=1
            #         else: # type is list
            #             for y in range(len(end_edge[e])):
            #                 edge_dupe_map[x].append(e+y)
            #             et +=1
            #             if et == len(end_edge[e])-1:
            #                 e=x 
            #                 et = 0

            #     id_now = 0
            #     release_time = 0
            #     release_period = .2
            #     for x in range(num_target_vehicles):
            #         start_edge = ""
            #         end_edge = ""
            #         start_iterator = random.randint(0,len(start_edges)-1)
            #         while True:#this is a stupid setup TODO change this entire method lmao
                        
            #             end_iterator = random.randint(0,len(end_edges)-1)
            #             if end_iterator not in edge_dupe_map[start_iterator]:
                            

            #                 start_edge = start_edges[start_iterator]
            #                 if type(end_edges[end_iterator]) is list:
            #                     iterator2 = random.randint(0,len(end_edges[end_iterator])-1)
            #                     end_edge= end_edges[end_iterator][iterator2]
            #                 else:
            #                     end_edge = end_edges[end_iterator]
            #                 break
            # else:
            #     start_edge = trips[str(id_now)][0]#  = [start1,dest1]
            #     end_edge = trips[str(id_now)][1]

            #temp_v.setAttribute('arrivalEdge', end_edge)
            temp_r.setAttribute('edges', start_edge)#+" "+end_edge)
            temp_v.appendChild(temp_r)
            if index == len(vs) - 1:
                root.appendChild(temp_v)
            else:
                root.insertBefore(temp_v, vs[index+1])
                #root.appendChild(temp_v)
            
            #append the vehicle to the final vehicle list
            #if pattern != 4:
            ddl_now = random.randint(500,1000)#randomly set ddl in a range for now
            # else: 
            #     ddl_now = deadlines[id_now]

            #todo will need to modify this to take values probably from sumo_trips in history LMAO
            v_now = Util.Vehicle(str(id_now),start_edge, end_edge, release_time, ddl_now)
            vehicle_list.append(v_now)
            release_time += release_period
            id_now += 1
            vs = root.getElementsByTagName("vehicle")
        #write the vehicle information into the xml file
        with open(target_xml_file, 'w') as f:
            f.write(doc.toprettyxml())
            f.flush()
            f.close
        print("target:",target_xml_file)
        return vehicle_list


    
def validate_path(net, start_point, destination):
    """
        param @net <sumolib.net.Net>: parameter that stores the information of a map.
        param @start_point <sumolib.net.edge.Edge>: a start-point on the map from @net.
        param @destination <sumolib.net.edge.Edge>: a destination on the map from @net.
        
        Function to validate the existence of a path from @start_point to @destination,
        using the shortest path algorithm offered by @net; returns True if such a path
        exists, and False otherwise.
        
    """
    shortestPath = net.getShortestPath(start_point, destination)
    #print("SHORTEST PATH:",shortestPath)
    return shortestPath[0] != None
    
def validate_path_start_points(net, start_points, destination):
    """
        param @net <sumolib.net.Net>: parameter that stores the information of a map.
        param @start_point <list of sumolib.net.edge.Edge>: a list of start-points on the map from @net.
        param @destination <sumolib.net.edge.Edge>: a destination on the map from @net.
        
        Function to validate the existence of a path from @start_point to @destination,
        using the shortest path algorithm offered by @net; returns True if such a path
        exists, and False otherwise.
    """
    num = 0
    for s in start_points:
        #print(s)
        shortest_path = net.getShortestPath(s, destination)
        if shortest_path[0]==None:
            #print("it goes") #AAAAAAAAAAA this is so stupdi
            return False
        num += 1
        if num >= len(start_points)/2:
            return True
    return True

def validate_path_starts_ends(net, start_points, destinations):
    """
        param @net <sumolib.net.Net>: parameter that stores the information of a map.
        param @start_point <list of sumolib.net.edge.Edge>: a list of start-points on the map from @net.
        param @destination <list of sumolib.net.edge.Edge>: a destination on the map from @net.
        
        Function to validate the existence of a path from @start_point to @destination,
        using the shortest path algorithm offered by @net; returns True if such a path
        exists, and False otherwise.
    """
    for d in destinations:
        if validate_path_start_points(net, start_points, d):
            return True
    return False
    
# Auxiliary Functions:
def __random_choices_with_rp__(lst, k=1):
    """
        param @lst <list>: a list of elements.
        param @k <int>: the number of elements to generate.
            
        Returns @k elements, stored in a list, repetitively selected at random with
        replacement from @lst.
    """
    result_lst = []
    while 0 < k:
        result_lst.append( random.choice(lst) )
        k -= 1
    return result_lst
    
        




#os.system(command)
