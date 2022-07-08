
    
from controller.RouteController import RouteController
from core.Util import ConnectionInfo, Vehicle
import numpy as np
import traci
import math
import copy
from core.Estimation import *
from core.STR_SUMO import *
from xml.dom import minidom
from history.data_recorder import *
import history.data_recorder
import core.Run_id
from core.Time_estimation.Timer_estimator_new import Time_predictors
from collections import OrderedDict

#this is just dijkstra's but with my saving of the route instead of recalculating it every time bit I did in selflesscontroller.

class STRBE_Collector(RouteController):
    #BE as in before execution
    def get_edges(self,vehicle,connection_info,d_list):
        e_list = []
        c_edge = vehicle.current_edge#self.decision[vehicle.vehicle_id]
        #print("vehicle:",vehicle.vehicle_id)#{}c_edge: {} decision_edge : {}".format(vehicle.vehicle_id,c_edge,self.decision[vehicle.vehicle_id]))
        #print("route:",self.nu_trips[vehicle.vehicle_id])
        #print("directions,",self.routes[vehicle.vehicle_id])
        e_list.append(c_edge)
        for d in d_list:
            if d == c_edge:
                continue
            #print("c_edge: {} d: {}".format(c_edge,d))
            #print("c_edge: {}  d: {} -> to {}".format(c_edge,d,self.connection_info.outgoing_edges_dict[c_edge][d]))
            c_edge = self.connection_info.outgoing_edges_dict[c_edge][d]
            e_list.append(c_edge)
        return e_list
    
    routes = {} # = to route with regards to directions from current edge is retruend by make decisions
    position = {}
    routes_edges = {} # same as routes but has the explicit id for each edge (ow the edge)
    base_trips = {}
    nu_trips = {}
    route_changed = []
    work_with = {}# vehicles rerouted that can change with another vehicle
    work_with_inverse = {} # vehicles not rerouted that have options
    cross_over = {}
    sharing_route = {}
    #also consider looking at start time of vehicles to see if they're going to be on the map at the same time
    est_travel_time = {}#estimated travel time of 
    num_edges = {} #number of edges a vehicle has to pass through
    distance_route = {} #distance wrt a vehicles routes
    num_on_edges = {}# a list for each edge that stores the VID's of vehicles on the list to get the number of vehicles going down a route

    #est_travel_time2 = {}
    def get_directions(self,connection_info,c_list):
        d_list =[] #direction list
        y=0
        #print("c_list: {}".format(c_list))
        for x in range(1,len(c_list)):
            #print(connection_info.outgoing_edges_dict[c_list[y]].items())
            for x2 in connection_info.outgoing_edges_dict[c_list[y]].items():
                #print(x2)
                if x2[1] == c_list[x]:
                    #print(c_list[x],' ',x2[1])
                    #print(x2[0])
                    d_list.extend([x2[0]])
                    break
            y+=1
        #print("here")
        #print("take a look at this D ... list: {}".format(d_list))
        return d_list        
            

    def __init__(self, connection_info,file_name,file_name2,vehicles,start_difference):
        super().__init__(connection_info)
        print("\nTime difference equals= ", start_difference)
        #csv2Data('./History/switch_information.csv')
        num_switched = 0
        self.route_information={}   # of form [est_travel_time,#_vehs_sharing_route,num_edges,self.distance_of_route]
        #self.predictors = predictors
        self.routes = {} # = to route with regards to directions from current edge is retruend by make decisions
        self.position = {}
        self.routes_edges = {} # same as routes but has the explicit id for each edge (ow the edge)
        self.base_trips = {}
        self.nu_trips = {}
        self.route_changed = []
        self.work_with = {}# vehicles rerouted that can change with another vehicle
        #self.work_with_inverse = {} # vehicles not rerouted that have options
        self.cross_over = {}
        self.sharing_route = {}
        #also consider looking at start time of vehicles to see if they're going to be on the map at the same time
        self.est_travel_time = {}#estimated travel time of 
        self.num_edges = {} #number of edges a vehicle has to pass through
        self.distance_route = {} #distance wrt a vehicles routes
        self.num_on_edges = {}
        self.route_edge_difference = {}
        self.edge_swapped_dict= {} #is of form {edge_id:{destination:listofvehicles}} that being vehicles who swap on
        route_options = {} #of form {vehicle_id:number_of_options_considered}
        core.Run_id.collector = True
        overall= []
        temp = []
        #self.comparison
        #file_name = "static2.rou.xml"
        doc = minidom.parse("./configurations/"+file_name)

        veh = doc.getElementsByTagName("vehicle")
        #self.
        for t in veh:
            vid = t.getAttribute("id")
            route_tag = t.getElementsByTagName("route")[0]
            route = route_tag.getAttribute("edges")
            root = route.split(' ')
            self.base_trips[vid]  = root
            #print(vid," : ", root )

        doc2 = minidom.parse("./configurations/"+file_name2)

        veh2 = doc2.getElementsByTagName("vehicle")
        #self.
        for t in veh2:
            vid = t.getAttribute("id")
            route_tag = t.getElementsByTagName("route")[0]
            route = route_tag.getAttribute("edges")
            root = route.split(' ')
            self.nu_trips[vid]  = root

        
        for x in self.nu_trips.keys():
            self.est_travel_time[x] = 0
            self.distance_route[x] =0
            self.num_edges[x] = len(self.nu_trips[x])

            for y in self.nu_trips[x]:
                self.est_travel_time[x] += connection_info.edge_length_dict[y]/connection_info.edge_speed_dict[y]
                self.distance_route[x] += connection_info.edge_length_dict[y]
                if y not in self.num_on_edges.keys():
                    self.num_on_edges[y] = []
                if x not in self.num_on_edges[y]:
                    self.num_on_edges[y].append(x)

        for x in self.nu_trips.keys():  
            self.sharing_route[x] = 0  
            temp_ids = []
            for y in self.nu_trips[x]:
                #print(type(temp_ids))
                #print(type(self.num_on_edges[y]))
                temp_ids =  set(self.num_on_edges[y] + list(temp_ids))
            self.sharing_route[x] = len(list(temp_ids))
            #print()
            self.route_information[x] =[vehicles[x].start_time,self.est_travel_time[x],self.sharing_route[x],self.num_edges[x],self.distance_route[x]]
        #print("working 1")
            #print(vid," : ", root )
        #print(self.base_trips.keys())
        #print(self.nu_trips.keys())
        for x in self.base_trips.keys():
            if self.base_trips[x] != self.nu_trips[x]:
                self.route_edge_difference[x] = []
                for z in range(len(self.base_trips[x])):
                    if self.base_trips[x][z] != self.nu_trips[x][z]:
                        if z > 0:
                            self.route_edge_difference[x] = [z,[self.base_trips[x][z-1:]],self.base_trips[x][z-1]]
                        else:
                            self.route_edge_difference[x] = [0,[self.base_trips[x][0:]],self.base_trips[x][0]]
                        # print("ROUUUUUUUTES:")
                        # print(self.base_trips[x])
                        # print(self.nu_trips[x])
                        # print(self.route_edge_difference[x])
                        break
                # print("vehicle:", x)
                # print("OLD trip:" ,self.base_trips[x])
                # print("new Trip:", self.nu_trips[x])
                self.route_changed.append(x)


        unique = []
        for x in self.route_edge_difference.keys():
            
            if self.route_edge_difference[x][2] not in unique:
                unique.append(self.route_edge_difference[x][2] )
        #print("unique:",unique)
        #print("route_changed:",self.route_changed)
        for x in self.route_changed:   
            #self.edge_swapped_dict= {}
            the_edge = self.route_edge_difference[x][2]
            the_dest = vehicles[x].destination
            # print("vehicle:",x)
            # print("the_edge",the_edge)
            # print("the_dest",the_dest)
            if the_edge not in self.edge_swapped_dict.keys():
                self.edge_swapped_dict[the_edge] = {}
            if the_dest not in self.edge_swapped_dict[the_edge].keys():
                self.edge_swapped_dict[the_edge][the_dest] = []

            self.edge_swapped_dict[the_edge][the_dest].append(x)
            #print("literal monster",self.edge_swapped_dict)
        
        #print("literal monster",self.edge_swapped_dict)
        vid_by_deadline =[]
        #print("ok here?")
        #print("vehicles",vehicles)
        for x in vehicles:
            if not vid_by_deadline:
                #print("EX:",x)
                vid_by_deadline.append(x)
                continue
            for y in range(0,len(vid_by_deadline)):
                if vehicles[vid_by_deadline[y]].deadline > vehicles[x].deadline:
                    vid_by_deadline.insert(y,x)
                    break
                if y == len(vid_by_deadline)-1:
                    vid_by_deadline.append(x)


        #print("vid_by_deadline",vid_by_deadline)
        for current in vid_by_deadline:
            edges_and_predictions = {} #of form {edge:[vehicle_id,predicted time.]}
            #current_prediction = 0
            veh_to_swap = -1
            The_edge = -1
            route_options[current]=0
            #print("keys:",self.edge_swapped_dict.keys())
            for y in self.nu_trips[current]:
                #print("consider edge:",y)
                if y in self.edge_swapped_dict.keys():
                    #the_edge = y
                    the_dest = vehicles[current].destination
                    if the_dest in self.edge_swapped_dict[y].keys():
                        #print("with destination:",the_dest)
                        edges_and_predictions[y] = [-1,1000000]
                        #current_prediction = self.predictors.route_est(self.route_information[current])[0][0]
                        for considering in self.edge_swapped_dict[y][the_dest]:
                            # print("vehicle:",considering)
                            # print("The_edge:",y)
                            # print(self.nu_trips[considering])
                            if y not in self.nu_trips[considering]: # this means that hte vehicle previously swapped and no longer has the edge
                                continue
                            if current == considering:
                                continue
                            if vehicles[current].deadline > vehicles[considering].deadline:
                                continue
                            # if vehicles[current].start_time> vehicles[considering].start_time:
                            #     continue    
                            #is this faster for current?
                            if self.base_trips[current] == self.base_trips[considering]:
                                temp_trip_info = self.route_information[considering]

                            else:
                                #print("wait does this even run?")
                                current_idx = self.nu_trips[current].index(y)
                                considering_idx = self.nu_trips[considering].index(y)
                                temp_route = self.nu_trips[current][:current_idx]
                                temp_route.extend(self.nu_trips[considering][considering_idx:])
                        
                                #print("AAAAA WHAY",x2)
                                temp_est_travel_time = 0
                                temp_distance_route =0
                                temp_num_edges = len(temp_route)
                                temp_sharing_route = 0  
                                temp_ids = []
                                
                                for y2 in temp_route:
                                    temp_est_travel_time += connection_info.edge_length_dict[y2]/connection_info.edge_speed_dict[y2]
                                    temp_distance_route += connection_info.edge_length_dict[y2]
                                    temp_ids = set(self.num_on_edges[y2] + list(temp_ids))
                                    
                                temp_sharing_route = len(list(temp_ids))
                                temp_trip_info =[vehicles[considering].start_time,temp_est_travel_time,temp_sharing_route,temp_num_edges,temp_distance_route]
                            considering_prediction = abs(sum(x2 - y2 for x2, y2 in zip(self.route_information[current],self.route_information[considering] )))#self.predictors.route_est(temp_trip_info)[0][0]

                            #route_difference = current_prediction - considering_prediction
                            route_difference = sum(x2 - y2 for x2, y2 in zip(self.route_information[current],self.route_information[considering] ))
                        
                            #print("route difference:",route_difference)
                            if route_difference > 0:
                                if edges_and_predictions[y][1] < considering_prediction:
                                    edges_and_predictions[y] = [considering,considering_prediction]
            
                #current_difference = abs(vehicles[current].deadline-current_prediction)
                
            considering_difference = 0
            # print("vehicle:",current)
            # print("considering",edges_and_predictions)
            for y2 in edges_and_predictions.keys():
                
                if edges_and_predictions[y2][0] == -1:
                    continue
                route_options[current]+=1
                vid_temp=edges_and_predictions[y2][0]
                temp_dif = abs(vehicles[current].deadline-edges_and_predictions[y2][1]+vehicles[current].start_time)
                #need to be a little more due notice for the considering difference
                # print("current:",current,' considering:',vid_temp)
                # print('deadlines:',vehicles[current].deadline,' ',vehicles[vid_temp].deadline)
                # print("temp_dif:",temp_dif)
                # print("considering_dif:",considering_difference)
                if considering_difference < temp_dif:
                    considering_difference = temp_dif
                    veh_to_swap = vid_temp
                    The_edge = y2
            # print("vehicle:",current)
            # print("number alternatives considered:",route_options[current])    
            if veh_to_swap != -1:
                #print("swap:VV:",current,' and VV:',veh_to_swap)
                considering=veh_to_swap
                num_switched += 1
                # if current not in self.edge_swapped_dict[The_edge][the_dest]:
                #     self.edge_swapped_dict[The_edge][the_dest].append(current)
                for x2 in [current,considering]:
                    for y2 in self.nu_trips[x2]:
                        if x2 in self.num_on_edges[y2]:
                            self.num_on_edges[y2].remove(x2)
                current_idx = self.nu_trips[current].index(The_edge)
                considering_idx = self.nu_trips[considering].index(The_edge)
                current_swap = self.nu_trips[current][current_idx:]
                considering_swap = self.nu_trips[considering][considering_idx:]
                # print("vehicles:",current," and ", considering)
                # print("before")
                # print("on edge:",The_edge)
                # print(self.nu_trips[current])
                # print(self.nu_trips[considering])

                self.nu_trips[current]=self.nu_trips[current][:current_idx]
                self.nu_trips[current].extend(considering_swap)
                self.nu_trips[considering]=self.nu_trips[considering][:considering_idx]
                self.nu_trips[considering].extend(current_swap)
                # print("after")
                # print(self.nu_trips[current])
                # print(self.nu_trips[considering])
                for x2 in [current,considering]:
                    #print("AAAAA WHAY",x2)
                    self.est_travel_time[x2] = 0
                    self.distance_route[x2] =0
                    self.num_edges[x2] = len(self.nu_trips[x2])

                    for y2 in self.nu_trips[x2]:
                        self.est_travel_time[x2] += connection_info.edge_length_dict[y2]/connection_info.edge_speed_dict[y2]
                        self.distance_route[x2] += connection_info.edge_length_dict[y2]
                        # print(x2)
                        # print(y2)
                        if x2 not in self.num_on_edges[y2]:
                            self.num_on_edges[y2].append(x2)

                for x2 in [current,considering]:#self.nu_trips.keys():  
                    self.sharing_route[x2] = 0  
                    temp_ids = []
                    for y2 in self.nu_trips[x2]:
                        temp_ids = set(self.num_on_edges[y2] + list(temp_ids))
                    self.sharing_route[x2] = len(list(temp_ids))
                    self.route_information[x2] =[vehicles[x2].start_time,self.est_travel_time[x2],self.sharing_route[x2],self.num_edges[x2],self.distance_route[x2]]

                #rework was here









                   
                   

                    # route_difference =current_prediction \
                    #      - self.predictors.route_est(temp_trip_info)
                    # #route_difference = sum(x2 - y2 for x2, y2 in zip(self.route_information[current],self.route_information[considering] ))
                    
                    # print("route difference:",route_difference)
                    # if route_difference > 0:
                       
                        # temp.append(core.Run_id.Round_sufix)
                        # temp.append(core.Run_id.run_id)
                        # temp.append(current)
                        # temp.append(considering)
                        # temp.append(vehicles[current].start_time)
                        # temp.append(vehicles[considering].start_time)
                        # temp.append(0)
                        # temp.append(0)
                        # temp.append(0)
                        # temp.append(0)
                        # temp.append(0)
                        # temp.append(0)
                        # temp.append(0)
                        # temp.append(0)
                        # temp.append(The_edge)
                        # temp.append(vehicles[current].deadline)
                        # temp.append(vehicles[considering].deadline)

                        # overall.append(temp)
                        # temp = []
                      
                        
        
        print("Number of vehicles that have switched:",num_switched)     
        core.Run_id.Num_switched = num_switched
        core.Run_id.trips = self.nu_trips
        core.Run_id.route_information = self.route_information
        core.Run_id.Controller_version = "STRBE_collector"
        core.Run_id.vehicle_num_options= route_options
        #data2Csv_general(overall,'./History/switch_information.csv')


        
    def dijkstra(self,vehicle, connection_info):
        decision_list = []
        unvisited = {edge: 1000000000 for edge in self.connection_info.edge_list} # map of unvisited edges
        visited = {} # map of visited edges
        current_edge = vehicle.current_edge
        
        current_distance = self.connection_info.edge_length_dict[current_edge]/self.connection_info.edge_speed_dict[current_edge] 
        unvisited[current_edge] = current_distance
        path_lists = {edge: [] for edge in self.connection_info.edge_list} #stores shortest path to each edge using directions
        while True:
            if current_edge not in self.connection_info.outgoing_edges_dict.keys():
                continue
            for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_edge].items():
                if outgoing_edge not in unvisited:
                    continue
                edge_length = self.connection_info.edge_length_dict[outgoing_edge]/self.connection_info.edge_speed_dict[outgoing_edge]# self.edge_speed_dict
                new_distance = current_distance + edge_length
                if new_distance < unvisited[outgoing_edge]:
                    unvisited[outgoing_edge] = new_distance
                    current_path = copy.deepcopy(path_lists[current_edge])
                    current_path.append(direction)
                    path_lists[outgoing_edge] = copy.deepcopy(current_path)

            visited[current_edge] = current_distance
            del unvisited[current_edge]
            if not unvisited:#fairly certain this means that there is no other route
                return []
                break
            if current_edge==vehicle.destination:
                break
            possible_edges = [edge for edge in unvisited.items() if edge[1]]
            current_edge, current_distance = sorted(possible_edges, key=lambda x: x[1])[0]
            


        for direction in path_lists[vehicle.destination]:
            decision_list.append(direction)
        #print("dijkstra:{}".format(decision_list))
        return decision_list   
    def make_decisions(self, vehicles, connection_info):
        """
        make_decisions algorithm uses Dijkstra's Algorithm to find the shortest path to each individual vehicle's destination
        :param vehicles: list of vehicles on the map
        :param connection_info: information about the map (roads, junctions, etc)
        """
        #print("what to heck")
        local_targets = {}
        #first_run = True
        #updated = False
        for vehicle in vehicles:
            if vehicle.vehicle_id not in self.routes.keys():
               
                
                self.routes.update({vehicle.vehicle_id:self.get_directions(connection_info,self.nu_trips[vehicle.vehicle_id])})#self.dijkstra(vehicle, connection_info)})
                
                self.position.update({vehicle.vehicle_id:0})
                self.routes_edges.update({vehicle.vehicle_id:self.get_edges(vehicle, connection_info,self.routes[vehicle.vehicle_id])})
                
                Vehicle_route["heh"] = copy.deepcopy(self.routes_edges)
                #updated=True
                
            
            while self.routes_edges[vehicle.vehicle_id][self.position[vehicle.vehicle_id]] != vehicle.current_edge:
                self.position[vehicle.vehicle_id] +=1
                if self.position[vehicle.vehicle_id] == len(self.routes_edges[vehicle.vehicle_id]):
                    break
                if self.routes[vehicle.vehicle_id]:
                    del self.routes[vehicle.vehicle_id][0]
                #updated=True
            
         
          
            decision_list = self.routes[vehicle.vehicle_id]   
            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)

            if local_targets[vehicle.vehicle_id] not in self.routes_edges[vehicle.vehicle_id]:
                self.routes.update({vehicle.vehicle_id:self.dijkstra(vehicle, connection_info)})
                self.position.update({vehicle.vehicle_id:0})
                self.routes_edges.update({vehicle.vehicle_id:self.get_edges(vehicle, connection_info,self.routes[vehicle.vehicle_id])})
                Vehicle_route["heh"] = copy.deepcopy(self.routes_edges)
               
                decision_list = self.routes[vehicle.vehicle_id]   
                local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
            #print(local_targets)
        
           
 
        return local_targets
# from controller.RouteController import RouteController
# from core.Util import ConnectionInfo, Vehicle
# import numpy as np
# import traci
# import math
# import copy
# from core.Estimation import *
# from core.STR_SUMO import *
# from xml.dom import minidom
# from history.data_recorder import *
# import history.data_recorder
# import core.Run_id
# from core.Time_estimation.Timer_estimator_new import Time_predictors
# from collections import OrderedDict

# #this is just dijkstra's but with my saving of the route instead of recalculating it every time bit I did in selflesscontroller.

# class STRBE_Collector(RouteController):
#     #BE as in before execution
#     def get_edges(self,vehicle,connection_info,d_list):
#         e_list = []
#         c_edge = vehicle.current_edge#self.decision[vehicle.vehicle_id]
#        # print("vehicle:",vehicle.vehicle_id)#{}c_edge: {} decision_edge : {}".format(vehicle.vehicle_id,c_edge,self.decision[vehicle.vehicle_id]))
#         #print("route:",self.nu_trips[vehicle.vehicle_id])
#        # print("directions,",self.routes[vehicle.vehicle_id])
#         e_list.append(c_edge)
#         for d in d_list:
#             if d == c_edge:
#                 continue
#             #print("c_edge: {} d: {}".format(c_edge,d))
#            # print("c_edge: {}  d: {} -> to {}".format(c_edge,d,self.connection_info.outgoing_edges_dict[c_edge][d]))
#             c_edge = self.connection_info.outgoing_edges_dict[c_edge][d]
#             e_list.append(c_edge)
#         return e_list
    
#     routes = {} # = to route with regards to directions from current edge is retruend by make decisions
#     position = {}
#     routes_edges = {} # same as routes but has the explicit id for each edge (ow the edge)
#     base_trips = {}
#     nu_trips = {}
#     route_changed = []
#     work_with = {}# vehicles rerouted that can change with another vehicle
#     work_with_inverse = {} # vehicles not rerouted that have options
#     cross_over = {}
#     sharing_route = {}
#     #also consider looking at start time of vehicles to see if they're going to be on the map at the same time
#     est_travel_time = {}#estimated travel time of 
#     num_edges = {} #number of edges a vehicle has to pass through
#     distance_route = {} #distance wrt a vehicles routes
#     num_on_edges = {}# a list for each edge that stores the VID's of vehicles on the list to get the number of vehicles going down a route

#     #est_travel_time2 = {}
#     def get_directions(self,connection_info,c_list):
#         d_list =[] #direction list
#         y=0
#         #print("c_list: {}".format(c_list))
#         for x in range(1,len(c_list)):
#             #print(connection_info.outgoing_edges_dict[c_list[y]].items())
#             for x2 in connection_info.outgoing_edges_dict[c_list[y]].items():
#                 #print(x2)
#                 if x2[1] == c_list[x]:
#                     #print(c_list[x],' ',x2[1])
#                     #print(x2[0])
#                     d_list.extend([x2[0]])
#                     break
#             y+=1
#         #print("here")
#         #print("take a look at this D ... list: {}".format(d_list))
#         return d_list        
            

#     def __init__(self, connection_info,file_name,file_name2,Round_name,vehicles,start_difference,predictors):
#         super().__init__(connection_info)
#         print("\nTime difference equals= ", start_difference)
#         csv2Data('./History/switch_information.csv')
#         num_switched = 0
#         self.route_information={}   # of form [est_travel_time,#_vehs_sharing_route,num_edges,self.distance_of_route]
#         self.predictors = predictors
#         self.routes = {} # = to route with regards to directions from current edge is retruend by make decisions
#         self.position = {}
#         self.routes_edges = {} # same as routes but has the explicit id for each edge (ow the edge)
#         self.base_trips = {}
#         self.nu_trips = {}
#         self.route_changed = []
#         self.work_with = {}# vehicles rerouted that can change with another vehicle
#         #self.work_with_inverse = {} # vehicles not rerouted that have options
#         self.cross_over = {}
#         self.sharing_route = {}
#         #also consider looking at start time of vehicles to see if they're going to be on the map at the same time
#         self.est_travel_time = {}#estimated travel time of 
#         self.num_edges = {} #number of edges a vehicle has to pass through
#         self.distance_route = {} #distance wrt a vehicles routes
#         self.num_on_edges = {}
#         self.route_edge_difference = {}
#         self.edge_swapped_dict= {} #is of form {edge_id:{destination:listofvehicles}} that being vehicles who swap on
#         route_options = {} #of form {vehicle_id:number_of_options_considered}
        
#         doc = minidom.parse(core.Run_id.rounds_directory+Round_name+'/'+file_name)

#         veh = doc.getElementsByTagName("vehicle")
#         #self.
#         for t in veh:
#             vid = t.getAttribute("id")
#             route_tag = t.getElementsByTagName("route")[0]
#             route = route_tag.getAttribute("edges")
#             root = route.split(' ')
#             self.base_trips[vid]  = root
#             #print(vid," : ", root )

#         doc2 = minidom.parse(core.Run_id.rounds_directory+Round_name+'/'+file_name2)

#         veh2 = doc2.getElementsByTagName("vehicle")
#         #self.
#         for t in veh2:
#             vid = t.getAttribute("id")
#             route_tag = t.getElementsByTagName("route")[0]
#             route = route_tag.getAttribute("edges")
#             root = route.split(' ')
#             self.nu_trips[vid]  = root

        
#         for x in self.nu_trips.keys():
#             self.est_travel_time[x] = 0
#             self.distance_route[x] =0
#             self.num_edges[x] = len(self.nu_trips[x])

#             for y in self.nu_trips[x]:
#                 self.est_travel_time[x] += connection_info.edge_length_dict[y]/connection_info.edge_speed_dict[y]
#                 self.distance_route[x] += connection_info.edge_length_dict[y]
#                 if y not in self.num_on_edges.keys():
#                     self.num_on_edges[y] = []
#                 if x not in self.num_on_edges[y]:
#                     self.num_on_edges[y].append(x)

#         for x in self.nu_trips.keys():  
#             self.sharing_route[x] = 0  
#             temp_ids = []
#             for y in self.nu_trips[x]:
#                 #print(type(temp_ids))
#                 #print(type(self.num_on_edges[y]))
#                 temp_ids =  set(self.num_on_edges[y] + list(temp_ids))
#             self.sharing_route[x] = len(list(temp_ids))
#             #print()
#             self.route_information[x] =[vehicles[x].start_time,self.est_travel_time[x],self.sharing_route[x],self.num_edges[x],self.distance_route[x]]
#         #print("working 1")
#             #print(vid," : ", root )
#         #print(self.base_trips.keys())
#         #print(self.nu_trips.keys())
#         for x in self.base_trips.keys():
#             if self.base_trips[x] != self.nu_trips[x]:
#                 self.route_edge_difference[x] = []
#                 for z in range(len(self.base_trips[x])):
#                     if self.base_trips[x][z] != self.nu_trips[x][z]:
#                         if z > 0:
#                             self.route_edge_difference[x] = [z,[self.base_trips[x][z-1:]],self.base_trips[x][z-1]]
#                         else:
#                             self.route_edge_difference[x] = [0,[self.base_trips[x][0:]],self.base_trips[x][0]]
#                         # print("ROUUUUUUUTES:")
#                         # print(self.base_trips[x])
#                         # print(self.nu_trips[x])
#                         # print(self.route_edge_difference[x])
#                         break
#                 # print("vehicle:", x)
#                 # print("OLD trip:" ,self.base_trips[x])
#                 # print("new Trip:", self.nu_trips[x])
#                 self.route_changed.append(x)


#         unique = []
#         for x in self.route_edge_difference.keys():
            
#             if self.route_edge_difference[x][2] not in unique:
#                 unique.append(self.route_edge_difference[x][2] )
#         #print("unique:",unique)
#         #print("route_changed:",self.route_changed)
#         for x in self.route_changed:   
#             #self.edge_swapped_dict= {}
#             the_edge = self.route_edge_difference[x][2]
#             the_dest = vehicles[x].destination
#             # print("vehicle:",x)
#             # print("the_edge",the_edge)
#             # print("the_dest",the_dest)
#             if the_edge not in self.edge_swapped_dict.keys():
#                 self.edge_swapped_dict[the_edge] = {}
#             if the_dest not in self.edge_swapped_dict[the_edge].keys():
#                 self.edge_swapped_dict[the_edge][the_dest] = []

#             self.edge_swapped_dict[the_edge][the_dest].append(x)
#             #print("literal monster",self.edge_swapped_dict)
        
#         #print("literal monster",self.edge_swapped_dict)
#         vid_by_deadline =[]
#         #print("ok here?")
#         #print("vehicles",vehicles)
#         for x in vehicles:
#             if not vid_by_deadline:
#                 #print("EX:",x)
#                 vid_by_deadline.append(x)
#                 continue
#             for y in range(0,len(vid_by_deadline)):
#                 if vehicles[vid_by_deadline[y]].deadline > vehicles[x].deadline:
#                     vid_by_deadline.insert(y,x)
#                     break
#                 if y == len(vid_by_deadline)-1:
#                     vid_by_deadline.append(x)


#         #print("vid_by_deadline",vid_by_deadline)
#         for current in vid_by_deadline:
#             edges_and_predictions = {} #of form {edge:[vehicle_id,predicted time]}
#             current_prediction = 0
#             veh_to_swap = -1
#             The_edge = -1
#             route_options[current]=0
#             #print("keys:",self.edge_swapped_dict.keys())
#             #edge_swapped_dict is of form {edge_id:{destination:listofvehicles}}
#             for y in self.nu_trips[current]:
#                 #print("consider edge:",y)
#                 if y in self.edge_swapped_dict.keys(): 
#                     #the_edge = y
#                     the_dest = vehicles[current].destination
#                     if the_dest in self.edge_swapped_dict[y].keys():
#                         #print("with destination:",the_dest)
#                         edges_and_predictions[y] = [-1,1000000]
#                         current_prediction = self.predictors.route_est(self.route_information[current])[0][0]
#                         for considering in self.edge_swapped_dict[y][the_dest]:
#                             # print("vehicle:",considering)
#                             # print("The_edge:",y)
#                             # print(self.nu_trips[considering])
#                             if y not in self.nu_trips[considering]: # this means that hte vehicle previously swapped and no longer has the edge
#                                 continue
#                             if current == considering:
#                                 continue
#                             if vehicles[current].deadline > vehicles[considering].deadline:
#                                 continue
#                             # if vehicles[current].start_time> vehicles[considering].start_time:
#                             #     continue    
#                             #is this faster for current?
#                             if self.base_trips[current] == self.base_trips[considering]:
                                
#                                 temp_trip_info = self.route_information[considering]
#                                 #if current has the same trips up to the shared edge as considering

#                             else:
#                                 #print("wait does this even run?")
#                                 current_idx = self.nu_trips[current].index(y)
#                                 considering_idx = self.nu_trips[considering].index(y)
#                                 temp_route = self.nu_trips[current][current_idx:]
#                                 temp_route.extend(self.nu_trips[considering][:considering_idx])
                        
#                                 #print("AAAAA WHAY",x2)
#                                 temp_est_travel_time = 0
#                                 temp_distance_route =0
#                                 temp_num_edges = len(temp_route)
#                                 temp_sharing_route = 0  
#                                 temp_ids = []
                                
#                                 for y2 in temp_route:
#                                     temp_est_travel_time += connection_info.edge_length_dict[y2]/connection_info.edge_speed_dict[y2]
#                                     temp_distance_route += connection_info.edge_length_dict[y2]
#                                     temp_ids = set(self.num_on_edges[y2] + list(temp_ids))
                                    
#                                 temp_sharing_route = len(list(temp_ids))
#                                 temp_trip_info =[vehicles[considering].start_time,temp_est_travel_time,temp_sharing_route,temp_num_edges,temp_distance_route]
#                             considering_prediction = self.predictors.route_est(temp_trip_info)[0][0]

#                             route_difference = current_prediction - considering_prediction
#                             #route_difference = sum(x2 - y2 for x2, y2 in zip(self.route_information[current],self.route_information[considering] ))
                        
#                             #print("route difference:",route_difference)
#                             if route_difference > 0:
#                                 if edges_and_predictions[y][1] > considering_prediction:
#                                     edges_and_predictions[y] = [considering,considering_prediction]
            
#                 #current_difference = abs(vehicles[current].deadline-current_prediction)
                
#             considering_difference = 0
#             # print("vehicle:",current)
#             # print("considering",edges_and_predictions)
#             for y2 in edges_and_predictions.keys():
                
#                 if edges_and_predictions[y2][0] == -1:
#                     continue
#                 route_options[current]+=1
#                 vid_temp=edges_and_predictions[y2][0]
#                 temp_dif = abs(vehicles[current].deadline-edges_and_predictions[y2][1]+vehicles[current].start_time)
#                 #need to be a little more due notice for the considering difference
#                 # print("current:",current,' considering:',vid_temp)
#                 # print('deadlines:',vehicles[current].deadline,' ',vehicles[vid_temp].deadline)
#                 # print("temp_dif:",temp_dif)
#                 # print("considering_dif:",considering_difference)
#                 if considering_difference < temp_dif:
#                     considering_difference = temp_dif
#                     veh_to_swap = vid_temp
#                     The_edge = y2
                    
#             if veh_to_swap != -1:
#                 #print("swap:VV:",current,' and VV:',veh_to_swap)
#                 considering=veh_to_swap
#                 num_switched += 1
#                 # if current not in self.edge_swapped_dict[The_edge][the_dest]:
#                 #     self.edge_swapped_dict[The_edge][the_dest].append(current)
#                 for x2 in [current,considering]:
#                     for y2 in self.nu_trips[x2]:
#                         if x2 in self.num_on_edges[y2]:
#                             self.num_on_edges[y2].remove(x2)
#                 current_idx = self.nu_trips[current].index(The_edge)
#                 considering_idx = self.nu_trips[considering].index(The_edge)
#                 current_swap = self.nu_trips[current][current_idx:]
#                 considering_swap = self.nu_trips[considering][considering_idx:]

#                 self.nu_trips[current]=self.nu_trips[current][:current_idx]
#                 self.nu_trips[current].extend(considering_swap)
#                 self.nu_trips[considering]=self.nu_trips[considering][:considering_idx]
#                 self.nu_trips[considering].extend(current_swap)
#                 # print("after")
#                 # print(self.nu_trips[current])
#                 # print(self.nu_trips[considering])
#                 for x2 in [current,considering]:
#                     #print("AAAAA WHAY",x2)
#                     self.est_travel_time[x2] = 0
#                     self.distance_route[x2] =0
#                     self.num_edges[x2] = len(self.nu_trips[x2])

#                     for y2 in self.nu_trips[x2]:
#                         self.est_travel_time[x2] += connection_info.edge_length_dict[y2]/connection_info.edge_speed_dict[y2]
#                         self.distance_route[x2] += connection_info.edge_length_dict[y2]
#                         # print(x2)
#                         # print(y2)
#                         if x2 not in self.num_on_edges[y2]:
#                             self.num_on_edges[y2].append(x2)

#                 for x2 in [current,considering]:#self.nu_trips.keys():  
#                     self.sharing_route[x2] = 0  
#                     temp_ids = []
#                     for y2 in self.nu_trips[x2]:
#                         temp_ids = set(self.num_on_edges[y2] + list(temp_ids))
#                     self.sharing_route[x2] = len(list(temp_ids))
#                     self.route_information[x2] =[vehicles[x2].start_time,self.est_travel_time[x2],self.sharing_route[x2],self.num_edges[x2],self.distance_route[x2]]

#                 #rework was here









                   
                   

#                     # route_difference =current_prediction \
#                     #      - self.predictors.route_est(temp_trip_info)
#                     # #route_difference = sum(x2 - y2 for x2, y2 in zip(self.route_information[current],self.route_information[considering] ))
                    
#                     # print("route difference:",route_difference)
#                     # if route_difference > 0:
                       
#                         # temp.append(core.Run_id.Round_sufix)
#                         # temp.append(core.Run_id.run_id)
#                         # temp.append(current)
#                         # temp.append(considering)
#                         # temp.append(vehicles[current].start_time)
#                         # temp.append(vehicles[considering].start_time)
#                         # temp.append(0)
#                         # temp.append(0)
#                         # temp.append(0)
#                         # temp.append(0)
#                         # temp.append(0)
#                         # temp.append(0)
#                         # temp.append(0)
#                         # temp.append(0)
#                         # temp.append(The_edge)
#                         # temp.append(vehicles[current].deadline)
#                         # temp.append(vehicles[considering].deadline)

#                         # overall.append(temp)
#                         # temp = []
                      
                        
        
#         print("Number of vehicles that have switched:",num_switched)     
#         core.Run_id.Num_switched = num_switched
#         core.Run_id.trips = self.nu_trips
#         #print("finsihing up:",self.route_information)
#         core.Run_id.route_information = self.route_information
#         core.Run_id.Controller_version = "STRBE_REWORK"
#         core.Run_id.vehicle_num_options= route_options
#         #data2Csv_general(overall,'./History/switch_information.csv')


        
#     def dijkstra(self,vehicle, connection_info):
#         decision_list = []
#         unvisited = {edge: 1000000000 for edge in self.connection_info.edge_list} # map of unvisited edges
#         visited = {} # map of visited edges
#         current_edge = vehicle.current_edge
        
#         current_distance = self.connection_info.edge_length_dict[current_edge]/self.connection_info.edge_speed_dict[current_edge] 
#         unvisited[current_edge] = current_distance
#         path_lists = {edge: [] for edge in self.connection_info.edge_list} #stores shortest path to each edge using directions
#         while True:
#             if current_edge not in self.connection_info.outgoing_edges_dict.keys():
#                 continue
#             for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_edge].items():
#                 if outgoing_edge not in unvisited:
#                     continue
#                 edge_length = self.connection_info.edge_length_dict[outgoing_edge]/self.connection_info.edge_speed_dict[outgoing_edge]# self.edge_speed_dict
#                 new_distance = current_distance + edge_length
#                 if new_distance < unvisited[outgoing_edge]:
#                     unvisited[outgoing_edge] = new_distance
#                     current_path = copy.deepcopy(path_lists[current_edge])
#                     current_path.append(direction)
#                     path_lists[outgoing_edge] = copy.deepcopy(current_path)

#             visited[current_edge] = current_distance
#             del unvisited[current_edge]
#             if not unvisited:#fairly certain this means that there is no other route
#                 return []
#                 break
#             if current_edge==vehicle.destination:
#                 break
#             possible_edges = [edge for edge in unvisited.items() if edge[1]]
#             current_edge, current_distance = sorted(possible_edges, key=lambda x: x[1])[0]
            


#         for direction in path_lists[vehicle.destination]:
#             decision_list.append(direction)
#         #print("dijkstra:{}".format(decision_list))
#         return decision_list   
#     def make_decisions(self, vehicles, connection_info):
#         """
#         make_decisions algorithm uses Dijkstra's Algorithm to find the shortest path to each individual vehicle's destination
#         :param vehicles: list of vehicles on the map
#         :param connection_info: information about the map (roads, junctions, etc)
#         """
#         #print("what to heck")
#         local_targets = {}
#         #first_run = True
#         #updated = False
#         for vehicle in vehicles:
#             if vehicle.vehicle_id not in self.routes.keys():
               
                
#                 self.routes.update({vehicle.vehicle_id:self.get_directions(connection_info,self.nu_trips[vehicle.vehicle_id])})#self.dijkstra(vehicle, connection_info)})
                
#                 self.position.update({vehicle.vehicle_id:0})
#                 self.routes_edges.update({vehicle.vehicle_id:self.get_edges(vehicle, connection_info,self.routes[vehicle.vehicle_id])})
                
#                 Vehicle_route["heh"] = copy.deepcopy(self.routes_edges)
#                 #updated=True
                
            
#             while self.routes_edges[vehicle.vehicle_id][self.position[vehicle.vehicle_id]] != vehicle.current_edge:
#                 self.position[vehicle.vehicle_id] +=1
#                 if self.position[vehicle.vehicle_id] == len(self.routes_edges[vehicle.vehicle_id]):
#                     break
#                 if self.routes[vehicle.vehicle_id]:
#                     del self.routes[vehicle.vehicle_id][0]
#                 #updated=True
            
#             # if updated == True:
#             #real time routing
#             #     temp = []
#             #     #overall=[]  
#             #     temp.append(core.Run_id.Round_sufix)
#             #     temp.append(core.Run_id.run_id)
#             #     temp.append(vehicle.vehicle_id)
#             #     temp.append(vehicle.current_edge)
                
#             #     temp_est_travel_time = 0
#             #     temp_route_distance =0
#             #     temp_route_length = 0
#             #     temp_vehicles_ahead = 0
#             #     temp_vehicles_assigned_ahead = 0
#             #     ahead_route = self.routes_edges[vehicle.vehicle_id][self.position[vehicle.vehicle_id]:len(self.routes_edges[vehicle.vehicle_id])]
#             #     # print("current_edge:",vehicle.current_edge)
#             #     # print("remaining:",ahead_route)
#             #     temp_ids = []
#             #     for x in ahead_route:
#             #         temp_ids=  set(self.num_on_edges[x] + list(temp_ids))
#             #         #temp_vehicles_assigned_ahead += self.num_on_edges[x]
#             #         temp_route_length += connection_info.edge_length_dict[x]
#             #         temp_est_travel_time +=connection_info.edge_length_dict[x]/connection_info.edge_speed_dict[x]
#             #         temp_route_distance += connection_info.edge_length_dict[x]
#             #         temp_vehicles_ahead += traci.edge.getLastStepVehicleNumber(x)
#             #     temp_vehicles_assigned_ahead = len(list(temp_ids))
#             #         #self.route_information[x] =[self.est_travel_time[x],self.sharing_route[x],self.num_edges[x],self.distance_route[x]]
#             #     temp.append(temp_est_travel_time)
#             #     temp.append(self.sharing_route[vehicle.vehicle_id])
#             #     temp.append(len(ahead_route))
#             #     temp.append(temp_route_length)
#             #     temp.append(temp_vehicles_ahead)
#             #     temp.append(temp_vehicles_assigned_ahead)
#             #     temp.append(traci.simulation.getTime()-vehicle.start_time)
                
#             #     #overall.append(temp)
#             #     history.data_recorder.route_real_time_data.append(temp)

                
#             #     updated = False
          
#             decision_list = self.routes[vehicle.vehicle_id]   
#             local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)

#             if local_targets[vehicle.vehicle_id] not in self.routes_edges[vehicle.vehicle_id]:
#                 self.routes.update({vehicle.vehicle_id:self.dijkstra(vehicle, connection_info)})
#                 self.position.update({vehicle.vehicle_id:0})
#                 self.routes_edges.update({vehicle.vehicle_id:self.get_edges(vehicle, connection_info,self.routes[vehicle.vehicle_id])})
#                 Vehicle_route["heh"] = copy.deepcopy(self.routes_edges)
               
#                 decision_list = self.routes[vehicle.vehicle_id]   
#                 local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
#             #print(local_targets)
        
           
 
#         return local_targets
