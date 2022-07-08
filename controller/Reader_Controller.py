from controller.RouteController import RouteController
from core.Util import ConnectionInfo, Vehicle
import numpy as np
import traci
import math
import copy
from core.Estimation import *
from core.STR_SUMO import *
import core.Run_id
from xml.dom import minidom

#this is just dijkstra's but with my saving of the route instead of recalculating it every time bit I did in selflesscontroller.

class Reader(RouteController):

    def get_edges(self,vehicle,connection_info,d_list):
        e_list = []
        c_edge = vehicle.current_edge#self.location[vehicle.vehicle_id]#self.decision[vehicle.vehicle_id]
        

        e_list.append(c_edge)
        for d in d_list:
            if d == c_edge:
                continue
            # print("c_edge: {} d: {}".format(c_edge,d))
            # print("c_edge: {}  d: {} -> to {}".format(c_edge,d,self.connection_info.outgoing_edges_dict[c_edge][d]))
            c_edge = self.connection_info.outgoing_edges_dict[c_edge][d]
            e_list.append(c_edge)
        return e_list
    
    
    
    def get_directions(self,connection_info,c_list):
        d_list =[] #direction list
        y=0
        #print("c_list: {}".format(c_list))
        for x in range(1,len(c_list)):
            for x2 in connection_info.outgoing_edges_dict[c_list[y]].items():
                if x2[1] == c_list[x]:
                    d_list.extend([x2[0]])
                    break
            y+=1
        #print("here")
        #print("take a look at this D ... list: {}".format(d_list))
        return d_list        
            

    def __init__(self, connection_info,file_name,Round_name):
        super().__init__(connection_info)
        self.directions = {} # = to route with regards to directions from current edge is retruend by make decisions 
        #{id:[ route in terms of directions]}
        self.location = {} # used to help with the operation of returning make decisions and routes
        self.position = {}
        self.trips = {} # form {vehicle_id:[route by edges]}
        #self.route_edges # same as trips but we will be cutting this up and removing entries as a result want to keep seperate
        #from self.trips
        doc = minidom.parse("./configurations/Rounds/"+Round_name+'/'+file_name)
        #print("FILE NAME:",file_name)
        veh = doc.getElementsByTagName("vehicle")
        for t in veh:
            vid = t.getAttribute("id")
            route_tag = t.getElementsByTagName("route")[0]
            route = route_tag.getAttribute("edges")
            root = route.split(' ')
            self.trips[vid]  = root    
        
        core.Run_id.trips = self.trips
        core.Run_id.Controller_version = "N/A"
        


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

        return decision_list   
    def make_decisions(self, vehicles, connection_info):
        """
        make_decisions algorithm uses Dijkstra's Algorithm to find the shortest path to each individual vehicle's destination
        :param vehicles: list of vehicles on the map
        :param connection_info: information about the map (roads, junctions, etc)
        """
        local_targets = {}
        for vehicle in vehicles:
            #print("looking at: {}".format(vehicle.vehicle_id))
            if vehicle.vehicle_id not in self.directions.keys():
                self.directions.update({vehicle.vehicle_id:self.get_directions(connection_info,self.trips[vehicle.vehicle_id])})#self.dijkstra(vehicle, connection_info)})
                #self.routes_edges.update({vehicle.vehicle_id:self.get_edges(vehicle, connection_info,self.routes[vehicle.vehicle_id])})
                self.position.update({vehicle.vehicle_id:0})
               #print("vehicle:",vehicle.vehicle_id,' route(directions): ',self.directions)
                
               
            while self.trips[vehicle.vehicle_id][self.position[vehicle.vehicle_id]] != vehicle.current_edge:
                self.position[vehicle.vehicle_id] +=1
                if self.position[vehicle.vehicle_id] == len(self.trips[vehicle.vehicle_id]):
                    break
                if self.directions[vehicle.vehicle_id]:
                    del self.directions[vehicle.vehicle_id][0]

        
            decision_list = self.directions[vehicle.vehicle_id]   
            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
            if local_targets[vehicle.vehicle_id] not in self.trips[vehicle.vehicle_id]:
                self.directions.update({vehicle.vehicle_id:self.dijkstra(vehicle, connection_info)})
                self.position.update({vehicle.vehicle_id:0})
                self.trips.update({vehicle.vehicle_id:self.get_edges(vehicle, connection_info,self.directions[vehicle.vehicle_id])})
                
               

                decision_list = self.directions[vehicle.vehicle_id]   
                local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
            #print(local_targets)
        
           
 
        return local_targets
    
