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

class NewPolicy_Reader(RouteController):

    def get_edges(self,vehicle,connection_info,d_list):
        e_list = []
        c_edge = vehicle.current_edge#self.location[vehicle.vehicle_id]#self.decision[vehicle.vehicle_id]
        
        # print("vehicle:",vehicle.vehicle_id)#{}c_edge: {} decision_edge : {}".format(vehicle.vehicle_id,c_edge,self.decision[vehicle.vehicle_id]))
        # print("route:",self.trips[vehicle.vehicle_id])
        # print("directions,",self.routes[vehicle.vehicle_id])'
        # print("current location:",vehicle.current_edge)

        e_list.append(c_edge)
        for d in d_list:
            if d == c_edge:
                continue
            # print("c_edge: {} d: {}".format(c_edge,d))
            # print("c_edge: {}  d: {} -> to {}".format(c_edge,d,self.connection_info.outgoing_edges_dict[c_edge][d]))
            c_edge = self.connection_info.outgoing_edges_dict[c_edge][d]
            e_list.append(c_edge)
        return e_list
    
    routes = {} # = to route with regards to directions from current edge is retruend by make decisions
    location = {} # used to help with the operation of returning make decisions and routes
    position = {}
    routes_edges = {} # same as routes but has the explicit id for each edge (ow the edge)
    trips = {}
    
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
        self.routes = {} # = to route with regards to directions from current edge is retruend by make decisions
        self.location = {} # used to help with the operation of returning make decisions and routes
        self.position = {}
        self.routes_edges = {} # same as routes but has the explicit id for each edge (ow the edge)
        self.trips = {}
        #file_name = "static2.rou.xml"
        doc = minidom.parse("./configurations/Rounds/"+Round_name+'/'+file_name)
        #print("FILE NAME:",file_name)
        veh = doc.getElementsByTagName("vehicle")
        #self.
        #print(self.trips)
        #print("brudda")
        for t in veh:
            vid = t.getAttribute("id")
            route_tag = t.getElementsByTagName("route")[0]
            route = route_tag.getAttribute("edges")
            root = route.split(' ')
            self.trips[vid]  = root    
        trip_info = {}
        est_travel_time = {}
        distance_route={}
        num_edges = {}
        sharing_route = {}
        num_on_edges={}
        for x in self.trips.keys():
            est_travel_time[x] = 0
            distance_route[x] =0
            num_edges[x] = len(self.trips[x])

            for y in self.trips[x]:
                est_travel_time[x] += connection_info.edge_length_dict[y]/connection_info.edge_speed_dict[y]
                distance_route[x] += connection_info.edge_length_dict[y]
                if y not in num_on_edges.keys():
                    num_on_edges[y] = []
                if x not in num_on_edges[y]:
                    num_on_edges[y].append(x)

        for x in self.trips.keys():  
            sharing_route[x] = 0  
            temp_ids = []
            for y in self.trips[x]:
                #print(type(temp_ids))
                #print(type(num_on_edges[y]))
                temp_ids =  set(num_on_edges[y] + list(temp_ids))
            sharing_route[x] = len(list(temp_ids))
            #print()
            trip_info[x] =[est_travel_time[x],sharing_route[x],num_edges[x],distance_route[x]]
        
        core.Run_id.route_information = trip_info#=[est_travel_time[x],sharing_route[x],num_edges[x],distance_route[x]]
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
        #print("dijkstra:{}".format(decision_list))
        return decision_list   
    def make_decisions(self, vehicles, connection_info):
        """
        make_decisions algorithm uses Dijkstra's Algorithm to find the shortest path to each individual vehicle's destination
        :param vehicles: list of vehicles on the map
        :param connection_info: information about the map (roads, junctions, etc)
        """
        local_targets = {}
        first_run = True
        #print(self.trips)
       # print(test)
        #for x in connection_info:
          #  print()
        for vehicle in vehicles:
            #print("looking at: {}".format(vehicle.vehicle_id))
            if vehicle.vehicle_id not in self.routes.keys():
                self.routes.update({vehicle.vehicle_id:self.get_directions(connection_info,self.trips[vehicle.vehicle_id])})
                self.position.update({vehicle.vehicle_id:0})
                self.routes_edges.update({vehicle.vehicle_id:self.get_edges(vehicle, connection_info,self.routes[vehicle.vehicle_id])})
                #self.location.update({vehicle.vehicle_id:self.routes_edges[vehicle.vehicle_id][self.position[vehicle_id]]})
                Vehicle_route["heh"] = self.routes_edges
                Vehicle_route_direction["heh"] = self.routes
                #print("fuggin why:{}".format(Vehicle_route["heh"]))
                #print(connection_info.edge_list)
                #print(self.routes)
                #print(self.routes_edges)
                est = 0.0
                est2 = 0.0
                #print("vehicle{} Route {}".format(vehicle.vehicle_id,self.routes_edges[vehicle.vehicle_id]))
                for x in self.routes_edges[vehicle.vehicle_id]:#self.routes):
                    est += connection_info.edge_length_dict[x]/connection_info.edge_speed_dict[x]
                    if traci.edge.getTraveltime(x) > 1000:
                        est2 += connection_info.edge_length_dict[x]/connection_info.edge_speed_dict[x]
                    else:
                        est2 += traci.edge.getTraveltime(x)
                    #print(traci.edge.getTraveltime(x))
                    #print(est)
                estimated_time[vehicle.vehicle_id] = [est,est2]
               
            while self.routes_edges[vehicle.vehicle_id][self.position[vehicle.vehicle_id]] != vehicle.current_edge:
                self.position[vehicle.vehicle_id] +=1
                if self.position[vehicle.vehicle_id] == len(self.routes_edges[vehicle.vehicle_id]):
                    break
                if self.routes[vehicle.vehicle_id]:
                    del self.routes[vehicle.vehicle_id][0]

           
            decision_list = self.routes[vehicle.vehicle_id]   
            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
            if local_targets[vehicle.vehicle_id] not in self.routes_edges[vehicle.vehicle_id]:
                #print("{} took a wrong turn but why".format(vehicle.vehicle_id))
                self.routes.update({vehicle.vehicle_id:self.dijkstra(vehicle, connection_info)})
                #self.location.update({vehicle.vehicle_id:vehicle.current_edge})
                self.position.update({vehicle.vehicle_id:0})
                self.routes_edges.update({vehicle.vehicle_id:self.get_edges(vehicle, connection_info,self.routes[vehicle.vehicle_id])})
                Vehicle_route["heh"] = self.routes_edges
                Vehicle_route_direction["heh"] = self.routes
                est = 0.0
                est2 = 0.0
                #print("vehicle {} Route {}".format(vehicle.vehicle_id,self.routes_edges[vehicle.vehicle_id]))
                for x in self.routes_edges[vehicle.vehicle_id]:#self.routes):
                    est += connection_info.edge_length_dict[x]/connection_info.edge_speed_dict[x]
                    if traci.edge.getLastStepMeanSpeed == 0.0:
                        est2 += connection_info.edge_length_dict[x]/connection_info.edge_speed_dict[x]
                    else:
                        est2 += traci.edge.getTraveltime(x)
                    #print(traci.edge.getTraveltime(x))
                    #print(est)
                estimated_time[vehicle.vehicle_id] = [est,est2]
                decision_list = self.routes[vehicle.vehicle_id]   
                local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
            #print(local_targets)
        
           
 
        return local_targets
    
