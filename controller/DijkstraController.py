from controller.RouteController import RouteController
from core.Util import ConnectionInfo, Vehicle
import numpy as np
import traci
import math
import copy


class DijkstraPolicy(RouteController):
    
    def __init__(self, connection_info):
        super().__init__(connection_info)

    def get_edges(self,vehicle,connection_info,d_list):
        e_list = []
        c_edge = vehicle.current_edge#self.location[vehicle.vehicle_id]#self.decision[vehicle.vehicle_id]
        #print("vehicle:{}c_edge: {} decision_edge : {}".format(vehicle.vehicle_id,c_edge,self.decision[vehicle.vehicle_id]))
        e_list.append(c_edge)
        for d in d_list:
            #print("c_edge: {}d: {}".format(c_edge,d))
            #print("c_edge: {} d: {} -> to {}".format(c_edge,d,self.connection_info.outgoing_edges_dict[c_edge][d]))
            c_edge = self.connection_info.outgoing_edges_dict[c_edge][d]
            e_list.append(c_edge)
        return e_list

    def make_decisions(self, vehicles, connection_info):
        """
        make_decisions algorithm uses Dijkstra's Algorithm to find the shortest path to each individual vehicle's destination
        :param vehicles: list of vehicles on the map
        :param connection_info: information about the map (roads, junctions, etc)
        """
        local_targets = {}
        for vehicle in vehicles:
            #print("{}: current - {}, destination - {}".format(vehicle.vehicle_id, vehicle.current_edge, vehicle.destination))
            decision_list = []
            print("what the hell pox",self.connection_info)
            unvisited = {edge: 1000000000 for edge in self.connection_info.edge_list} # map of unvisited edges
            visited = {} # map of visited edges
            current_edge = vehicle.current_edge
            
            current_distance = self.connection_info.edge_length_dict[current_edge]
            unvisited[current_edge] = current_distance
            path_lists = {edge: [] for edge in self.connection_info.edge_list} #stores shortest path to each edge using directions
            while True:
                if current_edge not in self.connection_info.outgoing_edges_dict.keys():
                    continue
                for direction, outgoing_edge in self.connection_info.outgoing_edges_dict[current_edge].items():
                    if outgoing_edge not in unvisited:
                        continue
                    edge_length = self.connection_info.edge_length_dict[outgoing_edge]
                    new_distance = current_distance + edge_length
                    if new_distance < unvisited[outgoing_edge]:
                        unvisited[outgoing_edge] = new_distance
                        current_path = copy.deepcopy(path_lists[current_edge])
                        current_path.append(direction)
                        path_lists[outgoing_edge] = copy.deepcopy(current_path)
                        #print("{} + {} : {} + {}".format(path_lists[current_edge]))# direction, path_edge_lists[current_edge], outgoing_edge))

                visited[current_edge] = current_distance
                del unvisited[current_edge]
                if not unvisited:
                    break
                if current_edge==vehicle.destination:
                    break
                possible_edges = [edge for edge in unvisited.items() if edge[1]]
                current_edge, current_distance = sorted(possible_edges, key=lambda x: x[1])[0]
                #print('{}:{}------------'.format(current_edge, current_distance))
            #current_edge = vehicle.current_edge

            
            for direction in path_lists[vehicle.destination]:
                decision_list.append(direction)

            edge_route = self.get_edges(vehicle,connection_info,decision_list)
            est_travel_time = 0
            for x in edge_route:
                est_travel_time += traci.edge.getTraveltime(x)
            #Vehicle_route[vehicle.vehicle_id] = edge_route

            print("vehicle {} Estimated Travel Time:{}".format(vehicle.vehicle_id,est_travel_time))

            local_targets[vehicle.vehicle_id] = self.compute_local_target(decision_list, vehicle)
        return local_targets
    
