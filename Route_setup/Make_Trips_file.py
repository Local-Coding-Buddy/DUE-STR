from xml.dom import minidom
from core import Util
import random
import pandas as pd
import subprocess
from history.data_recorder import *
import os
import shutil
import core.Run_id
import sys
import copy



def Create_trips(start_edges,end_edges,dependencies,num_trips_controlled,num_trips_uncontrolled,Round_name,even,net_file,refine,num_iterations):
    
    print("reminder that uncontrolled vehicles have not been implemented yet")
    os.mkdir("./configurations/Rounds/"+Round_name)
    deadlines= []
    vehicle_dict = {}
    release_time = 0
    release_period = .2 #TODO potentially use a range like .01 to 3 instead of a fixed value.   
    file_name = "str_sumo.rou.xml" # this should stay the same shouldn't have to change to dynamic I believe
    root = minidom.Document()
    xml = root.createElement('routes')
    xml.setAttribute('xmlns:xsi','"http://www.w3.org/2001/XMLSchema-instance"')
    xml.setAttribute('xsi:noNamespaceSchemaLocation','"http://sumo.dlr.de/xsd/routes_file.xsd"')
    root.appendChild(xml)

    for x in range(0,num_trips_controlled):
        start_iterator = random.randint(0,len(start_edges)-1)
        start_edge = start_edges[start_iterator]
        while True:
            end_iterator = random.randint(0,len(end_edges)-1)
            end_edge = end_edges[end_iterator]
            if end_edge not in dependencies[start_edge]:
                break
        ddl_now = random.randint(500,1000)
        deadlines.append([str(x+2),ddl_now])
        child = root.createElement('trip')
        child.setAttribute('id',str(x+2))
        child.setAttribute('depart',str(release_time))
        child.setAttribute('from',str(start_edge))
        child.setAttribute('to',str(end_edge))

        xml.appendChild(child)
        
        v_now = Util.Vehicle(str(x+2),start_edge, end_edge, float(release_time), int(ddl_now))
        release_time += release_period
        vehicle_dict[str(x+2)]=v_now
    #print("why are you empty:",deadLines2Push)
    data2Csv_Deadlines(deadlines,Round_name) #insert file name into this as argument

    xml_str = root.toprettyxml(indent ="\t") 
    
    with open(core.Run_id.rounds_directory+Round_name+'/static_trip.rou.xml',"w") as f:
        #print(xml_str)
        f.write(xml_str)
        f.flush()
        f.close


    subprocess.call(['./Route_setup/Dua_temp/duarouter.exe','-n',\
    './configurations/maps/'+net_file,'-r',\
    core.Run_id.rounds_directory+Round_name+'/static_trip.rou.xml','-o',\
    core.Run_id.rounds_directory+Round_name+'/str_sumo.rou.xml','-e','2000'])
    
    shutil.copy2(core.Run_id.rounds_directory+Round_name+'/str_sumo.rou.xml','./configurations')
    

    if not os.path.exists("./configurations/"+net_file):
        shutil.copy2('./configurations/maps/'+net_file,'./configurations')

    if refine == True:
        subprocess.call([
            'python', './Route_setup/Dua_temp/duaIterate.py', '-t', core.Run_id.rounds_directory+Round_name+'/str_sumo.rou.xml',  '-n', \
                './configurations/maps/'+net_file,'-l '+str(num_iterations) 
        ])
        print("and realize it's a miss")
        noomber = str(num_iterations-1)
        noomber_base = copy.deepcopy(noomber)
        print("noomber_base:",noomber_base)
        print("noomber:",noomber)
        while len(noomber) <3:
            noomber = '0'+noomber
        print("noomber:",noomber)
        shutil.copy2('./'+noomber_base+'/str_sumo_'+noomber+'.rou.xml','./configurations')
        shutil.copy2('./'+noomber_base+'/str_sumo_'+noomber+'.rou.xml','./configurations/Rounds/'+Round_name)

    return vehicle_dict

   

def read_vehicle_file(trip_file_name,Round_name,net_file):
    vehicle_dict = {}
    shutil.copy2(core.Run_id.rounds_directory+Round_name+'/str_sumo.rou.xml','./configurations')
    shutil.copy2(core.Run_id.rounds_directory+Round_name+'/str_sumo_099.rou.xml','./configurations')
    print(core.Run_id.rounds_directory+Round_name+'/'+trip_file_name)
    doc = minidom.parse(core.Run_id.rounds_directory+Round_name+'/'+trip_file_name)
    trips = doc.getElementsByTagName("trip")
    print("round name",Round_name)
    working_set = pd.read_csv(core.Run_id.rounds_directory+Round_name+'/SUMO_Trip_Deadline_Data.csv')
    x = working_set["vehicle_id"].values
    y = working_set["deadline"].values
    deadlines = {}
    for hero in range(len(x)):
        deadlines[x[hero]] = y[hero] 

    if not os.path.exists("./configurations/"+net_file):
        shutil.copy2('./configurations/maps/'+net_file,'./configurations')
    for x in trips:

        v_now = Util.Vehicle(str(x.getAttribute("id")),x.getAttribute("from"), \
            x.getAttribute("to"), float(x.getAttribute("depart")), deadlines[int(x.getAttribute("id"))])

        vehicle_dict[str(x.getAttribute("id"))] = v_now
    print(vehicle_dict['2'].current_edge)
    return vehicle_dict