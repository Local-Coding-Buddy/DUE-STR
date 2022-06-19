from xml.dom.minidom import parse, parseString
from lxml import etree
from datetime import datetime
run_id = ''
Num_switched = 0
Round_sufix = ''
current_time = 0
trips = {}
route_information = {} # passes
Controller_version = '' #another string passed into the output csv to denote which version of the controller is used
vehicle_num_options ={} # of form {vehicle_id:number_of_options_considered} Used to keep track of how many options 
#a vehicle has and whether they missed their deadline or not, for example a vehicle may have considerd 4 shared edges
rounds_directory = "./configurations/Rounds/" # holds where I store rounds that way all I need to do is change a single line
collector = False # boolean that denotes whether to collect route information during execution via STR SUMO meta collector


# cfg_file = './configurations/myconfig.sumocfg' #getting the contents of the config file
# with open(cfg_file) as xml:
#     root = etree.XML(xml.read())

# net_map = root.xpath('./input/net-file/@value')[0]
# timestamp = datetime.timestamp(datetime.now())


# run_id = net_map+":"+str(timestamp)
# print("yes hello sir!",run_id)

# def update_run_id():
#     cfg_file = './configurations/myconfig.sumocfg' #getting the contents of the config file
#     with open(cfg_file) as xml:
#         root = etree.XML(xml.read())
    
#     net_map = root.xpath('./input/net-file/@value')[0]
#     timestamp = datetime.timestamp(datetime.now())


#     run_id = net_map+":"+str(timestamp)
#     print("yes hello sir!",run_id)