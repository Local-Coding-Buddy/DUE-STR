import csv  

csv_data = []
data2Push = [] #new data aquired that will be pushed
deadLines2Push = [] # this is for make_Trips_file.py
shared_guide = {} # dict used to synchronize the data being pushed in str_sumo_meta and strbe reader
                #I want to make sure I only get the routes in which I actively compare and see if they are swapped or not.
route_real_time_csv = []
route_real_time_data = []
def csv2Data(file_name_and_directory):
    # with open('./history/SUMO_Trips_Data.csv','r') as f2:
    global csv_data
    global data2Push

    csv_data = []
    data2Push = []
    #print("is this really blank?",file_name_and_directory)
    with open(file_name_and_directory,'r',encoding='cp932', errors='ignore') as f2:
        reader = csv.reader(f2)
        #print(reader)
        for item in reader:
            #print(item)
            csv_data.append(item)

def csv2Data_route():
    # with open('./history/SUMO_Trips_Data.csv','r') as f2:
    global route_real_time_csv 
    global route_real_time_data 

    route_real_time_csv  = []
    route_real_time_data  = []
    #print("is this really blank?",file_name_and_directory)
    with open('./history/SUMO_Route_Data_real_time.csv','r') as f2:
        reader = csv.reader(f2)
        #print(reader)
        for item in reader:
            #print(item)
            route_real_time_csv.append(item)   
    

def data2Csv():
    with open('./history/SUMO_Trips_Data.csv', 'w',newline='') as f:
        global data2Push
        # using csv.writer method from CSV package
        #print("recording data2Csv")
        write = csv.writer(f)
        pushing = []
        #write.writerow(fields)
        print("checking output data2Push:",data2Push)
        if not csv_data:
            pushing = data2Push
        else:
            pushing = csv_data+data2Push
        #print("checking output pushing:",pushing)
        write.writerows(pushing)
        f.flush()
        f.close
        
        #data2Push = []

def data2Csv_Meta():
    with open('./history/SUMO_Trips_Data_Meta.csv', 'w',newline='') as f:
      
        # using csv.writer method from CSV package
        #print("why isn't this being pushed?")
        write = csv.writer(f)
        pushing = []
        #write.writerow(fields)
        #print("doota now:",csv_data)
        if not csv_data:
            pushing = data2Push
        else:
            pushing = csv_data+data2Push
        write.writerows(pushing)
        f.flush()
        f.close

def data2Csv_Static():
    with open('./history/SUMO_Single_Trip_Data.csv', 'w',newline='') as f:
      
        # using csv.writer method from CSV package
        write = csv.writer(f)
        pushing = []
        #write.writerow(fields)
        Csv_data_static = [["run_id","vehicle_id","starting_time","deadline","Starting_edge","Destination_edge"]] # this needs to be just the heads of the columns
        
        pushing = Csv_data_static+data2Push
        #print("POOOOOSHIUNG:",pushing)
        write.writerows(pushing)
        f.flush()
        f.close

def data2Csv_Deadlines(deadlines_pushing,Round_name=""):
    with open('./configurations/rounds/'+Round_name+'/SUMO_Trip_Deadline_Data.csv', 'w',newline='') as f:
       
        write = csv.writer(f)
        pushing = []
        #write.writerow(fields)
        Csv_data_static = [["vehicle_id","deadline"]] # this needs to be just the heads of the columns
       
        pushing = Csv_data_static+deadlines_pushing
        #print("POOOOOSHIUNG:",pushing)
        write.writerows(pushing)       
        # f.flush()
        f.close         
        #deadLines2Push = []

def Csv2data_Route_real_Time(directory):
    #with open('./history/SUMO_Route_Data_real_time.csv', 'w',newline='') as f:
    #with open(directory, 'r',newline='') as f:  
    global route_real_time_csv 
    global route_real_time_data 

    route_real_time_csv  = []
    route_real_time_data  = []
    #print("is this really blank?",file_name_and_directory)
    with open(directory,'r') as f:
        reader = csv.reader(f)
        #print(reader)
        for item in reader:
            #print(item)
            route_real_time_csv.append(item)   
        # using csv.writer method from CSV package
        

def data2Csv_Route_real_Time(directory):
    #with open('./history/SUMO_Route_Data_real_time.csv', 'w',newline='') as f:
    with open(directory, 'w',newline='') as f:  
        # using csv.writer method from CSV package
        write = csv.writer(f)
        pushing = []
        #write.writerow(fields)
        if not route_real_time_csv:
            pushing = route_real_time_data
        else:
            pushing = route_real_time_csv+route_real_time_data
        
        write.writerows(pushing)
        f.flush()
        f.close

def data2Csv_general(data_selected,directory):
    with open(directory, 'w',newline='') as f:
      
        # using csv.writer method from CSV package
        write = csv.writer(f)
        pushing = []
        #write.writerow(fields)
        if not csv_data:
            pushing = data_selected
        else:
            pushing = csv_data+data_selected
        # print("OOOOOOO")
        # print(type(pushing))
        # print(pushing)
        write.writerows(pushing)    
        f.flush()
        f.close    

def data2Csv_general_single(data_selected,directory):
    with open(directory, 'w',newline='') as f:
      
        # using csv.writer method from CSV package
        write = csv.writer(f)
        pushing = []
        #write.writerow(fields)
        if not csv_data:
            pushing = data_selected
        else:
            pushing = csv_data+data_selected
        
        write.writerow(pushing)      
        f.flush()
        f.close


def csv_intialize(directory,variables):
    with open(directory, 'w',newline='') as f:
        write = csv.writer(f)
        write.writerow(variables)      
        f.flush()
        f.close

def pass_travel_times(vehicle_finish_time):
    print("I am not FINISH!\n",vehicle_finish_time)
    for x in route_real_time_data:
        print("Travel_time_pass_test:",x)
        x.append(vehicle_finish_time[x[2]])