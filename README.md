***Selfless Traffic Routing Plus (STR-SUMO-Plus)***

This is the development version used in the paper:Enhancing Dynamic User Equilibrium Utilizing the Selfless Traffic Routing Model

***High level over view***
From a high level this code: 
1 - generates trips considering the start and end edges in main.py as well as edge_dependcies.
2 - uses duarouter to make simple shortes path trips using Djikstra's algorithm. 
3 - uses duaiterate to take the output from the last step and split vehicles arcoss the network in order to equilize delay between vehicles
4 - takes output from the previous step and refines it prioritizing vehicles with lower deadlines to get higher quality route assignments

Routes are regarded by quality on 4 metrics: Estimated travel time, Number of vehicles sharing the edges encountered, The number of edges, The distance
(This is in the self.route_information dict in STRBE_controller) 

***Guide to experiments***

Currently the code should be set to run Round3 from the paper. If you run main you should end up with the exact same results as given in the paper.

**Switch Experiments**
To switch to another round: simply go to main.py and change the variable "Round_name" to the name of any existing round in the configurations folder. 
So for example if I wanted to re-run Round1 of the experiments I would change Round_name = "Round3" ->Round_name = "Round1" This gives us access to important files namely:
- static_trip.rou.xml (step1)
- str_sumo.rou.xml (step2 output)
- SUMO_Trip_Deadline_Data.csv (step2 output)

This brings us to step 2 from the previous section so what we need to do here is take the file generated in step 3 (in this case static2_090.rou.xml) and drag it into the configurations folder. 
Next we need to head back to main.py and change the variable "file_name2" to static2_090.rou.xml.

At this point you should be able to run main.py again and recreate the experiments from the paper. 

**Creating New Experiments**
Creating a new experiment is simular to switching experiments. 
Begin by going to Round_name and changing the value to what you wish to name the experiement. In this case I will use the name "duel_35".

Up to step 2 is automated currently now all that is left to do is generate a k-shortest trips file (provided by duarouter).
~~To ensure that step3 is automated please uncomment line 386 "file_name2 = using_duaIterate(file_name,Round_name)" in order for duatierate to run and generate a file.~~
Unfortunately due to duaiterate wanting to be run in the same directory as main (I literally can't find a fix please help) you will need to take the output file str_sumo.str.xml and the map file (in this case the map file is test_vehicle_only_4corners.net.xml) as defined by the net-file value in myconfig.sumocfg in the configurations file and place them into a independent file for duaiterate to work on.

In our case you can use either /Route_setup/Dua_temp or /configurations/duaiterate grinding in order to get the relevent file you will need. 
python duaIterate.py -t str_sumo.rou.xml  -n test_vehicle_only_4corners.net.xml -l 100
This command is from scripts.txt

Afterward duaiterate should run for 100 iterations with the last iteration being in the file 099,
retrieve the file str_sumo_099.rou.xml and place it in configurations.

Ensure that file_name2 in main.py is set to str_sumo_099.rou.xml and you should be able to run your new experiment.

**Notes:** 
-Duaiterate set to 100 iterations typically took around 3 minutes to run on my computer.
-The newest iteration of duaiterate will be used.
-This Duaiterate implementation is exetremely rough, since we plan on using something other than duaiterate I am not going to try and fix it at this point.

All relevant files will be moved to the relevenat round folder as well as the configurations folder. 

A rule of thumb here is the files that will be used are in configurations while the round folders are "saves" for their respective trips.
