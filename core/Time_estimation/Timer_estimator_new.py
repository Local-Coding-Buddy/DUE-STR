import pandas as pd
import numpy as np
import os
from sklearn.neighbors import KNeighborsRegressor
from sklearn.model_selection import train_test_split
from sklearn.metrics import r2_score




class Time_predictors():

    def __init__(self,net_map):
        Route_info =pd.read_csv("./core/Time_estimation/maps/"+net_map+"/With_start_time.csv")
         
       
        #####from non-real-time beginning route predition#####
        Route_info.replace([np.inf, -np.inf], np.nan, inplace=True)
        Route_info.dropna(inplace=True)

        X = Route_info[['start_time','Est_travel_time','#_Sharing_route',"Route_length"]]
        y = Route_info[['Travel_time']]

        #X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.3)

        self.Static_route_predictor = KNeighborsRegressor(n_neighbors=5)

      
        #self.Static_route_predictor.fit(X_train,y_train)
        self.Static_route_predictor.fit(X,y)

    def route_est(self,conditions):
        #TODO make sure input is in correct format
        # 'Est_travel_time','#_Sharing_route','#_of_edges',"Route_length"
        # conditions = np.array(conditions)
        # conditions.reshape(1,-1)
        overall=[]
        overall.append(conditions)
        #print(conditions)
        data = pd.DataFrame(overall,columns = ['start_time','Est_travel_time','#_Sharing_route',"Route_length"])
        return(self.Static_route_predictor.predict(data))
        pass
