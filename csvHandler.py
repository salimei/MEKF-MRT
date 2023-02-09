


#class to handle csv read/write 


import pandas as pd

#CSV example 
"""
gyro = np.array([20, 30, 40])
acc = np.array( [1, 3, 4])
gps = np.array( [9, 0, 2])
print("gyro:" + str(acc))
csvHandler.store(acc, 'acc')
print(csvHandler.read( 0, 'acc'))
"""

#store columns in csv
def store(results_to_be_stored, column):
    df = pd.DataFrame(results_to_be_stored)
    df.to_csv(str(column) + '.csv')
    print("stored")


#returns the column we want as a numpy array 
def read(column_to_be_read, column):
    # yes, I know, a relative path would have been better 
    path = "C:/Users/Dell/McGillRocketTeam/avionics-2022/Arrays/" + str(column) +".csv"
    df = pd.read_csv(path, encoding= 'utf-8') #static
    df.to_numpy()
    #selecting columns
    return df[str(column_to_be_read)].to_numpy() 

