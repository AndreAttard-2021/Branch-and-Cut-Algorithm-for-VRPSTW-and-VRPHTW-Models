import gurobipy as gp
import csv
from numpy import genfromtxt


Statement = input("Are soft time windows being\
 considered? (Yes/No):")
if Statement =="YES" or Statement=="Yes" or Statement=="yes":
    soft = True
else: 
    soft = False


# Vector of Nodes (N)
addresses = []
with open("nodes.csv", "r") as file:
   reader = csv.reader(file)
   nodes_data = list(reader)
for row in nodes_data:
   addresses.append(row[0])

# Vector of Demands (d_{i})
d = []
with open("Demand.csv", "r") as file:
    reader = csv.reader(file)
    demand_data = list(reader)
for row in demand_data:
    d.append(int(row[0]))

# Matrix of Driving Times (t_{ij}/c_{ij})
t = genfromtxt('Distances.csv', delimiter=',')

# Matrix of Cost {c_{ij}}
c = genfromtxt('Cost.csv', delimiter=',')

# Vector of Service Times (h_{i})
h = []
with open("ServiceTimes.csv", "r") as file:
    reader = csv.reader(file)
    ServiceTimes_data = list(reader)
for row in ServiceTimes_data:
    h.append(int(row[0]))

# Vector of Ready Times (r_{i})
r = []
with open("Ready.csv", "r") as file:
    reader = csv.reader(file)
    ReadyTime_data = list(reader)
for row in ReadyTime_data:
    r.append(int(row[0]))

# Matrix of Due Times (f_{i}) 
f = []
with open("Due.csv", "r") as file:
    reader = csv.reader(file)
    DueTime_data = list(reader)
for row in DueTime_data:
    f.append(int(row[0]))

# Vector of Setup Cost (s_{i})
s = []
with open("SetupCost.csv", "r") as file:
    reader = csv.reader(file)
    SetupCost_data = list(reader)
for row in SetupCost_data:
    s.append(int(row[0]))

# Defining the Model Parameters
print("Soft Time Windows:", soft)
numberofaddresses = len(addresses)-2
print("Total number of addresses:", numberofaddresses)
numberofvehicles = len(s)  # Defining A
print("Total number of vehicles:", numberofvehicles)
vehiclecapacity = 1000  # Defining m_{a}
print("Vehicle capacity:", vehiclecapacity,\
      "Packages,Messages or Letters")
vehiclewaitpenalty = 0.5  # Wait Penalty w
print("Vehicle Wait Penalty:", vehiclewaitpenalty)
vehicledelaypenalty = 2.5 # Delay Penalty e
print("Vehicle Delay Penalty:", vehicledelaypenalty)
DepotDueTime = f[0]  # Define f_{0}*
print("Due Time of Depot:", DepotDueTime)

# Defining the Model
CVRPTW = gp.Model(name="Capacitated Vehicle Routing\
 Problem with Time Windows")

# Defining the Decision Variables x_{ija}
x = {}
for i in range(numberofaddresses + 2):
    for j in range(numberofaddresses + 2):
        for a in range(1, numberofvehicles + 1):
            x[i, j, a] = CVRPTW.addVar\
                (vtype=gp.GRB.BINARY,\
                 name='x' +str(i) \
                + ',' + str(j) + ',' + str(a))

# Defining the Decision Variables recording Arrival Times Z_{ia} 
Z = {}
for i in range(numberofaddresses + 2):
    for a in range(1, numberofvehicles + 1):
        Z[i, a] = CVRPTW.addVar\
            (vtype=gp.GRB.CONTINUOUS,\
             lb=0, name='Z' + str(i) + ',' + str(a))
    
# Defining the Decision Variables recording\
# Earliness/Lateness at Nodes in case of Soft Time Windows
if (soft == True): 
    delta = {}
    gamma = {}
    for i in range(numberofaddresses + 2):
        if (i != 0):
            delta[i] = CVRPTW.addVar\
                (vtype=gp.GRB.CONTINUOUS,\
                 lb=0, name='delta' + str(i))
        if (i !=(numberofaddresses +1)):
            gamma[i] = CVRPTW.addVar\
                (vtype=gp.GRB.CONTINUOUS,\
                 lb=0, name='gamma' + str(i))

# Defining the Constraints

# x_{iia} = 0 for all i
for i in range(numberofaddresses + 2):
    for a in range(1, numberofvehicles + 1):
        CVRPTW.addConstr(x[i, i, a] == 0)

# Constrant Set 1 (Customer Visits)
for j in range(1, numberofaddresses + 1):
    CVRPTW.addConstr((gp.quicksum(x[i, j, a] \
                    for i in range(numberofaddresses + 1) \
                    for a in range\
                    (1, numberofvehicles + 1))) == 1)

# Constraint Set 2 (Limiting Capacity of Vehicles)
for a in range(1, numberofvehicles + 1):
    CVRPTW.addConstr((gp.quicksum(d[i] * x[i, j, a] \
                    for i in range(numberofaddresses + 1) \
                    for j in range(\
                   1,numberofaddresses + 2))) <= vehiclecapacity)

# Constraint Set 3 (Depot Visits - Routes must start at Start Depot 0 and end at Return Depot numberofaddresses+1)
for a in range(1, numberofvehicles + 1):
    CVRPTW.addConstr((gp.quicksum(x[0, j, a]  \
    for j in range(\
    numberofaddresses + 2))) <= 1)
    CVRPTW.addConstr((gp.quicksum(\
        x[j, numberofaddresses + 1, a] \
        for j in range(numberofaddresses + 2))) <= 1)

# Constraint set 4 (Cohesive Routes)
for k in range(1, numberofaddresses + 1):
    for a in range(1, numberofvehicles + 1):
        CVRPTW.addConstr(gp.quicksum(\
            x[i, k, a] for i in range\
            (numberofaddresses + 1)) \
             - gp.quicksum(\
            x[k, j, a] for j in range\
            (1, numberofaddresses + 2)) == 0)

# Constraint Set 5 (Subtour Elimination Constraints)
for i in range(numberofaddresses + 1):
    for j in range(1, numberofaddresses + 2):
        for a in range(1, numberofvehicles + 1):
            if (soft == True):
                CVRPTW.addConstr(\
                Z[i, a] + t[i, j] + h[i] + \
                (f[0] + t[i, j] + h[i]) * (x[i, j, a] - 1) \
                <= Z[j, a])
                # M1_ij is an upper bound on Z[i,a]\
                # + t[i,j] + h[i] - Z[j,a]
                CVRPTW.addConstr(\
                Z[i, a] + t[i, j] + h[i] + \
                (t[i, j] + h[i] - f[0]) * (x[i, j, a] - 1) \
                >= Z[j, a])
                # M2_ij is a lower bound on Z[i,a]\
                # + t[i,j] + h[i] - Z[j,a]
            else:
                CVRPTW.addConstr(\
                    Z[i, a] + t[i, j] + h[i] +\
                    (f[i] + t[i, j] + h[i] - r[j])\
                    * (x[i, j, a] - 1)\
                    <= Z[j, a])


# Constraint Set 6 (Hard Time Windows)
if (soft == False):
    for i in range(numberofaddresses + 2):
        for a in range(1, numberofvehicles + 1):
            CVRPTW.addConstr(r[i] <= Z[i,a])
            CVRPTW.addConstr(f[i] >= Z[i,a])

# Constraint Set 6a (Soft Time Windows)
if (soft == True):
    for i in range(numberofaddresses + 2):
        for a in range(1, numberofvehicles + 1):
            if (i != 0): 
                CVRPTW.addConstr(delta[i] >=\
                (Z[i, a] - f[i]))
            if (i != (numberofaddresses +1)):
                CVRPTW.addConstr(gamma[i] >=\
                r[i] - Z[i,a])
                

    
# Defining the Objective Function
objective = gp.quicksum(s[a - 1] * x[0, j, a] \
            for j\
            in range(1, numberofaddresses + 1) \
            for a\
            in range(1, numberofvehicles + 1)) \
            + gp.quicksum(c[i, j] * x[i, j, a] \
            for i\
            in range(numberofaddresses + 1) \
            for j\
            in range(1, numberofaddresses + 2) \
            for a\
            in range(1, numberofvehicles + 1))



if (soft == True): 
    objective += gp.quicksum(vehiclewaitpenalty\
                * gamma[i] for i in range\
                (1, numberofaddresses + 1)) \
                 + gp.quicksum(vehicledelaypenalty\
                * delta[i]\
                for i in range(1,numberofaddresses + 2))

#Setting Time Limit
CVRPTW.Params.TimeLimit=14400

# Solving the model
CVRPTW.ModelSense = gp.GRB.MINIMIZE
CVRPTW.setObjective(objective)
CVRPTW.update()
CVRPTW.optimize()



# Printing non-zero variables
for v in CVRPTW.getVars():
    if (v.x != 0):
        print('%s %g' % (v.varName, v.x))
print('Obj: %g' % CVRPTW.objVal)

for a in range(1, numberofvehicles + 1):
    route = []
    arrival_times = []
    ready_times = []
    due_times = []
    stop_no = 0
    arrival_times.append(\
    round(CVRPTW.getVarByName\
    ('Z' + str(stop_no) + ',' + str(a)).x,2))
    ready_times.append(r[stop_no])
    due_times.append(f[stop_no])
    for j in range(numberofaddresses + 2):
        arc = CVRPTW.getVarByName(\
        'x' + str(stop_no)\
        + ',' + str(j) + ',' + str(a))
        if (arc.x > 0):
            route.append(0)
            route.append(j)
            stop_no = j
            arrival_times.append\
            (round(CVRPTW.getVarByName\
            ('Z' + str(stop_no) + ',' + str(a)).x,2))
            ready_times.append(r[stop_no])
            due_times.append(f[stop_no])
            break
    if (stop_no != 0):
        while (stop_no != (numberofaddresses + 1)):
              for j in range(numberofaddresses + 2):
                  arc = CVRPTW.getVarByName\
                      ('x' + str(stop_no) + ','\
                       + str(j) + ',' + str(a))
                  if (arc.x > 0):
                    stop_no = j
                    route.append(j)
                    arrival_times.append\
                    (round(CVRPTW.getVarByName\
                    ('Z' + str(j) + ',' + str(a)).x,2))
                    ready_times.append(r[stop_no])
                    due_times.append(f[stop_no])
                    break
    if (len(route) > 0):
        print('\nRoute', a, ':', route)
        print('Arrival Times:', arrival_times)
        print('Ready Times:', ready_times)
        print('Due Times:', due_times)


