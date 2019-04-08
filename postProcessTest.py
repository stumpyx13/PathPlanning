import csv
from matplotlib import pyplot as plt
import numpy as np

nodeList = list()
optimalPath = list()
obstacleList = list()
nodeListFile = 'nodesSampled_RRTstar.csv'
optimalPathFile = 'optimalPath_RRTstar.csv'
obstacleFile = 'obstacles.csv'

environmentFile = 'environment.csv'

with open(nodeListFile) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter = ',')
    for row in csv_reader:
        nodeList.append((float(row[0]), float(row[1])));

with open(optimalPathFile) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter = ',')
    for row in csv_reader:
        optimalPath.append((float(row[0]),float(row[1])));

with open(obstacleFile) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter = ',')
    for row in csv_reader:
        obstacleList.append((float(row[0]),float(row[1]),float(row[2]),float(row[3])))


with open(environmentFile) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter = ',')
    N_line = 0
    for row in csv_reader:
        if N_line == 0:
            x_env = float(row[0])
            y_env = float(row[1])
        elif N_line == 1:
            x_start = float(row[0])
            y_start = float(row[1])
        elif N_line == 2:
            x_goal_TL = float(row[0])
            y_goal_TL = float(row[1])
            dx_goal = float(row[2])
            dy_goal = float(row[3])
        N_line += 1

x_nodeList, y_nodeList = zip(*nodeList)

x_optimalPath, y_optimalPath = zip(*optimalPath)

#figure size
plt.figure(figsize=(16,16))
# plot optimal path
plt.plot(x_optimalPath,y_optimalPath,'b-*',linewidth=3)

# plot sampled nodes
plt.plot(x_nodeList,y_nodeList,'r.')

# plot start point
plt.plot(x_start,y_start,'g.', markersize=30)

# plot goal region
rect_goal = plt.Rectangle((x_goal_TL,y_goal_TL-dy_goal),dx_goal,dy_goal,color = 'g',alpha=0.3)
plt.gca().add_patch(rect_goal)

#plot obstacles
for ob in obstacleList:
    rect = plt.Rectangle((ob[0],ob[1]-ob[3]),ob[2],ob[3],color='r',alpha=0.3)
    plt.gca().add_patch(rect)
    #plt.add_patch(rect)
#    plt.plot((ob[0],ob[0]+ob[2]),(ob[1],ob[1]),'r-')
#    plt.plot((ob[0]+ob[2], ob[0]+ob[2]),(ob[1],ob[1]-ob[3]),'r-')
#    plt.plot((ob[0],ob[0]),(ob[1],ob[1]-ob[3]),'r-')
#    plt.plot((ob[0],ob[0]+ob[2]),(ob[1]-ob[3],ob[1]-ob[3]),'r-')

plt.xticks(np.arange(0,x_env,step=10))
plt.yticks(np.arange(0,y_env,step=10))
plt.show()
