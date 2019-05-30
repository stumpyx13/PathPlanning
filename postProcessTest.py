import csv
from matplotlib import pyplot as plt
import numpy as np
import matplotlib.collections as mcoll
import matplotlib.path as mpath

def colorline(
    x, y, z=None, cmap=plt.get_cmap('copper'), norm=plt.Normalize(0.0, 1.0),
        linewidth=3, alpha=1.0):
    """
    http://nbviewer.ipython.org/github/dpsanders/matplotlib-examples/blob/master/colorline.ipynb
    http://matplotlib.org/examples/pylab_examples/multicolored_line.html
    Plot a colored line with coordinates x and y
    Optionally specify colors in the array z
    Optionally specify a colormap, a norm function and a line width
    """

    # Default colors equally spaced on [0,1]:
    if z is None:
        z = np.linspace(0.0, 1.0, len(x))

    # Special case if a single number:
    if not hasattr(z, "__iter__"):  # to check for numerical input -- this is a hack
        z = np.array([z])

    z = np.asarray(z)

    segments = make_segments(x, y)
    lc = mcoll.LineCollection(segments, array=z, cmap=cmap, norm=norm,
                              linewidth=linewidth, alpha=alpha)

    ax = plt.gca()
    ax.add_collection(lc)

    return lc


def make_segments(x, y):
    """
    Create list of line segments from x and y coordinates, in the correct format
    for LineCollection: an array of the form numlines x (points per line) x 2 (x
    and y) array
    """

    points = np.array([x, y]).T.reshape(-1, 1, 2)
    segments = np.concatenate([points[:-1], points[1:]], axis=1)
    return segments



nodeList = list()
optimalPath = list()
optimalPathVel = list();
obstacleList = list()
treeNodeConnections = list()
nodeListFile = 'nodesSampled_RRTstar.csv'
optimalPathFile = 'optimalPath_RRTstar.csv'
obstacleFile = 'obstacles.csv'
treeNodeFile = 'tree_RRTstar.csv'
environmentFile = 'environment.csv'

with open(treeNodeFile) as csv_file:
    csv_reader = csv.reader(csv_file,delimiter = ' ')
    for row in csv_reader:
        point1 = row[0].split(',')
        point2 = row[1].split(',')
        treeNodeConnections.append(((float(point1[0]),float(point1[1])),(float(point2[0]),float(point2[1]))))

# with open(nodeListFile) as csv_file:
#     csv_reader = csv.reader(csv_file, delimiter = ',')
#     for row in csv_reader:
#         nodeList.append((float(row[0]), float(row[1])));

with open(optimalPathFile) as csv_file:
    csv_reader = csv.reader(csv_file, delimiter = ',')
    for row in csv_reader:
        optimalPath.append((float(row[0]),float(row[1])))
        optimalPathVel.append((float(row[2])**2 + float(row[3])**2)/25)

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

#x_nodeList, y_nodeList = zip(*nodeList)

x_optimalPath, y_optimalPath = zip(*optimalPath)

# Normalize velocity
print(max(optimalPathVel))

#figure size
plt.figure(figsize=(10,10))

# #plot sampled nodes
# for connection in treeNodeConnections:
#     plt.plot([connection[0][0],connection[1][0]],[connection[0][1],connection[1][1]],'k',linewidth=0.2)

#plt.plot(x_nodeList,y_nodeList,'r.')

# plot optimal path
#plt.plot(x_optimalPath,y_optimalPath,'b-*',linewidth=3)
colorline(x_optimalPath,y_optimalPath,optimalPathVel, \
    cmap=plt.get_cmap('winter'), linewidth=3)

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
