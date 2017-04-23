#!/usr/bin/python

from matplotlib.colors import colorConverter
import numpy as np
import matplotlib as mpl
from pylab import *
from Queue import *
import rospy
from std_msgs.msg import *
from time import sleep
import threading

#grid=np.loadtxt("grid.txt")

#need a class representing a grid.

#should tie this in with the format of the camera mapped grid, either by merging the code or by including the information in a text file.
#for now will set up manually
#code built from example at redblobgames

diagonal_cost=0.75
cell_cost=0
explore=0

class MapGrid:
    def __init__(self, grid, cellsize=1, diagonal_cost=3, neighbour_cost=2):    #build grid from a given file????
        self.grid = grid
        self.width = len(self.grid[0])
        self.height = len(self.grid)
        self.cellsize=cellsize
        self.neighbour_cost=neighbour_cost
        self.diagonal_cost=diagonal_cost    
    
    def in_bounds(self, id):
        (x, y) = id
        return 0 <= x < self.width and 0 <= y < self.height 

    def passable(self, id):
        (x, y) = id
        return self.in_bounds((x,y)) and self.grid[x][y] < 0.75  # simple iteration for 0,1 grid

    def neighbours(self, id):
        (x, y) = id
        results = [(x-1,y+1),(x,y+1),(x+1,y+1),(x-1,y),(x+1,y),(x-1,y-1),(x,y-1),(x+1,y-1)]
        results=filter(self.in_bounds, results)
        results=filter(self.passable, results)
        return results

    def allneighbours(self,id):
        x,y=id
        results = [(x-1,y+1),(x,y+1),(x+1,y+1),(x-1,y),(x+1,y),(x-1,y-1),(x,y-1),(x+1,y-1)]
        results=filter(self.in_bounds, results)
        return results


    def setgridval(self, id, value):
        (x, y) = id
        self.grid[x][y]=value

    def getgridval(self, id):
        (x, y) = id
        return self.grid[x][y]

    def pad_obstacles(self):
        obstacles = []
        neighbours = []
        for x in range(0, self.width):
            for y in range(0, self.height):
                if self.getgridval((x,y)) == 1.0:
                    obstacles.append((x,y))
        for o in obstacles:
            n = self.neighbours(o)
            for neighbour in n:
                neighbours.append(neighbour)
                self.setgridval(neighbour, 0.8)
        for n in neighbours:
            n1=self.neighbours(n)
            for ni in n1:
                if self.getgridval(ni) < 0.5:
                    self.setgridval(ni,0.5)

    def set_neighbours(self, thresh, val):
        cells=[]
        for x in range(0, self.width):
            for y in range(0, self.height):
                if self.getgridval((x,y)) == thresh:
                    cells.append((x,y))
        for c in cells:
            neighbours = self.neighbours(c)
            for n in neighbours:
                if self.getgridval(n) < val:
                   self.setgridval(n,val)

    def show(self):
        mpl.pyplot.imshow(self.grid, interpolation='none', origin='lower')
        #ax=gca()
        #ax.set_ylim(ax.get_ylim()[::-1])  #two lines to flip the y axis as pyplot by default places 0,0 at the top left corner
        mpl.pyplot.show()

    def cost(self, id1, id2):
        (x1, y1) = id1
        (x2, y2) = id2
        cost=1
        cost+=(self.getgridval((x2,y2)))*cell_cost
        #for n in self.allneighbours((x2,y2)):
        #    if self.getgridval(n)>0:
        #        cost+=neighbour_cost
        if x1 == x2 or y1 == y2:
            return cost
        else:
            cost+=diagonal_cost
            return cost

    def convert(self, id):
        (y,x) = id
        x-=(self.width/2)
        x*=self.cellsize
        y*=self.cellsize
        return(x,y)

    def backconvert(self, id):
        x,y=id
        x = int(x/self.cellsize)
        y = int(y/self.cellsize)
        x+=(self.width/2)
        return(x,y)
    

def breadth_first_search(graph, start, goal):
    frontier = Queue()
    frontier.put(start)
    came_from = {}
    came_from[start] =  None
    count = 0    

    while not frontier.empty():
        current = frontier.get()
        
        if current == goal:
            break
        for next in graph.neighbours(current):
            if next not in came_from:
                frontier.put(next)
                came_from[next] = current
    
    ##if path not found?? consider (a) how to tell and (b) robot behaviour  

    return reconstruct_path(came_from, start, goal)

def dijkstra_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0

    while not frontier.empty():
        current = frontier.get()
        #print "Visiting ", current

        if current == goal:
            break
        
        for next in graph.neighbours(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                priority=new_cost
                frontier.put(next, priority)
                came_from[next] = current
                cost_so_far[next] = new_cost 
     
    return reconstruct_path(came_from, start, goal)

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return sqrt((x1 - x2)*(x1-x2) + (y1 - y2)*(y1-y2))

def a_star_search(graph, start, goal):
    frontier = PriorityQueue()
    frontier.put((0, start))
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        current=current[1]
        if current == goal:
            break
        
        for next in graph.neighbours(current):
            new_cost = cost_so_far[current] + graph.cost(current, next)
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put((priority, next))
                came_from[next] = current
    
    return reconstruct_path(came_from, start, goal)
    
def reconstruct_path(came_from, start, goal):
    current = goal
    path = [current]
    while current != start:
        try:
            current = came_from[current]
            path.append(current)
        except KeyError as e:
            print "Path not found"
            return [robot_pos]
    #path.append(start)
    path.reverse()
    return path

def path_to_coords(path):
    last_slope=0
    coords=[]
    for p in range(0,(len(path)-1)):
        n = slope(path[p],path[p+1])
        if n!=last_slope:
            coords.append(path[p])
            last_slope=n
    coords.append(path[-1])
    return coords   
    

def slope(id1, id2):
    (x1,y1)=id1
    (x2,y2)=id2
    if x2==x1:
        return 100
    m=(y2-y1)/(x2-x1)
    return m


def visualise(m,n):
    # generate the colors for your colormap
    color1=colorConverter.to_rgba('white')
    color2 = colorConverter.to_rgba('red')

    # make the colormaps
    cmap1 = mpl.colors.LinearSegmentedColormap.from_list('my_cmap',['green','blue'],256)
    cmap2 = mpl.colors.LinearSegmentedColormap.from_list('my_cmap2',[color1,color2],256)

    cmap2._init() # create the _lut array, with rgba values

    # create your alpha array and fill the colormap with them.
    # here it is progressive, but you can create whathever you want
    alphas = np.linspace(0, 1, cmap2.N+3)
    cmap2._lut[:,-1] = alphas

    img2 = plt.imshow(m.grid, interpolation='nearest', cmap='gray_r', origin='lower')
    img3 = plt.imshow(n.grid, interpolation='nearest', cmap=cmap2, origin='lower')

    plt.show()
    plt.pause(0.0001)
    sleep(0.1)
    plt.close()


def talker(route):
    pub = rospy.Publisher('route', String, queue_size=10)
    rospy.init_node('router', anonymous=True)
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        route_str=" ".join([str(r) for r in route])
        rospy.loginfo(route_str)
        pub.publish(route_str)
        rate.sleep()

def user_callback(data):
    global user_pos
    message = data.data.strip().split()
    message = map(float, message)
    user_pos = [message[0], message[1]]
    #print("Position Received", user_pos)

def robot_callback(data):
    global robot_pos
    global route_flag
    message = data.data.strip().split()
    message = map(float, message)
    #rospy.loginfo(message)
    robot_pos = [message[0], message[1]]
    collision=message[2]
    #print "Robot Received", robot_pos
    if message[2]!=0 and np.linalg.norm(robot_pos) > 200:
        print "obstacle at ", robot_pos
        rospy.loginfo(message)
        (y,x)=m.backconvert(robot_pos)
        m.setgridval((x,y), 0.7)
        m.set_neighbours(0.7, 0.5)
        
    elif message[2]==0:
        print robot_pos," clear"

        (y,x)=m.backconvert(robot_pos)
        m.setgridval((x,y), 0.5*m.getgridval((x,y)))
    
    route_flag=message[3] 

class robot_position_thread(threading.Thread):
    def __init__(self, threadID, name, counter):
        threading.Thread.__init__(self)
        self.threadID=threadID
        self.name = name
        self.counter = counter
    def run(self):
        global m
        global n
        print "Starting " +self.name
        
        # generate the colors for your colormap
        color1=colorConverter.to_rgba('white')
        color2 = colorConverter.to_rgba('red')
        # make the colormaps
        cmap1 = mpl.colors.LinearSegmentedColormap.from_list('my_cmap',['green','blue'],256)
        cmap2 = mpl.colors.LinearSegmentedColormap.from_list('my_cmap2',[color1,color2],256)
        cmap2._init() # create the _lut array, with rgba values
        # create your alpha array and fill the colormap with them.
        # here it is progressive, but you can create whathever you want
        alphas = np.linspace(0, 1, cmap2.N+3)
        cmap2._lut[:,-1] = alphas
        
        while(1):
            img2 = plt.imshow(m.grid, interpolation='nearest', cmap='gray_r', origin='lower')
            img3 = plt.imshow(n.grid, interpolation='nearest', cmap=cmap2, origin='lower')
           
            plt.pause(0.001)
            plt.gcf().show()
        plt.close()
        print"Exiting " +self.name

if __name__ == "__main__":
    
    route_flag=2    
    rospy.init_node('router', anonymous = True)

    user_pos=[0,0]
    subscriber = rospy.Subscriber("user_pos", String, user_callback)
    
    robot_pos = [0, 0]
    subscriber=rospy.Subscriber("robot_info", String, robot_callback)

    publisher = rospy.Publisher("route", String, queue_size=1)

    grid = np.array(np.loadtxt("grid.txt",skiprows=1))
   
    fin=open("grid.txt","r")
    line=fin.readline()
    arr=line.strip().split(",")
    cell=float(arr[1])

    fin.close()

    m = MapGrid(grid,cellsize=cell)

    #m.pad_obstacles() #pad obstacles to account for robot diameter
    m.set_neighbours(1,0.9)
    m.set_neighbours(0.9, 0.6)  
    m.set_neighbours(0.6,0.3)
    
    #m.setgridval((19,21),1)

    grid1=np.zeros((40,40))
    n=MapGrid(grid1,cellsize=cell)


    threads=[]
    robot_thread1=robot_position_thread(1, "Robot polling thread", 1)
    threads.append(robot_thread1)
    robot_thread1.start()

    try:   
        while not rospy.is_shutdown():
            if route_flag == 2:
                print("from dock")
                (x, y)=m.backconvert((0,500))
                start=(x,y)
            else:
                (x,y)=m.backconvert(robot_pos) #FIX THIS!!!!!
                print("from ", robot_pos)
                start = (x,y)
      
            if(route_flag == 1): # destination reached, route home
                print("to dock")
                (y,x) = m.backconvert((0,500))
                goal=(x,y)
            else:
                print("to ", user_pos)
                (y,x) = m.backconvert(user_pos)
                goal=(x,y)        

            path = a_star_search(m, start, goal)
            #print path    
        
            ##need to display path without altering grid
            ##could overlay a second grid??
            grid1=np.zeros((40,40))
            n=MapGrid(grid1,cellsize=cell)
    
            if path != 0:
                for v in path:
                    n.setgridval(v, 1)

            coords=path_to_coords(path)
            #print coords

            #visualise(m,n)
            #for i in range(len(coords)):
            #    coords[i]=m.convert(coords[i])

            #fout=open("path.txt", "w")
            #for c in coords:
            #    fout.write(str(c[0])+", "+str(c[1])+"\n")
            #fout.close()
            #m.show()

            message_vals=[]
            for coord in coords:
                coord=m.convert(coord)
                message_vals.append(coord[0])
                message_vals.append(coord[1])
            message1 = " ".join([str(i) for i in message_vals])
            #message1="0 2500 -1500 2500"
            #rospy.loginfo(message1)
            publisher.publish(message1)
    except KeyboardInterrupt:
        robot_thread1.join()
        rospu.signal_shutdown()
