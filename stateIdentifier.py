import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

#data classes
class Par:
    def __init__(self,n,v):
        self.name = n
        self.value = v
        
    def show(self):
        print(self.name,":",self.value,"\n")
        
class File:
    def __init__(self,t1,t2):
        self.tag1 = t1
        self.tag2 = t2
        self.values = []
        self.size = 0;
    def app(self,n,v):
        self.values.append(Par(n,v))
        self.size+=1
    def show(self):
        for i in range(self.size):
            self.values[i].show()
            
        print("\n\n")
        
#constants
numFiles = 999
data = []

#functions
def readData(file,line,index):
    for j in range(line):
        buffer = file.readline(500)
        colon = 0;
        num = ""
        name = ""
        flag = 0
        #read buffer in order to get a number
        for k in range(100):
            if colon==0 and buffer[k] != ':':
                name += buffer[k]
            if buffer[k] == ':':
                colon = k
                flag = 1
            if flag==1 and k>colon+1:
                if buffer[k] == "\n":
                    break
                else:
                    num += buffer[k]

        #now that we have a number, cast it to a float
        data[-1].app(name,float(num))
        
def readFile(file,i):
    #skip line
    file.readline(30)
    file.readline(30)
    readData(file,9,i)
    #skip line
    file.readline(30)
    
    readData(file,3,i)
    file.close()


#main body of code
for i in range(numFiles):
    #open file     
    filename ="./NetBeansProjects/variation/dist/Debug/300bhighdamp/parameters" + str(i) + ".txt"
    file = open(filename,"r")
    
    #get tags
    line = file.readline(50)
    t1 = line[6]
    t2 = line[8]
    if (t1=='|'):
        t2 = line[7]
    #check if there are enough boids for a good data sample
    if t1=='E':
        continue
    
    data.append(File(t1,t2))
    
    readFile(file,i)
    
#graph!
fig = plt.figure(figsize = (8,7))
ax = fig.add_subplot(111, projection='3d')
plt.rcParams.update({'font.size': 10})

#for the legend
labelguide = [1,1,1,1,1,1]

for i in range(len(data)):
    #get the three points starting with D_align
    p1 = data[i].values[6]
    p2 = data[i].values[3]
    p3 = data[i].values[7]
    
    m = ""
    lab = ""
    
    #assign colors to each state
    if data[i].tag2 == 'S':
        m = "g"
        if labelguide[0]:
            lab = "Sprocket"
            labelguide[0] = 0
    elif data[i].tag2 == 'B':
        m = "b"
        if labelguide[1]:
            lab = "Bullet"
            labelguide[1] = 0
    elif data[i].tag2 == 'W':
        m = "r"
        if labelguide[2]:
            lab = "Wandering Flock"
            labelguide[2] = 0
    elif data[i].tag1 == 'R':
        m = "m"
        if labelguide[3]:
            lab = "Rotation"
            labelguide[3] = 0
    elif data[i].tag2 == 'G':
        m = "k"
        if labelguide[4]:
            lab = "Gas"
            labelguide[4] = 0
    elif data[i].tag2 == 'C':
        m = "y"
        if labelguide[5]:
            lab = "Circulation"
            labelguide[5] = 0
    
    if lab != "":
        ax.scatter(p1.value, p2.value, p3.value, color=m, marker = 'o',label = lab)
    else:
        ax.scatter(p1.value, p2.value, p3.value, color=m, marker = 'o')
ax.set_xlabel(p1.name,labelpad = 20, fontsize = 20)
ax.set_ylabel(p2.name,labelpad = 20, fontsize = 20)
ax.set_zlabel(p3.name,labelpad = 20, fontsize = 20)
ax.legend(loc = 2,frameon = False)
ax.view_init(elev=77., azim=-62)
plt.rc('ytick',labelsize=10)
plt.locator_params(axis='z', nbins=4)

print("LENGTH OF DATA: ",len(data))
data[0].show()
    
        
        