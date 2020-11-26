##Libraries
import numpy as np
import matplotlib.pyplot as plt
import csv
#inputs
obstacles = np.genfromtxt('obstacles.csv',delimiter = ',')
#euclidian distance
def d(a,b):
    return ((a[0]-b[0])**2+(a[1]-b[1])**2)**0.5

#generate random points betwen -0.5 0.5
def create_points(n,r,obstacles):
    #initial point
    points = np.array([-0.5,-0.5]).reshape(1,2)

    
    x = np.random.random_sample()-0.5
    for _ in range(n-2):
        flag = True
        y = np.random.random_sample()-0.5
        x = np.random.random_sample()-0.5
        p = np.array([x,y])
            
        for obs in obstacles:
            #check if the point is inside the obstacle
            if d(p,obs[:2]) < (obs[2]/2)+r:
                flag = False
                break
        if flag:
            points = np.append(points,p.reshape(1,2),axis = 0)
    #final point
    points = np.append(points,np.array([0.5,0.5]).reshape(1,2),axis = 0)
    return points

rr = create_points(50,0.,obstacles)
## Code just to check the condition of the points

#ax = plt.gca()
#ax.set(xlim=(-0.51,0.51), ylim=(-0.51,0.51))
#plt.plot(rr[:,0],rr[:,1],'ro')
#len(rr)

#Function to check intersection between edeges and obstacles
def cut(p1,p2,circle):
    if p1[0]-p2[0] == 0:
        xl = p1[0]
        yl = circle[1]
    else: 
        a = (p1[1]-p2[1])/(p1[0]-p2[0])
        b = -1 
        c = p1[1]-a*p1[0]
        xc = circle[0]
        yc = circle[1]
        xl = (xc*b*b-a*(yc*b+c))/(b*b+a*a)
        yl = a*xl+c
    pl = [xl,yl]
    d1 = d(p1,pl)
    d2 = d(p2,pl)
    dx = d(p1,p2)
    if (max(d1,d2) > dx) or (d(pl,circle[:2])>((circle[2]/2)+0.01)):
        return False
    else:
        return True

##Function to generate nodes.csv and edges.csv

def gen(points, obstacles):
    nodes = []
    for i,j in enumerate(points):
        nodes.append([i+1,j[0],j[1],d(j,np.array([0.5,0.5]))])
    nodes = np.array(nodes)
    aux = np.flip(np.sort(nodes[:,3]))
    print(aux)
    rnodes = []
    k = 1
    for j in aux: 
        for i in nodes:
            if i[3] == j :
                rnodes.append([k,i[1],i[2],j])
        k += 1

    edges = []
    for i in range(len(rnodes)-1):
        a1 = [rnodes[i][1],rnodes[i][2]]
        for j in range(i+1,len(rnodes)):
            a2 = [rnodes[j][1],rnodes[j][2]]
            flag = True
            for circle in obstacles: 
                if cut(a1,a2,circle):
                    flag = False
                    break
            if flag:
                edges.append([j+1,i+1,d(a1,a2)])
    
    with open("nodes.csv", "w", newline="") as myfile:
        wr = csv.writer(myfile)
        wr.writerows(rnodes)

    with open('edges.csv', 'w',newline="") as myfile:
        wr = csv.writer(myfile)
        wr.writerows(edges)
    
    return rnodes, edges

rnodes, edges = gen(rr,obstacles)

#A* search writed in the previos assignment
def sp(el,lista):
    vacia = []
    for i in lista:
        if i[1]==el:
            vacia.append((i[0],i[2]))
    return vacia

def Astar(edges,nodes):

    #Initialization of the table in dictionaries
    pastcost ={i[0]:np.Inf for i in nodes}
    pastcost[1] = 0
    optimist = {i[0]:i[3] for i in nodes}
    #optimist = {i[0]:0 for i in nodes}
    tt = {i:optimist[i]+pastcost[i] for i in optimist}
    parents = {i[0]:0 for i in nodes}
    
    ## Arrays of nodes open and closed 
    opens = []
    closeds = []
    opens.append((tt[nodes[0,0]],nodes[0,0]))
    answ = []
    while len(closeds) < len(nodes) and opens:
         
        a_node = opens[0][1]
        aux = sp(a_node,edges)
        for i in aux:
            
            if not (i[0] in closeds):
                vv = i[1]+pastcost[a_node]
                if vv < pastcost[i[0]]:
                    pastcost[i[0]] = vv #update of pastcoast
                    parents[i[0]] = a_node
                tt[i[0]] = optimist[i[0]] + pastcost[i[0]] # update of total coast
                opens.append((tt[i[0]],i[0]))
        
        ##code to remove a node from open and added to closed
        for i in opens:
            if i[1] == a_node:
                opens.remove((i[0],i[1]))
        opens = sorted(opens)
        closeds.append(a_node)
    if not parents[len(nodes)] == 0:
        xx = len(nodes)
    else:
        xx = closeds[-1]

    #Create a list using the parents
    while not(parents[xx] == 0):
        answ.append(xx)
        xx = parents[xx]
    answ.append(1)
    answ.reverse()
    answ = list(map(int, answ))
    
    #Saving as csv
    with open('path.csv', 'w', newline='') as myfile:
        wr = csv.writer(myfile)
        wr.writerow(answ)
    print(parents)
    return answ

out = Astar(np.array(edges),np.array(rnodes))
