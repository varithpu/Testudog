#!/usr/bin/env python

# Originally written by Steve LaValle, UIUC for simple RRT in
# May 2011
# Modified by Cheng Liu
# May 2022

import rospy
import geometry_msgs.msg
import gazebo_msgs.msg
import sys, random, math, pygame
from bleach import Cleaner
from pygame.locals import *
from math import sqrt,cos,sin,atan2

#constants
XDIM = 640  #64m
YDIM = 480  #48m
WINSIZE = [XDIM, YDIM]
EPSILON = 7.0
NUMNODES = 5000
RADIUS=15
CLEARANCE=10
OBS=[(90,0,20,300),(240,200,20,280),(490,0,20,200),(490,300,20,180),(300,225,100,50)]
OBS2=[(150,100),(200,150),(150,200)]
# global variable
count = 0

def obsDraw(pygame,screen):
    blue=(0,0,255)
    for o in OBS: 
      pygame.draw.rect(screen,blue,o)
    for o2 in OBS2:
      pygame.draw.circle(screen,blue,o2,10)

def dist(p1,p2):
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def step_from_to(p1,p2):
    if dist(p1,p2) < EPSILON:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + EPSILON*cos(theta), p1[1] + EPSILON*sin(theta)

def chooseParent(nn,newnode,nodes):
    for p in nodes:
        if dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and p.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y]):
            nn = p
    newnode.cost=nn.cost+dist([nn.x,nn.y],[newnode.x,newnode.y])
    newnode.parent=nn
    return newnode,nn

def reWire(nodes,newnode,pygame,screen):
    white = 255, 240, 200
    black = 20, 20, 40
    for i in range(len(nodes)):
        p = nodes[i]
        if p!=newnode.parent and dist([p.x,p.y],[newnode.x,newnode.y]) <RADIUS and newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) < p.cost:
            pygame.draw.line(screen,white,[p.x,p.y],[p.parent.x,p.parent.y])  
            p.parent = newnode
            p.cost=newnode.cost+dist([p.x,p.y],[newnode.x,newnode.y]) 
            nodes[i]=p  
            pygame.draw.line(screen,black,[p.x,p.y],[newnode.x,newnode.y])                    
    return nodes

def drawSolutionPath(start,goal,nodes,pygame,screen):
    pink = 200, 20, 240
    nn = nodes[0]
    path = []
    for p in nodes:
        if dist([p.x,p.y],[goal.x,goal.y]) < dist([nn.x,nn.y],[goal.x,goal.y]):
            nn = p
    while nn!=start:
        pygame.draw.line(screen,pink,[nn.x,nn.y],[nn.parent.x,nn.parent.y],5)  
        path.append([nn.x,nn.y])
        nn=nn.parent
    path.append([nodes[0].x,nodes[0].y])   
    path.reverse()
    return path 

def ccw(A,B,C):
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def checkIntersect(nodeA,nodeB,OBS,OBS2):
    A=(nodeA.x,nodeA.y)
    B=(nodeB.x,nodeB.y)
    for o in OBS:
      obs=(o[0]-CLEARANCE,o[1]-CLEARANCE,o[0]+o[2]+CLEARANCE,o[1]+o[3]+CLEARANCE)
      C1=(obs[0],obs[1])
      D1=(obs[0],obs[3])
      C2=(obs[0],obs[1])
      D2=(obs[2],obs[1])
      C3=(obs[2],obs[3])
      D3=(obs[2],obs[1])
      C4=(obs[2],obs[3])
      D4=(obs[0],obs[3])
      inst1= ccw(A,C1,D1) != ccw(B,C1,D1) and ccw(A,B,C1) != ccw(A,B,D1) 
      inst2= ccw(A,C2,D2) != ccw(B,C2,D2) and ccw(A,B,C2) != ccw(A,B,D2)
      inst3= ccw(A,C3,D3) != ccw(B,C3,D3) and ccw(A,B,C3) != ccw(A,B,D3)
      inst4= ccw(A,C4,D4) != ccw(B,C4,D4) and ccw(A,B,C4) != ccw(A,B,D4)
      if inst1==False and inst2==False and inst3==False and inst4==False:
        #print(A,B)
        #input("Press Enter to continue...")
        continue      
      else:
        return False
    for o2 in OBS2:
      a=B[1]-A[1]
      b=A[0]-B[0]
      c=a*(A[0])+b*(A[1])
      dist=((abs(a*o2[0]+b*o2[1]-c))/math.sqrt(a*a+b*b))
      if (10+CLEARANCE) < dist:
        continue
      else:
        return False
    return True

class Node:
    x = 0
    y = 0
    cost=0  
    parent=None
    def __init__(self,xcoord, ycoord):
         self.x = xcoord
         self.y = ycoord
	
def main():
    #initialize and prepare screen
    pygame.init()
    screen = pygame.display.set_mode(WINSIZE)
    pygame.display.set_caption('RRTstar')
    white = 255, 240, 200
    black = 20, 20, 40
    screen.fill(white)
    obsDraw(pygame,screen)
    nodes = []
    
    #nodes.append(Node(XDIM/2.0,YDIM/2.0)) # Start in the center
    nodes.append(Node(30.0,30.0)) # Start in the corner
    start=nodes[0]
    goal=Node(600.0,450.0)
    for i in range(NUMNODES):
        rand = Node(random.random()*XDIM, random.random()*YDIM)
        nn = nodes[0]
        for p in nodes:
            if dist([p.x,p.y],[rand.x,rand.y]) < dist([nn.x,nn.y],[rand.x,rand.y]):
                nn = p
        interpolatedNode= step_from_to([nn.x,nn.y],[rand.x,rand.y])
	
        newnode = Node(interpolatedNode[0],interpolatedNode[1])
        #check obstacle:
        if checkIntersect(nn,rand,OBS,OBS2):
          [newnode,nn]=chooseParent(nn,newnode,nodes)
       
          nodes.append(newnode)
          pygame.draw.line(screen,black,[nn.x,nn.y],[newnode.x,newnode.y])
          nodes=reWire(nodes,newnode,pygame,screen)
          pygame.display.update()
          #print i, "    ", nodes
          for e in pygame.event.get():
              if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                sys.exit("Leaving because you requested it.") 
    path = drawSolutionPath(start,goal,nodes,pygame,screen)
    screen.blit(pygame.transform.flip(screen, False, True),(0,0))
    pygame.display.update()
    return path

def get_index(list, str):
    for i in range(len(list)):
        if list[i] == str:
            return i

def callback(data):
    global count
    idx = get_index(data.name, "robot_model")
    pos_x = data.pose[idx].position.x
    pos_y = data.pose[idx].position.y
    dist =  math.sqrt((pos_x-paths[count][0])**2 + (pos_y-paths[count][1])**2)
    while dist < 3:
        count += 1
        dist =  math.sqrt((pos_x-paths[count][0])**2 + (pos_y-paths[count][1])**2)
    msg.x = paths[count][0]
    msg.y = paths[count][1]
    print(paths[count])
    pub.publish(msg)
        
if __name__ == '__main__':
    rospy.init_node('trajectory_publisher')
    pub = rospy.Publisher('trajectory', geometry_msgs.msg.Point, queue_size=10)
    msg = geometry_msgs.msg.Point()
    paths = main()
    paths = [(round(x/10,2),round(y/10,2)) for (x,y) in paths]
    for path in paths:
        print(path)
    sub =  rospy.Subscriber("gazebo/model_states", gazebo_msgs.msg.ModelStates, callback)
    rospy.spin()
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False


