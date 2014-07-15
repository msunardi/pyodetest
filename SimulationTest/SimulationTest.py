#import common_utils
import pygame
from pygame.locals import *
import ode

from random import randint

def coord(x, y):
    """Convert world coordinates to pixel coordinates"""
    return int(320+170*x), int(400-170*y)

world = ode.World()
world.setGravity((0, -9.81, 0))

fps = 50
dt = 1.0/fps
loopFlag = True
clk = pygame.time.Clock()

someblue = (55,0,200)

def createObjects(n):
    
    foos = []
    foos_joints = []
    attached_to = ode.environment
    le_anchor = (0, 2, 0)
    positions = [(i+1,2,0) for i in range(n)]

    for i in range(n):
        body = ode.Body(world)
        M = ode.Mass()
        M.setSphere(2500, 0.05)
        body.setMass(M)
        body.setPosition(positions[i])
        foos.append(body)

        if n > 1:
            j = ode.BallJoint(world)
            j.attach(body, attached_to)
            j.setAnchor(le_anchor)
            foos_joints.append(j)
           

            #x,y,z = body.getPosition()
            le_anchor = (positions[i])
            attached_to = body

    return foos, foos_joints


def runSimulation(loopFlag, n=1):

    pygame.init()

    srf = pygame.display.set_mode((640,480))

    bodies, joints = createObjects(n)
    line_x = 0
    line_y = 2
    prev_x = 0
    prev_y = 2
    anchor = 0
    prev_body = ode.environment

    while loopFlag:
        events = pygame.event.get()
        for e in events:
            print("Event %s" % e)
            if e.type == QUIT or e.type == KEYDOWN:
                loopFlag = False
                break
    
        srf.fill((255,255,255))

        for i in range(len(bodies)):
            x, y, z = bodies[i].getPosition()
            jx, jy, jz = joints[i].getAnchor()
            pygame.draw.circle(srf, someblue, coord(x,y), 20, 0)
            pygame.draw.line(srf, (55,0,200), coord(jx,jy), coord(x,y), 2)
            
            
        pygame.display.flip()

        world.step(dt)

        print("Tick")
        clk.tick(fps)