#!/usr/bin/python

##-------------------------------------------------------------------
##http://forums.trossenrobotics.com/showthread.php?6164-Kinematic-simulator-in-python
## Module: grinder_simulator.py
##
## Grinder Hexapod Robot Simulator
## 
## Uses Visual Python to generate simplified realtime render
## of robot leg linkages given foot x,y,z positions using 
## Inverse Kinematics equations 
##
## Simulates the kinematics of the hexpaod robot Grinder
## http:///www.gotrobots.com/grinder/
##
## Written by Nick Donaldson nick@gotrobots.com
##
## Release Information:
##  V0.0.15 Added IK for wheel on ground case
##  V0.0.01 05/03/2013: Created
##
##-------------------------------------------------------------------
##
## NOTES
##  1.  Requires visual module from www.vpython.org
##  2.  Simulates linkage mechanisms of 18 DOF robot 
##  3.  This software is a work in progress, check back for updates
##
##-------------------------------------------------------------------
##
## Copyright (c) 2013, Nick Donaldson
## All rights reserved.
## 
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are met:
##     * Redistributions of source code must retain the above copyright
##       notice, this list of conditions and the following disclaimer.
##     * Redistributions in binary form must reproduce the above copyright
##       notice, this list of conditions and the following disclaimer in the
##       documentation and/or other materials provided with the distribution.
##
##-------------------------------------------------------------------


#from __future__ import division, print_function
from visual import *
import sys

debug = True
#debug = False

class Leg():
    ''' Class that implements the robot leg model
    '''
    def __init__(self, name, x, y, z, side, x_config):

        # static lengths pulled from CAD
        self.AB = 35.0
        self.BC = 40.0
        self.CD = 35.0
        self.DE = 80.0
        self.EF = 305.
        self.EK = 100.0
        self.FK = 209.358
        self.IJ = 35.0
        self.JK = 114.0
        self.IK = 95.26
        self.KL = 184.730
        self.EN = 274.247
        self.FN = 57.79
        self.LF = 24.627
        self.LM = 44.534
        self.MN = 23.0
        self.NW = 6.5 # wheel ring radius
        #self.FWav = 10.52 # average vertical distance from foot to wheel contact point (when wheel is at median)

        # static angles
        self.CDE = radians(105)
        self.KLM = radians(57.55)

        # reference skating geometry
        #self.skating_in_Wx = 213.972
        #self.skating_in_Wy = -130
        #self.skating_out_Wx = 296.366
        #self.skating_out_Wy = -130
        #self.skating_circle_center = self.skating_in_Wx + ( (self.skating_out_Wx - self.skating_in_Wx)/2)
        #self.skating_circle_radius = self.skating_circle_center - self.skating_in_Wx

        self.skating_circle_center = 255
        self.skating_circle_radius = 41

        #self.skating_out_Y_angle = radians(103.44)
        #self.skating_Z_out_angle = radians(24.98)
        #self.skating_out_Fx = 353.63
        #self.skating_out_Fy = 115.70
        #self.skating_in_Y_angle = radians(30.95)
        #self.skating_Z_in_angle = radians(15.0)
        #self.skating_in_Fx = 271.76
        #self.skating_in_Fy = 123.28
        #self.skating_in_wheel_angle = radians(-1.98) # median 1.79
        #self.skating_out_wheel_angle = radians(5.56) # 5.55903982deg
        if debug == True: print "self.skating_circle_center =", self.skating_circle_center, "skating_circle_radius = ", self.skating_circle_radius

        # initial stance
        self.init_x = 0
        self.init_y = 250
        self.init_z = -180

        # IK coordinates
        self.Xx = 0.0
        self.Xy = 0.0
        self.Ax = 23.0
        self.Ay = 0.0
        self.Bx = 0.0
        self.By = 0.0
        self.Cx = 0.0
        self.Cy = 0.0
        self.Dx = 88.0
        self.Dy = 40.0
        self.Ex = 0.0
        self.Ey = 0.0
        self.Fx = 0.0
        self.Fy = 0.0
        self.Ix = 88.0
        self.Iy = -10.0
        self.Jx = 0.0
        self.Jy = 0.0
        self.Kx = 0.0
        self.Ky = 0.0
        self.Lx = 0.0
        self.Ly = 0.0
        self.Mx = 0.0
        self.My = 0.0
        self.Nx = 0.0
        self.Ny = 0.0
        self.Wx = 0.0
        self.Wy = 0.0

        # squared values
        self.AB2 = self.AB * self.AB
        self.BC2 = self.BC * self.BC
        self.CD2 = self.CD * self.CD
        self.DE2 = self.DE * self.DE
        self.EF2 = self.EF * self.EF
        self.EK2 = self.EK * self.EK
        self.FK2 = self.FK * self.FK
        self.IJ2 = self.IJ * self.IJ
        self.JK2 = self.JK * self.JK
        self.IK2 = self.IK * self.IK
        self.EN2 = self.EN * self.EN
        self.FN2 = self.FN * self.FN

        # Static value calculations, only calc once
        # FEK is angle between line from top leg axis to foot and line from top leg axis to bottom leg axis
        self.FEK = acos((self.EF2 + self.EK2 - self.FK2)/(2 * self.EF * self.EK))
        # if debug == True: print "FEK =", degrees(self.FEK)

        # ADV is the angle between AD and vertical
        self.ADV = atan(abs(self.Dx-self.Ax)/abs(self.Dy-self.Ay))
        # if debug == True: print "ADV =", degrees(self.ADV)

        self.AD = ((self.Dx-self.Ax)*(self.Dx-self.Ax) + (self.Dy-self.Ay)*(self.Dy-self.Ay))**0.5
        # if debug == True: print "AD =", self.AD

        # leg position config
        self.side = side
        self.x_config = x_config

        # element colors 
        horn_color = (0.5,0,0.0)
        linkage_color = (0.4,0,1.0)
        vertex_color = (0.4,0.4,0.4)
        fixed_color = (0.8,0.8,0.8)
        axle_color = (0.0,0.5,0.5)
        #wheel_color = (0.0,0.4,0.0)
        wheel_color = (0.0,0.6,0.0)

        # leg frame, containing Z and Y mechanism
        self.frame = frame(pos=vector(x,y,z), axis=vector(x+1,0,0))

        # now create graphical elements (initially laid out in a line)

        # display origin as a sphere for reference
        #origin = sphere(frame=self.frame, pos=(0,0,0), radius=3, color=axle_color)
        #axis_X = cylinder(frame=self.frame, pos=(0,-15,0), axis=(0,30,0), radius=1.5, color=axle_color)

        # first linkage, servo horn AB
        (self.link_AB, self.start_AB, self.end_AB) = self.create_linkage(self.Ax, self.Ay, 0, self.Bx, self.By, 0, self.frame, horn_color, vertex_color)
        self.start_AB.color = fixed_color

        # second linkage BC
        (self.link_BC, self.start_BC, self.end_BC) = self.create_linkage(self.Bx, self.By, 0, self.Cx, self.Cy, 0, self.frame, linkage_color, vertex_color)

        # third linkage CDE
        (self.link_CD, self.start_CD, self.end_CD) = self.create_linkage(self.Cx, self.Cy, 0, self.Dx, self.Dy, 0, self.frame, linkage_color, vertex_color)
        (self.link_DE, self.start_DE, self.end_DE) = self.create_linkage(self.Dx, self.Dy, 0, self.Ex, self.Ey, 0, self.frame, linkage_color, vertex_color)
        self.end_CD.color = fixed_color
        #self.end_CD.radius = 4
        self.start_DE.color = fixed_color
        #self.start_DE.radius = 4
        #(self.link_CE, self.start_CE, self.end_CE) = self.create_linkage(self.Cx, self.Cy, 0, self.Ex, self.Ey, 0, self.frame, linkage_color, vertex_color)

        # fourth linkage EKF
        (self.link_EK, self.start_EK, self.end_EK) = self.create_linkage(self.Ex, self.Ey, 0, self.Kx, self.Ky, 0, self.frame, linkage_color, vertex_color)
        (self.link_KF, self.start_KF, self.end_KF) = self.create_linkage(self.Kx, self.Ky, 0, self.Fx, self.Fy, 0, self.frame, linkage_color, vertex_color, False)

        # fifth linkage IJ
        (self.link_IJ, self.start_IJ, self.end_IJ) = self.create_linkage(self.Ix, self.Iy, 0, self.Jx, self.Jy, 0, self.frame, horn_color, vertex_color)
        self.start_IJ.color = fixed_color

        # sixth linkage J
        (self.link_JK, self.start_JK, self.end_JK) = self.create_linkage(self.Jx, self.Jy, 0, self.Kx, self.Ky, 0, self.frame, linkage_color, vertex_color)

        # wheel support LM
        (self.wheel_axle, self.start_wheel, self.end_wheel) = self.create_linkage(self.Lx, self.Ly, 0, self.Mx, self.My, 0, self.frame, linkage_color, vertex_color)

        # wheel
        self.wheel = ring(frame=self.frame, pos=(self.Mx,self.My,0), axis=(self.Mx-self.Lx,self.My-self.Ly,0), radius=self.MN-self.NW, thickness=2*self.NW, color=wheel_color)
        #self.wheel = ring(frame=self.frame, pos=(self.Mx,self.My,0), axis=(1,0,0), radius=self.MN+self.NW, thickness=20, color=wheel_color)
        #self.wheel_sphere = sphere(frame=self.frame, pos=(self.Mx,self.My,0), radius=28.5, color=color.white)

    def create_linkage(self, X0, Y0, Z0, X1, Y1, Z1, frame_name, linkage_color, vertex_color, create_end=True):
        #sphere(frame=frame_name,pos=vector(0,-130,0), radius=5, color=color.red)
        linkage = cylinder(frame=frame_name,pos=vector(X0,Y0,Z0), axis=vector(X1-X0,Y1-Y0,Z1-Z0), radius=4, color=linkage_color)
        start = sphere(frame=frame_name,pos=vector(X0,Y0,Z0), radius=6, color=vertex_color)
        if create_end == True: 
            end = sphere(frame=frame_name,pos=vector(X1,Y1,Z1), radius=6, color=vertex_color)
        else:
            end = sphere(frame=frame_name,pos=vector(X1,Y1,Z1), radius=1, color=linkage_color)
        return (linkage, start, end)

    def run_IK(self, x, y, z, mode):
        # Calculate servo angles needed to place foot at x,y,z in leg local frame of reference
        # whose origin is on X axis at center of body, x along from-back axis, y along left-right 
        # axis, z along top-bottom axis. X servo angle is calculated from x,y via trig. From
        # x,y,z input we calculate ZY local frame x,y as viewed if leg was flat on a table  

        if debug == True: print
        if debug == True: print "x, y, z = %s, %s, %s" % (x,y,z)

        # first, calculate X servo angle using x and y, oh so easy...
        self.X_angle = atan(x/y)
        if debug == True: print "X_angle = %s" % (degrees(self.X_angle))

        # ZY local frame x is hypoteneuse of xy triangle - cos tested faster than sin or pyth
        local_x = abs(y / cos(self.X_angle) )

        # ZY local frame y is simply z...
        local_y = z

        if debug == True: print "local_x, local_y = %s, %s" % (local_x,local_y)

        # if we are skating, need to back calculate foot position from wheel first
        if mode == 'skate':
            if debug == True: print ">>>> skating mode IK"
            
            self.Nx = local_x
            self.Ny = local_y + self.NW
            if debug == True: print "Nx =", self.Nx
            if debug == True: print "Ny =", self.Ny

            DN = ((self.Nx-self.Dx)*(self.Nx-self.Dx) + (self.Ny-self.Dy)*(self.Ny-self.Dy))**0.5
            if debug == True: print "DN =", DN

            EDN = acos((self.DE2 + DN*DN - self.EN2)/(2 * self.DE * DN))
            if debug == True: print "EDN =", degrees(EDN)

            NDV = atan(abs(self.Nx-self.Dx)/abs(self.Ny-self.Dy))
            if debug == True: print "NDV =", degrees(NDV)

            EDV = pi - NDV - EDN
            self.Ex = self.Dx + self.DE * sin(EDV)
            self.Ey = self.Dy + self.DE * cos(EDV)
            if debug == True: print "Nx =", self.Nx
            if debug == True: print "Ny =", self.Ny
            if debug == True: print "Ex =", self.Ex
            if debug == True: print "Ey =", self.Ey

            NEV = atan(abs(self.Nx-self.Ex)/abs(self.Ey-self.Ny))
            if debug == True: print "NEV =", degrees(NEV)

            FEN = acos((self.EF2 + self.EN2 - self.FN2)/(2 * self.EF * self.EN))
            if debug == True: print "FEN =", degrees(FEN)

            FEV = NEV + FEN
            self.Fx = self.Ex + self.EF * sin(FEV)
            self.Fy = self.Ey - self.EF * cos(FEV)
            if debug == True: print "Fx =", self.Fx
            if debug == True: print "Fy =", self.Fy
        else:
            if debug == True: print ">>>> walking mode IK"

            # store foot position
            self.Fx = local_x
            self.Fy = local_y

        if debug == True: print ">>>> rest of the IK..."

        # Z servo

        # DF = dist from fixed axle D to foot
        DF = ((self.Fx-self.Dx)*(self.Fx-self.Dx) + (self.Fy-self.Dy)*(self.Fy-self.Dy))**0.5
        if debug == True: print "DF =", DF


        # EDF is between the long arm of L shaped linkage to line connecting fixed axle D of L linkage to foot
        #if debug == True: print "EDF = acos((%s + %s - %s)/(2 * %s * %s))" % (DF*DF, self.DE2, self.EF2, self.DE, DF)
        #if debug == True: print 'EDF = acos %s + %s - %s 2 * %s * %s ' % (DF*DF, self.DE2, self.EF2, self.DE, DF)
        EDF = acos((DF*DF + self.DE2 - self.EF2)/(2 * self.DE * DF))
        if debug == True: print "EDF =", degrees(EDF)

        # FDV is the angle between line from L linkage fixed axis to foot and vertical
        FDV = atan(abs(self.Fx-self.Dx)/abs(self.Fy-self.Dy))
        if debug == True: print "FDV =", degrees(FDV)

        # CDV is the angle between vertical and short arm of L linkage
        CDV = 2 * pi - EDF - FDV - self.CDE
        if debug == True: print "CDV =", degrees(CDV)

        BDC = CDV - self.ADV
        if debug == True: print "BDC =", degrees(BDC)

        self.Cx = self.Dx - (self.CD * sin(pi - CDV))
        self.Cy = self.Dy + (self.CD * cos(pi - CDV))
        if debug == True: print "Cx =", self.Cx
        if debug == True: print "Cy =", self.Cy

        AC = ((self.Cx-self.Ax)*(self.Cx-self.Ax) + (self.Cy-self.Ay)*(self.Cy-self.Ay))**0.5
        if debug == True: print "AC =", AC

        # BAC is angle between servo horn AB and line from A to C
        BAC = acos((self.AB2 + AC*AC - self.BC2)/(2 * self.AB * AC))
        if debug == True: print "BAC =", degrees(BAC)

        # CAH is the angle between servo horn AB and horizontal
        CAH = atan(abs(self.Cy-self.Ay)/abs(self.Cx-self.Ax))
        if debug == True: print "CAH =", degrees(CAH)

        # .: servo horn angle is CAH - BAC
        self.Z_angle = CAH - BAC
        if debug == True: print "z_angle =", degrees(self.Z_angle)
         
        self.Bx = self.Ax + self.AB*cos(self.Z_angle)
        self.By = self.Ay + self.AB*sin(self.Z_angle)
        if debug == True: print "Bx =", self.Bx
        if debug == True: print "By =", self.By

        ### # Y Servo

        self.Ex = self.Dx + (self.DE * sin(pi - FDV - EDF))
        if debug == True: print "Ex =", self.Ex

        self.Ey = self.Dy + (self.DE * cos(pi - FDV - EDF))
        if debug == True: print "Ey =", self.Ey

        FEV = atan(abs(self.Fx-self.Ex)/abs(self.Ey-self.Fy))
        if debug == True: print "FEV =", degrees(FEV)

        self.Kx = self.Ex + (self.EK * sin(FEV + self.FEK))
        self.Ky = self.Ey - (self.EK * cos(FEV + self.FEK))
        if debug == True: print "Kx =", self.Kx
        if debug == True: print "Ky =", self.Ky

        IK = ((self.Kx-self.Ix)*(self.Kx-self.Ix) + (self.Ky-self.Iy)*(self.Ky-self.Iy))**0.5
        if debug == True: print "IK =", IK

        # JIK is angle between Y servo horn IJ and line from I to K
        if (self.IJ2 + IK*IK - self.JK2) < (2 * self.IJ * IK):
            JIK = acos((self.IJ2 + IK*IK - self.JK2)/(2 * self.IJ * IK))
        else:
            JIK = 0
        if debug == True: print "JIK =", degrees(JIK)

        # IKH is the angle between line between I and K and horizontal
        KIV = atan(abs(self.Kx-self.Ix)/abs(self.Ky-self.Iy))
        if debug == True: print "KIV =", degrees(KIV)

        # .: servo horn angle is 180 - IJK - KIV
        self.Y_angle = pi - JIK - KIV
        if debug == True: print "y_angle =", degrees(self.Y_angle)

        self.Jx = self.Ix + self.IJ*sin(self.Y_angle)
        self.Jy = self.Iy - self.IJ*cos(self.Y_angle)
        if debug == True: print "Jx =", self.Jx
        if debug == True: print "Jy =", self.Jy

        # L is point where wheel axle intersects KF
        self.Lx = self.Kx + (self.KL/(self.KL+self.LF))*(self.Fx-self.Kx)
        self.Ly = self.Ky + (self.KL/(self.KL+self.LF))*(self.Fy-self.Ky)
        if debug == True: print "Lx =", self.Lx
        if debug == True: print "Ly =", self.Ly

        self.KLH = atan((self.Ky-self.Ly)/(self.Lx-self.Kx))
        if debug == True: print "self.KLH = ", degrees(self.KLH)

        # M is center of wheel
        self.MLH = self.KLH - self.KLM
        if debug == True: print "self.MLH = ", degrees(self.MLH)
        self.Mx = self.Lx - (self.LM * cos(self.MLH))
        if debug == True: print "Mx =", self.Mx
        self.My = self.Ly + (self.LM * sin(self.MLH))
        if debug == True: print "My =", self.My

        #if self.Fy == 179.3:
        #    exit()
        #return [self.Z_angle, self.Y_angle]

    def calc_wheel_offset(self):
        # L is point where wheel axle intersects KF
        self.Lx = self.Kx + (self.KL/(self.KL+self.LF))*(self.Fx-self.Kx)
        self.Ly = self.Ky + (self.KL/(self.KL+self.LF))*(self.Fy-self.Ky)
        if debug == True: print "Lx =", self.Lx
        if debug == True: print "Ly =", self.Ly

        self.KLH = atan((self.Ky-self.Ly)/(self.Lx-self.Kx))
        if debug == True: print "self.KLH = ", degrees(self.KLH)

        # M is center of wheel
        self.MLH = self.KLH - self.KLM
        if debug == True: print "self.MLH = ", degrees(self.MLH)
        self.Mx = self.Lx - (self.LM * cos(self.MLH))
        if debug == True: print "Mx =", self.Mx
        self.My = self.Ly + (self.LM * sin(self.MLH))
        if debug == True: print "My =", self.My

        # N is center of wheel ring model cross section
        self.Nx = self.Mx - self.MN * sin(self.MLH)
        self.Ny = self.My - self.MN * cos(self.MLH)
        if debug == True: print "Nx =", self.Nx
        if debug == True: print "Ny =", self.Ny

        # W is contact point, vertically below Nx,Ny
        self.Wx = self.Nx
        self.Wy = self.Ny - self.NW
        if debug == True: print "Wx =", self.Wx
        if debug == True: print "Wy =", self.Wy
        
                
    def get_foot_from_wheel(self):
        self.Nx = self.Wx
        self.Ny = self.Wy + self.NW
        if debug == True: print "Nx =", self.Nx
        if debug == True: print "Ny =", self.Ny

        DN = ((self.Nx-self.Dx)*(self.Nx-self.Dx) + (self.Ny-self.Dy)*(self.Ny-self.Dy))**0.5
        if debug == True: print "DN =", DN

        EDN = acos((self.DE2 + DN*DN - self.EN2)/(2 * self.DE * DN))
        if debug == True: print "EDN =", degrees(EDN)

        NDV = atan(abs(self.Nx-self.Dx)/abs(self.Ny-self.Dy))
        if debug == True: print "NDV =", degrees(NDV)

        EDV = pi - NDV - EDN
        Ex = self.Dx + self.DE * sin(EDV)
        Ey = self.Dy + self.DE * cos(EDV)
        if debug == True: print "Ex =", Ex
        if debug == True: print "Ey =", Ey

        NEV = atan(abs(self.Nx-self.Ex)/abs(self.Ny-self.Ey))
        if debug == True: print "NEV =", degrees(NEV)

        FEN = acos((self.EF2 + self.EN2 - self.FN2)/(2 * self.EF * self.EN))
        if debug == True: print "FEN =", degrees(FEN)

        FEV = NEV + FEN
        self.Fx = self.Ex + self.EF * sin(FEV)
        self.Fy = self.Ey - self.EF * cos(FEV)
        if debug == True: print "Fx =", self.Fx
        if debug == True: print "Fy =", self.Fy

        return (self.Fx, self.Fy)

        foo='''
        # FWy = bottom of foot to bottom of the wheel in y 
        self.FWy = self.Fy - self.Wy
        if debug == True: print "FWy =", self.FWy

        self.offset = (self.FWav - self.FWy)
        if debug == True: print "offset =", self.offset

        z_adjusted = self.Fy - self.offset
        if debug == True: print "z_orig =", self.Fy
        if debug == True: print "z_adjusted =", z_adjusted
        return z_adjusted 

        self.offset = (self.FWav - self.FWy)
        if debug == True: print "offset =", self.offset

        z_adjusted = self.Fy - self.offset
        if debug == True: print "z_orig =", self.Fy
        if debug == True: print "z_adjusted =", z_adjusted
        return z_adjusted 



        # LFy = vertical distance between L and F
        LFy = self.LF * sin(self.KLH)
        if debug == True: print "LFy =", LFy

        MNy = self.MN * cos(self.MLH)
        if debug == True: print "MNy =", MNy

        LMy = self.LM * sin(self.MLH)
        if debug == True: print "LMy =", LMy

        # bottom of foot to bottom of the wheel in y 
        self.FWy = (MNy + self.NW) - (LMy + LFy)
        if debug == True: print "FWy =", self.FWy
        if debug == True: print "Wy =", self.Fy - self.FWy

        self.offset = (self.FWav - self.FWy)
        if debug == True: print "offset =", self.offset

        z_adjusted = self.Fy - self.offset
        if debug == True: print "z_orig =", self.Fy
        if debug == True: print "z_adjusted =", z_adjusted
        return z_adjusted 
'''

    def world_space_pos(frame, local):
        """Returns the position of local in world space."""
        x_axis = norm(frame.axis)
        z_axis = norm(cross(frame.axis, frame.up))
        y_axis = norm(cross(z_axis, x_axis))
        return frame.pos+local.x*x_axis+local.y*y_axis+local.z*z_axis

    def move_linkage(self, linkage, start, end, start_x, start_y, end_x, end_y):
        # move & rotate linkage and ends
        start.pos = (start_x, start_y, 0)
        end.pos = (end_x, end_y, 0)
        linkage.pos = (start_x, start_y, 0)
        linkage.axis = vector(end_x - start_x, end_y - start_y, 0)

    def update_leg(self):
        # move & rotate servo horn AB
        self.move_linkage(self.link_AB, self.start_AB, self.end_AB, self.Ax, self.Ay, self.Bx, self.By)

        # move & rotate BC
        self.move_linkage(self.link_BC, self.start_BC, self.end_BC, self.Bx, self.By, self.Cx, self.Cy)

        # move & rotate CD
        self.move_linkage(self.link_CD, self.start_CD, self.end_CD, self.Cx, self.Cy, self.Dx, self.Dy)

        # move & rotate DE
        self.move_linkage(self.link_DE, self.start_DE, self.end_DE, self.Dx, self.Dy, self.Ex, self.Ey)

        # move & rotate EK
        self.move_linkage(self.link_EK, self.start_EK, self.end_EK, self.Ex, self.Ey, self.Kx, self.Ky)

        # move & rotate KF
        self.move_linkage(self.link_KF, self.start_KF, self.end_KF, self.Kx, self.Ky, self.Fx, self.Fy)

        # move & rotate IJ
        self.move_linkage(self.link_IJ, self.start_IJ, self.end_IJ, self.Ix, self.Iy, self.Jx, self.Jy)

        # move & rotate JK
        self.move_linkage(self.link_JK, self.start_JK, self.end_JK, self.Jx, self.Jy, self.Kx, self.Ky)

        # move & rotate LM
        self.move_linkage(self.wheel_axle, self.start_wheel, self.end_wheel, self.Lx, self.Ly, self.Mx, self.My)

        # move & rotate wheel ring
        self.wheel.pos = (self.Mx, self.My, 0)
        self.wheel.axis = (self.Mx-self.Lx,self.My-self.Ly,0)
        #self.wheel_sphere.pos = (self.Mx, self.My, 0)
        #(self.link_JK, self.start_JK, self.end_JK, self.Jx, self.Jy, self.Kx, self.Ky)

    def rotate_leg(self, angle):
        # if debug == True: print angle
        if self.side == 'left':
            self.frame.axis = vector(-cos(angle), 0, sin(angle))
        else:
            self.frame.axis = vector(cos(angle), 0, sin(angle))
        # if debug == True: print self.frame.axis


#if __name__ == "__main__":
def main(debug_on=False):
    if debug_on:
        debug = True
    scene.title = 'Grinder Leg Linkage'
    scene.width = 1920
    scene.height = 1200

    ground_Z = -130.0
    #ground_Z = -127.0

    # base plates
    #top_plate = box(pos=(0,37,0), length=160, width=650, height=4, color=(0.3,0.0,0.6), axis=(1,0,0))    
    bottom_plate = box(pos=(0,-37,0), length=160, width=650, height=4, color=(0.3,0.0,0.6), axis=(1,0,0))    
    #ground = box(pos=(0,ground_Z,0), length=800, width=800, height=4, color=(0.6,0.3,0.0), axis=(1,0,0))    
    ground = box(pos=(0,ground_Z-1,0), length=800, width=800, height=2, color=(1,1,1), axis=(1,0,0))    


    leg_names = ['FL', 'FR', 'ML', 'MR', 'BL', 'BR']
    # leg origins in global space
    leg_origins = { 
        'FL': [310, -70, 0],
        'FR': [310, 70, 0],
        'ML': [0, -70, 0],
        'MR': [0, 70, 0],
        'BL': [-310, -70, 0],
        'BR': [-310, 70, 0]
        }

    # create all legs
    legs = {}
    for name in leg_names:
        if name[1] == 'L':
            side = 'left'
        else:
            side = 'right'
        if name[0] == 'B':
            x_config = 'forward'
        else:
            x_config = 'back'


        # create leg object
        legs[name] = Leg(name, leg_origins[name][1], 0, leg_origins[name][0], side, x_config)

        # run IK on initial position
        #legs[name].run_IK(0, 356.324, -103.314, 'walk')
        legs[name].run_IK(0, 296.366, -130, 'skate')

        # move legs to initial position
        legs[name].update_leg()

        #legs[name].update_leg(legs[name].init_x, legs[name].init_y, legs[name].init_z)

        #legs[name].update_leg(0, 274.624, -174.756)
        #legs[name].update_leg(0, 331.863, -116.476)
        #legs[name].update_leg(0, 324.062, -119.479)

        #z_adj = legs[name].calc_wheel_offset()
        #legs[name].update_leg(0, legs[name].Fx, z_adj, 'walk')
        #legs[name].calc_wheel_offset()

    #exit()

    scene.autocenter = True
    sleep(0.1)
    scene.autocenter = False

    #leg_FR = Leg('FR', -200, 100, 0, 'left', 'back')

    #exit()
    x_center = 0.
    #y_center = 314.
    #radius = 40.
    y_center = legs['FL'].skating_circle_center
    if debug == True: print"y_center =", y_center

    radius = 40.
    #radius = legs['FL'].skating_circle_radius
    z = ground_Z# + legs['FL'].FWav# + 6
    skating = True
    while True:
        for d in range(0,360):
            rate(200)

            # if debug == True: print "#################### X_angle = %s, y = %s" % (X_angle, y)
            #label(pos=(-30,-15,0), text='x,y = %s,%s, i = %s, Z_angle = %s' % (x,y, i, degrees(Z_angle)))

            #if debug == True: print "(x,y,z) = (%s,%s,%s)" % (x,y,z)
            for name,leg in legs.iteritems():

                # calculate circular coords
                if name[0] == 'M':
                    x = x_center + radius * sin(radians(d))
                    y = y_center + radius * cos(radians(d))
                else:
                    x = x_center + radius * sin(radians((d+180)%360))
                    y = y_center + radius * cos(radians((d+180)%360))



                if debug == True: print
                if debug == True: print"name: ",name
                if debug == True: print
                if debug == True: print"z =", z

                if debug == True: print"y_center =", y_center
                if debug == True: print"leg.skating_circle_radius =", leg.skating_circle_radius

                # update IK 
                if skating == True:
                    if debug == True: print "### Skating!"
                    leg.run_IK(x, y, z, 'skate')
                else:
                    if debug == True: print "walking"
                    leg.run_IK(x, y, z, 'walk')

                # move leg to new position
                leg.update_leg()

                # rotate leg about X
                leg.rotate_leg(leg.X_angle)
            #exit()

