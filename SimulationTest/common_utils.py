import sys, os, random, time
from math import *
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import ode
import numpy as np

# rotation directions are named by the third (z-axis) row of the 3x3 matrix,
#   because ODE capsules are oriented along the z-axis
rightRot = (0.0, 0.0, -1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0)
leftRot = (0.0, 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0)
upRot = (1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0)
downRot = (1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 1.0, 0.0)
bkwdRot = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)

def sign(x):
    """ Returns 1.0 if x positive, -1.0 otherwise"""
    return 1.0 if x > 0.0 else -1.0

def len3(vector):
    """Returns the length of 3-vector vector"""
    #return sqrt(pow(vector[0], 2) + pow(vector[1], 2) + pow(vector[2], 2))
    return np.linalg.norm(vector)

def neg3(vector):
    """Returns the negation of 3-vector vector"""
    return np.negative(vector)

def add3(vector1, vector2):
    return np.add(vector1, vector2)

def sub3(vector1, vector2):
    return np.subtract(vector1, vector2)

def mul3(vector, scalar):
    return np.multiply(vector,scalar)

def div3(vector, scalar):
    return np.divide(vector, float(scalar))

def dist3(vector1, vector2):
    """Returns the distance between point 3-vectors vector1 and 2"""
    return len3(sub3(vector1, vector2))

def norm3(vector):
    """Returns unit length vector"""
    mag = len3(vector)
    return div3(vector, mag)

def dot3(vector1, vector2):
    """Returns dot product of vector1 and 2"""
    return np.dot(vector1, vector2)

def cross(vector1, vector2):
    """Returns cross product of two vectors"""
    return np.cross(vector1, vector2)

def project3(vector1, vector2):
    """Returns projection of vector1 onto unit vector2"""
    return mul3(vector, dot3(norm3(vector), float(d)))

def acosdot3(vector1, vector2):
    """Returns angle between vector1 and 2"""
    x = dot3(vector1, vector2)
    if x < -1.0: return pi
    elif x > 1.0: return 0.0
    else: return acos(x)

def rotate3(m, vector):
    """Returns the rotation of vector by 3x3 (row major) matrix m"""
    return np.multiply(vector, m) #matrix m might need to be transposed

def invert3x3(m):
    """Returns the inversion/transpose of 3x3 rotation matrix m"""
    return np.transpose(m)

def zaxis(m):
    """Returns the z-axis vector from (row major) rotation matrix m"""
    return m[:,2]

def calcRotMatrix(axis, angle):
    """Returns the row-major 3x3 rotation matrix defining a rotation around axis by angle"""
    cosTheta = cos(angle)
    sinTheta = sin(angle)
    t = 1.0 - cosTheta

    return mat2array(np.multiply(t, np.array([[pow(axis[0],2) + cosTheta, axis[0]*axis[1] - sinTheta*axis[2], axis[0]*axis[2] + sinTheta*axis[1]],
                                    [axis[0]*axis[1] + sinTheta*axis[2], pow(axis[1],2) + cosTheta, axis[1]*axis[2] - sinTheta*axis[0]],
                                    [axis[0]*axis[2] - sinTheta*axis[1], axis[1]*axis[2] + sinTheta*axis[0], pow(axis[2],2) + cosTheta]])))

def mat2array(matrix):
    return np.asarray(matrix).reshape(-1)

def makeOpenGLMatrix(rot, vec):
    """
    Returns an OpenGL-compatible (column-major, 4x4 homogenous) transformation
    matrix from ODE compatible (row-major, 3x3) rotation matrix rot, and position
    vector vec
    """
    return (rot[0], rot[3], rot[6], 0.0,
            rot[1], rot[4], rot[7], 0.0,
            rot[2], rot[5], rot[8], 0.0,
            vec[0], vec[1], vec[2], 1.0)

def prepare_GL():
    """ Setup basic OpenGL rendering """

    glClearColor(0.8, 0.8, 0.9, 0.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_NORMALIZE)
    glShadeModel(GL_SMOOTH)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(45.0, 1.3333, 0.2, 10.0)    
    #gluPerspective(55.0, 1.3333, 0.5, 20.0)
    #gluPerspective(25.0, 1.3333, 0.0, 20.0)

    glViewport(0, 0, 640, 480)

    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()

    glLightfv(GL_LIGHT0, GL_POSITION, [0, 0, 1, 0])
    glLightfv(GL_LIGHT0, GL_DIFFUSE, [1, 1, 1, 1])
    glLightfv(GL_LIGHT0, GL_SPECULAR, [1, 1, 1, 1])
    glEnable(GL_LIGHT0)

    glEnable(GL_COLOR_MATERIAL)
    glColor3f(0.8, 0.8, 0.8)

    #gluLookAt(1.5, 3.5, 3.5, -0.5, 1.0, 0.0, 0.0, 1.5, 0.0)
    gluLookAt(1.5, 4.0, 3.0, 0.5, 1.0, 0.0, 0.0, 1.5, 0.0)    


def createCapsule(world, space, density, length, radius):
    """Creates a capsule body and corresponding geometry"""
    
    """Create capsule body (aligned along z-axis so it matches the
    GeomCCylinder created below, which is z-axis-aligned by default
    """
    body = ode.Body(world)
    M = ode.Mass()
    M.setCylinder(density, 3, radius, length)
    body.setMass(M)

    # set parameters for drawing the body
    body.shape = "capsule"
    body.length = length
    body.radius = radius

    # create a capsule geometry for collision detectoin
    geom = ode.GeomCCylinder(space, radius, length)
    geom.setBody(body)

    return body, geom

def create_box(world, space, density, lx, ly, lz):
    """ Create a box body and its corresponding geometry """

    # Create body
    body = ode.Body(world)
    M = ode.Mass()
    M.setBox(density, lx, ly, lz)
    body.setMass(M)

    # Set parameters for drawing the body
    body.shape = "block"
    body.boxsize = (lx, ly, lz)

    # Create a box geometry for collision detection
    geom = ode.GeomBox(space, lengths=body.boxsize)
    geom.setBody(body)

    return body, geom

# Polygon resolution for capsule bodies
CAPSULE_SLICES = 16
CAPSULE_STACKS = 12
def draw_body(body):
    """ Draw an ODE body """
    
    #glPushMatrix()
    if body.shape == "capsule":
        rot = makeOpenGLMatrix(body.getRotation(), body.getPosition())
        glPushMatrix()
        glMultMatrixd(rot)
        cylHalfHeight = body.length / 2.0
        glBegin(GL_QUAD_STRIP)
        for i in range(0, CAPSULE_SLICES + 1):
            angle = i / float(CAPSULE_SLICES) * 2.0 * pi
            ca = cos(angle)
            sa = sin(angle)
            glNormal3f(ca, sa, 0)
            glVertex3f(body.radius * ca, body.radius * sa, cylHalfHeight)
            glVertex3f(body.radius * ca, body.radius * sa, -cylHalfHeight)

        glEnd()
        glTranslated(0, 0, cylHalfHeight)
        
        glutSolidSphere(body.radius, CAPSULE_SLICES, CAPSULE_STACKS)
        glTranslated(0, 0, -2.0 * cylHalfHeight)
        glutSolidSphere(body.radius, CAPSULE_SLICES, CAPSULE_STACKS)
    elif body.shape == "block":
        x,y,z = body.getPosition()
        R = body.getRotation()
        rot = [R[0], R[3], R[6], 0.,
               R[1], R[4], R[7], 0.,
               R[2], R[5], R[8], 0.,
               x, y, z, 1.0]
        glPushMatrix()
        glMultMatrixd(rot)
        sx, sy, sz = body.boxsize
        glScalef(sx, sy, sz)
        glutSolidCube(1)
    glPopMatrix()