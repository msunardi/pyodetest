from common_utils import *
from win32api import GetSystemMetrics as gsm

class Fubar():
    """
    Base class to create whole bodies
    To use: create a sub-class of this class, and specify the desired shapes
    """

    def __init__(self, world, space, density, offset = (0.0, 0.0, 0.0)):        

        self.world = world
        self.space = space
        self.density = density
        self.bodies = []
        self.geoms = []
        self.joints = []
        self.totalMass = 0.0

        self.offset = offset
        
    def createBody(self):
        pass
        
    def addBody(self, p1, p2, radius, dimension=(1.4, 0.1, 0.2), shape="block"):
        """ Adds a  body between joint positions p1 and p2 with given dimension to robot"""

        p1 = add3(p1, self.offset)
        p2 = add3(p2, self.offset)
        
        if (shape == "cylinder"):
            print "Creating cylinder ..."
            dist = dist3(p1, p2) - radius
            body = ode.Body(world)
            m = ode.Mass()        
            m.setCylinder(self.density, 3, radius, dist)
            body.setMass(m)

            # set parameters for drawing the body            
            body.shape = "capsule"
            body.length = dist
            body.radius = radius
            body.width = body.height = 0.2

            # create a capsule geom for collision detection
            geom = ode.GeomCCylinder(self.space, radius, dist)
            geom.setBody(body)
        else:
            print "Creating non-cylinder ..."
            dist = dist3(p1, p2)
            #lx = dist
            lx = dimension[0]                        
            ly = dimension[1]
            lz = dimension[2]

            body = ode.Body(world)
            M = ode.Mass()
            M.setBox(self.density, lx, ly, lz)
            body.setMass(M)

            # Set parameters for drawing the body
            body.shape = "block"
            body.boxsize = (lx, ly, lz)

            # Create a box geometry for collision detection
            geom = ode.GeomBox(self.space, lengths=body.boxsize)
            geom.setBody(body)
        
        
        # define body rotation automatically from body axis
        za = norm3(sub3(p2, p1))
        if (abs(dot3(za, (1.0, 0.0, 0.0))) < 0.7):
            xa = (1.0, 0.0, 0.0)
        else:
            xa = (0.0, 1.0, 0.0)
        ya = cross(za, xa)
        xa = norm3(cross(ya, za))
        ya = cross(za, xa)
        rot = (xa[0], ya[0], za[0], xa[1], ya[1], za[1], xa[2], ya[2], za[2])
        
        body.setPosition(mul3(add3(p1, p2), 0.5))
        body.setRotation(rot)

        self.bodies.append(body)
        self.geoms.append(geom)

        self.totalMass += body.getMass().mass

        return body

    def addFixedJoint(self, body1, body2):
        joint = ode.FixedJoint(self.world)
        joint.attach(body1, body2)
        joint.setFixed()

        joint.style = "fixed"
        self.joints.append(joint)

        return joint

    def addBallJoint(self, body1, body2, anchor):
        anchor = add3(anchor, self.offset)

        # Create the joint
        joint = ode.BallJoint(self.world)
        joint.attach(body1, body2)
        joint.setAnchor(anchor)

        joint.style = "ball"
        self.joints.append(joint)

        return joint

    def update(self):
        pass

class TestFigure(Fubar):

    def createBody(self):
        BROW_H = 1.68
        MOUTH_H = 1.53
        NECK_H = 1.50
        TOP_PLATE_DIM = (1.0, 0.75, 0.02)

        self.head = self.addBody((0.0, BROW_H, 0.0), (0.0, MOUTH_H, 0.0), 0.11, shape='cylinder')
        #self.neck = self.addBallJoint(self.chest, self.head, (0.0, NECK_H, 0.0))
        self.topPlate = self.addBody((0.0, 1.5, 0.0), (0.0, 1.2, 0.0), 0.2, dimension=TOP_PLATE_DIM)
        #return super(TestFigure, self).defineBody()        

def onKey(c, x, y):
    """  GLUT keyboard callback """
    global SloMo, Paused

    # Set simulation speed
    if c >= '0' and c <= '9':
        SloMo = 4 * int(c) + 1
    # Pause/unpause simulation
    elif c == 'p' or c == 'P':
        Paused = not Paused
    # quit
    elif c == 'q' or c == 'Q':
        sys.exit(0)


def onDraw():
    """ GLUT render callback """

    prepare_GL()
    for b in bodies:
        draw_body(b)
    for b in ragdoll.bodies:
        draw_body(b)

    glutSwapBuffers()

def onIdle():
    """ GLUT idle processing callback, performs ODE simulation step. """

    global Paused, lasttime, numiter

    if Paused:
        return

    t = dt - (time.time() - lasttime)
    if (t > 0):
        time.sleep(t)

    glutPostRedisplay()

    for i in range(stepsPerFrame):
        # Detect collisions and create contact joints
        space.collide((world, contactgroup), near_callback)

        # Simulation step (with slo motion)
        world.step(dt / stepsPerFrame / SloMo)

        numiter += 1

        # apply internal ragdoll forces
        ragdoll.update()

        # remove all contact joints
        contactgroup.empty()

    lasttime = time.time()

""" OTHER Methods """
def near_callback(args, geom1, geom2):
    """Callback function for the collide() method
    This function checks if the given geoms do collide and creates
    contact joints if they do
    """
    if(ode.areConnected(geom1.getBody(), geom2.getBody())):
       return

    # Check if the objects collide
    contacts = ode.collide(geom1, geom2)

    # Create contact joints
    world, contactgroup = args
    for c in contacts:
        c.setBounce(0.2)
        c.setMu(500)    # 0-5: very slippery, 50-500: normal, 5000: very sticky
        j = ode.ContactJoint(world, contactgroup, c)
        j.attach(geom1.getBody(), geom2.getBody())

def init_GLUT():        
    # intialize GLUT
    glutInit([])
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE)

    # Create the program window

    screenwidth = gsm(0)
    screenheight = gsm(1)
    width = 800
    height = 600
    x = (screenwidth - width)/2
    y = (screenheight - height)/2
    glutInitWindowPosition(x, y)
    glutInitWindowSize(width, height)
    glutCreateWindow("PyODE Ragdoll Simulation")

    glutKeyboardFunc(onKey)
    glutDisplayFunc(onDraw)
    glutIdleFunc(onIdle)

# Create an ODE world object
world = ode.World()
world.setGravity((0.0, -9.81, 0.0))
world.setERP(0.3)
world.setCFM(1E-5)

# Create an ODE space object
space = ode.Space()

# Create a plane geom to simulate a floor
floor = ode.GeomPlane(space, (0,1,0), 0)

# Create a list to store any ODE bofies which are not part of the
# ragdoll. (this is needed to avoid Python garbage collecting these bodies)
bodies = []


# Create a joint group for the contact joints generated during collisions
# between two bodies collide
contactgroup = ode.JointGroup()


# Simulation parameters
fps = 60
dt = 1.0 / fps
stepsPerFrame = 2
SloMo = 7
Paused = True
lasttime = time.time()
numiter = 0

ragdoll = TestFigure(world, space, 500, (0.0, 1.4, 0.0))
ragdoll.createBody()
print "Total mass is %.1f kg (%.1f lbs)" % (ragdoll.totalMass, ragdoll.totalMass * 2.2)
print "Ragdoll geoms: %d" % (len(ragdoll.geoms))

# create an obstacle
obstacle, obsgeom = createCapsule(world, space, 1000, 0.05, 0.15)
pos = (random.uniform(-0.3, 0.3), 0.2, random.uniform(-0.15, 0.2))
#pos = (0.27396178783269359, 0.20000000000000001, 0.17531818795388002)
obstacle.setPosition(pos)
obstacle.setRotation(rightRot)
bodies.append(obstacle)
print "obstacle created at %s" % (str(pos))

def main():
    init_GLUT()
    # Enter the GLUT event loop
    glutMainLoop()
#exit()
