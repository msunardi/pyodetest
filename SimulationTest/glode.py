from common_utils import *
from win32api import GetSystemMetrics as gsm
from Fubar import Fubar

class TestFigure(Fubar):

    def createBody(self):
        BROW_H = 1.68
        MOUTH_H = 1.53
        NECK_H = 1.50
        TOP_PLATE_DIM = (1.0, 0.75, 0.02)

        self.head = self.addBody((0.0, BROW_H, 0.0), (0.0, MOUTH_H, 0.0), 0.11, shape='cylinder')
        self.topPlate = self.addBody((0.0, 1.5, 0.0), (0.0, 1.2, 0.0), 0.2, dimension=TOP_PLATE_DIM)
        self.addHingeJoint(self.topPlate, ode.environment, (0,2,0), (0,0,1))
        """
        j1 = ode.HingeJoint(self.world)
        j1.attach(self.head, ode.environment)
        j1.setAnchor((0,1,0))
        j1.setAxis((0,0,1))
        j1.setParam(ode.ParamVel, 3)
        j1.setParam(ode.ParamFMax, 30)
        """

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

