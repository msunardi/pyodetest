from common_utils import *
from win32api import GetSystemMetrics as gsm
from Fubar import Fubar

Torque_incr = 5.3333
FMax_incr = 10.5

# GUI Params
g_fViewDistance = 9.
g_Width = 800
g_Height = 600

g_nearPlane = 1.
g_farPlane = 1000.

action = ""
xStart = yStart = 0.
zoom = 65.

xRotate = 0.
yRotate = 0.
zRotate = 0.

xTrans = 0.
yTrans = 0.



class TestFigure(Fubar):

    def createBody(self):
        BROW_H = 1.68
        MOUTH_H = 1.53
        NECK_H = 1.50
        TOP_PLATE_DIM = (1.0, 0.75, 0.02)

        SHOULDER_W = 0.41
        SHOULDER_H = 1.37
        UPPER_ARM_LEN = 0.75#0.30
        FORE_ARM_LEN = 0.65#0.25
        CHEST_H = 1.35
        CHEST_W = 0.36

        R_SHOULDER_POS = (-SHOULDER_W * 0.5, SHOULDER_H, 0.0)
        #R_ELBOW_POS = sub3(R_SHOULDER_POS, (UPPER_ARM_LEN, 0.0, 0.0))
        #R_WRIST_POS = sub3(R_ELBOW_POS, (FORE_ARM_LEN, 0.0, 0.0))
        R_ELBOW_POS = sub3(R_SHOULDER_POS, (0.0, UPPER_ARM_LEN, 0.0))
        R_WRIST_POS = sub3(R_ELBOW_POS, (0.0, FORE_ARM_LEN, 0.0))

        #self.chest = self.addBody((-CHEST_W * 0.5, CHEST_H, 0.0),
		#	(CHEST_W * 0.5, CHEST_H, 0.0), 0.13, shape="cylinder")
        #self.addFixedJoint(ode.environment, self.chest)
        self.head = self.addBody((0.0, BROW_H, 0.0), (0.0, MOUTH_H, 0.0), 0.11, shape='cylinder')
        #self.topArm = self.addBody((0.0, 0.0, 0.0), (0.1, 0.0, 0.0), 0.3, dimension=(1.0, 0.1, 0.1))
        self.topArm = self.addBody(R_SHOULDER_POS, R_ELBOW_POS, 0.08, shape="cylinder")#, dimension=(1.0, 0.1, 0.1))
        #self.rightShoulder = self.addBallJoint(self.chest, self.rightUpperArm,
        #	R_SHOULDER_POS)

        #self.addHingeJoint(ode.environment, self.topArm, (0, 0.5 ,0), (1, 0, 0))
        #self.addHingeJoint(self.chest, self.topArm, R_SHOULDER_POS, rightAxis)
        #self.addBallJoint(ode.environment, self.topArm, R_SHOULDER_POS, loStop=0.0, hiStop=0.6 * pi)
        self.shoulder = self.addHingeJoint(ode.environment, self.topArm, R_SHOULDER_POS, fwdAxis, paramvel=0, paramfmax=50, loStop=-0.8*pi, hiStop=0.9*pi)
        #self.addEnhancedBallJoint(self.chest, self.topArm, R_SHOULDER_POS, norm3((-1.0, -1.0, 4.0)), (0.0, 0.0, 1.0), pi * 0.5, 
        #                          pi * 0.25, 150.0, 100.0)
        self.lowArm = self.addBody(R_ELBOW_POS, R_WRIST_POS, 0.075, shape="cylinder")#, dimension=(1.0, 0.1, 0.1))
        #self.addBallJoint(self.topArm, self.lowArm, R_ELBOW_POS)
        self.addHingeJoint(self.topArm, self.lowArm, R_ELBOW_POS, fwdAxis, 
                           paramvel=0.0, paramfmax=0.0, loStop=0.0, hiStop=0.6 * pi)
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

    #prepare_GL()
    display()
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

        try:
            ragdoll.shoulder.setParam(ode.ParamVel, Torque)            
        except:
            print("Failed adding torque to shoulder")

        try:
            ragdoll.shoulder.setParam(ode.ParamFMax, FMax)
        except:
            print("Failed increasing FMax to shoulder")

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

"""UI control: http://carloluchessa.blogspot.com/2012/09/simple-viewer-in-pyopengl.html"""
def init():
    glEnable(GL_NORMALIZE)
    glLightfv(GL_LIGHT0,GL_POSITION,[ .0, 10.0, 10., 0. ] )
    glLightfv(GL_LIGHT0,GL_AMBIENT,[ .0, .0, .0, 1.0 ]);
    glLightfv(GL_LIGHT0,GL_DIFFUSE,[ 1.0, 1.0, 1.0, 1.0 ]);
    glLightfv(GL_LIGHT0,GL_SPECULAR,[ 1.0, 1.0, 1.0, 1.0 ]);
    glEnable(GL_LIGHT0)
    glEnable(GL_LIGHTING)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LESS)
    glShadeModel(GL_SMOOTH)
    resetView()

"""
def scenemodel():
    glRotate(90, 0., 0., 1.)
    glutSolidTeapot(1.)
"""

def resetView():
    global zoom, xRotate, yRotate, zRotate, xTrans, yTrans
    zoom = 65.
    xRotate = 0.
    yRotate = 0.
    zRotate = 0.
    xTrans = 0.
    yTrans = 0.
    glutPostRedisplay()

def display():
    # Clear frame buffer and depth buffer
    #glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glClearColor(0.8, 0.8, 0.9, 0.0)
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING)
    glEnable(GL_NORMALIZE)
    glShadeModel(GL_SMOOTH)
    # Set up viewing transformation, looking down -Z axis
    glLoadIdentity()
    #gluLookAt(0, 0, -g_fViewDistance, 0, 0, 0, -.1, 0, 0)   #-.1,0,0
    gluLookAt(1.5, 4.0, 3.0, 0.5, 1.0, 0.0, 0.0, 1.5, 0.0) 
    # Set perspective (also zoom)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(zoom, float(g_Width)/float(g_Height), g_nearPlane, g_farPlane)
    glMatrixMode(GL_MODELVIEW)
    # Render the scene
    polarView()
    #scenemodel()
    # Make sure changes appear onscreen
    #glutSwapBuffers()

def reshape(width, height):
    global g_Width, g_Height
    g_Width = width
    g_Height = height
    glViewport(0, 0, g_Width, g_Height)
    

def polarView():
    glTranslatef( yTrans/100., 0.0, 0.0 )
    glTranslatef(  0.0, -xTrans/100., 0.0)
    glRotatef( -zRotate, 0.0, 0.0, 1.0)
    glRotatef( -xRotate, 1.0, 0.0, 0.0)
    glRotatef( -yRotate, .0, 1.0, 0.0)
   
def keyboard(key, x, y):
    """  GLUT keyboard callback """
    global zTr, yTr, xTr, SloMo, Paused, Torque, FMax

    if(key=='r'): resetView()
    if(key=='q' or key == 'Q'): sys.exit(0)    
    
    # Set simulation speed
    if key >= '0' and key <= '9':
        SloMo = 4 * int(key) + 1
    # Pause/unpause simulation
    if key == 'p' or key == 'P':
        Paused = not Paused
        print "Paused: %s" % (Paused)
    # Add torque to shoulder joint:
    elif key == 's':
        print("Increasing torque to: %s" % Torque)
        Torque += Torque_incr
    elif key == 'd':
        print("Decreasing torque to: %s" % Torque)
        Torque -= Torque_incr
    elif key == 'w':
        print("Increasing FMax to: %s" % FMax)
        FMax += FMax_incr
    elif key == 'W':
        print("Decreasing FMax to: %s" % FMax)
        FMax -= FMax_incr
    glutPostRedisplay()


def mouse(button, state, x, y):
    global action, xStart, yStart
    if (button==GLUT_LEFT_BUTTON):
        if (glutGetModifiers() == GLUT_ACTIVE_SHIFT):
            action = "MOVE_EYE_2"
        else:
            action = "MOVE_EYE"
    elif (button==GLUT_MIDDLE_BUTTON):
        action = "TRANS"
    elif (button==GLUT_RIGHT_BUTTON):
        action = "ZOOM"
    xStart = x
    yStart = y


def motion(x, y):
    global zoom, xStart, yStart, xRotate, yRotate, zRotate, xTrans, yTrans
    if (action=="MOVE_EYE"):
        xRotate += x - xStart
        yRotate -= y - yStart
    elif (action=="MOVE_EYE_2"):
        zRotate += y - yStart
    elif (action=="TRANS"):
        xTrans += x - xStart
        yTrans += y - yStart
    elif (action=="ZOOM"):
        zoom -= y - yStart
        if zoom > 150.:
            zoom = 150.
        elif zoom < 1.1:
            zoom = 1.1
    else:
        print("unknown action\n", action)
    xStart = x
    yStart = y 
    glutPostRedisplay()

def glSetup():
    # GLUT Window Initialization
    glutInit()
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB| GLUT_DEPTH)      # zBuffer
    glutInitWindowSize (g_Width,g_Height) 
    glutInitWindowPosition (0 + 4, g_Height / 4)
    glutCreateWindow ("GLODE View")
    # Initialize OpenGL graphics state
    init ()
    # Register callbacks
    glutReshapeFunc(reshape)
    #glutDisplayFunc(display)    
    glutDisplayFunc(onDraw)
    glutMouseFunc(mouse)
    glutMotionFunc(motion)
    glutKeyboardFunc(keyboard)
    glutIdleFunc(onIdle)
    #printHelp()
    glutMainLoop()

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
SloMo = 1
Paused = True
lasttime = time.time()
numiter = 0
Torque = 0.0
FMax = 30.0

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
    #init_GLUT()
    glSetup()
    # Enter the GLUT event loop
    #glutMainLoop()
#exit()

