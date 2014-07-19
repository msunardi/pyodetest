from common_utils import *

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
            body = ode.Body(self.world)
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

            body = ode.Body(self.world)
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

    def addHingeJoint(self, body1, body2, anchor, axis, paramvel=3, paramfmax=30):
        anchor = add3(anchor, self.offset)
        joint = ode.HingeJoint(self.world)
        joint.attach(body1, body2)
        joint.setAnchor(anchor)
        joint.setAxis(axis)
        joint.setParam(ode.ParamVel, paramvel)
        joint.setParam(ode.ParamFMax, paramfmax)

        joint.style = "hinge"
        self.joints.append(joint)

        return joint

    def update(self):
        pass



