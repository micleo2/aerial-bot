# from casadi import MX, SX, vertcat, mtimes, Function, sqrt, norm_2
from p5 import *
import casadi as ca

rocket = None
path = None
controller = None
rad = 20
U_sol = None
u_idx = None
wp_thresh = 50

# GRAVITY = 0.175
GRAVITY = 0
# BOOST_ACCEL = 0.4
BOOST_ACCEL = 0.1
YAW_ACCEL = 0.008
ANG_VEL_DAMPING = 0.90
MAX_ANG_VEL = 0.15
MAX_VEL = 10

def setup():
    size(1000, 1000)
    global path, rocket, controller
    rocket = Rocket(width / 2, height / 2)
    controller = ControllerInput()
    path = diamondPath()

keys = {}
def key_pressed():
    global keys
    keys[key.name.upper()] = True
def key_released():
    global keys
    keys[key.name.upper()] = False
    if key.name.upper() == "S":
        solve_input(path.get_next_n(2))
    if key.name.upper() == "R":
        rocket.pos.x = width / 2
        rocket.pos.y = height / 2
def key_is_down(k):
    global keys
    return keys.get(str(k).upper(), False)

def solve_input(waypnts):
    global rocket, wp_thresh
    # Number of frames to compute ahead of time, the horizon width.
    N = 200
    opti = ca.Opti()

    # ---- decision variables ---------
    # state trajectory
    X       = opti.variable(6,N+1)
    posx      = X[0,:]
    posy      = X[1,:]
    velx      = X[2,:]
    vely      = X[3,:]
    theta     = X[4,:]
    theta_dot = X[5,:]
    dists     = opti.variable(N)
    # ---- control variables ---------
    # yaw, hold_boost
    U = opti.variable(2,N)
    Uyaw = U[0,:]
    Uboost = U[1,:]

    # ---- objective          ---------
    opti.minimize(ca.sum1(dists))

    for k in range(N):
       # ---- dynamic constraints --------
        x_k = X[:,k]
        posx_k = x_k[0]
        posy_k = x_k[1]
        velx_k = x_k[2]
        vely_k = x_k[3]
        theta_k = x_k[4]
        theta_dot_k = x_k[5]
        U_k = U[:,k]
        Uyaw_k = U_k[0]
        Uboost_k = U_k[1]
        step_boost = 0.5 + 0.5 * (ca.tanh(500 * Uboost_k - 20))
        velx_next = velx_k + step_boost * ca.cos(theta_k) * BOOST_ACCEL
        vely_next = vely_k + step_boost * -ca.sin(theta_k) * BOOST_ACCEL
        theta_dot_next = (theta_dot_k + Uyaw_k * YAW_ACCEL) * ANG_VEL_DAMPING
        x_next = ca.vertcat(
            posx_k + velx_k,
            posy_k + vely_k,
            velx_next,
            vely_next,
            theta_k + theta_dot_k,
            theta_dot_next
            )
        # Apply dynamics as a constraint
        opti.subject_to(X[:,k+1] == x_next)
        # Enforce limits
        opti.subject_to(opti.bounded(-MAX_VEL, velx_k, MAX_VEL))
        opti.subject_to(opti.bounded(-MAX_VEL, vely_k, MAX_VEL))
        opti.subject_to((velx_k ** 2 + vely_k ** 2) <= MAX_VEL ** 2)
        opti.subject_to(opti.bounded(-MAX_ANG_VEL, theta_dot_k, MAX_ANG_VEL))
        opti.subject_to(theta_dot_k >= -MAX_ANG_VEL)
        opti.subject_to(theta_dot_k <= MAX_ANG_VEL)
        dist_to_target = (posx_k - waypnts[0].x) ** 2 + (posy_k - waypnts[0].y) ** 2
        opti.subject_to(dists[k] == dist_to_target)

    pos = rocket.pos
    vel = rocket.vel
    opti.subject_to(posx[0] == pos.x) # start at current position.
    opti.subject_to(posy[0] == pos.y) # start at current position.
    opti.subject_to(velx[0] == vel.x) # with current vel
    opti.subject_to(vely[0] == vel.y) # with current vel
    opti.subject_to(theta[0] == rocket.angle)
    opti.subject_to(theta_dot[0] == rocket.angular_vel)

    opti.subject_to(dists[-1] <= wp_thresh)
    opti.subject_to(opti.bounded(0, Uboost, 1))
    opti.subject_to(Uboost >= 0)
    opti.subject_to(Uboost <= 1)
    opti.subject_to(opti.bounded(-1, Uyaw, 1))
    opti.subject_to(Uyaw >= -1)
    opti.subject_to(Uyaw <= 1)

    p_opts = dict(print_time=False, verbose=False)
    s_opts = dict(print_level=0, tol=1e-3)
    opti.solver("ipopt", p_opts, s_opts)
    sol = opti.solve()
    global U_sol, u_idx
    U_sol = sol.value(U)
    u_idx = 0

def rocket_ideal_input(ctrl):
    global u_idx
    if u_idx is None:
        return
    background(100)
    U_k = U_sol[:,u_idx]
    Uyaw_k = U_k[0]
    Uboost_k = U_k[1]
    ctrl.yaw = Uyaw_k
    ctrl.hold_boost = Uboost_k > 0.01
    u_idx += 1
    if u_idx >= U_sol.shape[1]-1:
        u_idx = None

def draw():
    global path, rocket, controller
    background(50)
    controller.set_from_user();
    rocket_ideal_input(controller)
    rocket.updateWithInput(controller);
    path.update(rocket.pos)
    path.draw()
    rocket.draw()

class ControllerInput:
    def __init__(self):
        # 0 XOR 1.
        self.hold_boost = False
        # From [-1, 1].
        self.yaw = 0
    def set_from_user(self):
        self.hold_boost = False
        self.yaw = 0
        if key_is_down("RIGHT"):
            self.yaw -= 1
        if key_is_down("LEFT"):
            self.yaw += 1
        if key_is_down("SPACE") or key_is_down("UP"):
            self.hold_boost = True

class Rocket:
    def __init__(self, x, y):
        self.pos = Vector(x, y)
        self.vel = Vector(0, 0)
        self.accel = Vector(0, 0)
        self.angle = PI / 2
        self.angular_vel = 0
        self.angular_accel = 0
        self.width = 40
        self.height = self.width * 2.5
        self.boosting = False

    def updateWithInput(self, controller):
        # Angular section
        self.angle += self.angular_vel
        self.angular_vel = (self.angular_vel + controller.yaw * YAW_ACCEL) * ANG_VEL_DAMPING;
        self.angular_vel = max(-MAX_ANG_VEL, min(self.angular_vel, MAX_ANG_VEL))
        # Translation section
        self.pos += self.vel
        self.accel *= 0
        self.accel.y = GRAVITY
        self.boosting = controller.hold_boost
        if controller.hold_boost:
            thrust = Vector(cos(self.angle), -sin(self.angle))
            thrust *= BOOST_ACCEL
            self.accel += thrust
        self.vel += self.accel
        # Clamp vel.
        self.vel.limit(MAX_VEL)
        # self.wrapBounds()

    def wrapBounds(self):
        if self.pos.y > height:
            self.pos.y = 0
        if self.pos.y < 0:
            self.pos.y = height

    def draw(self):
        push()
        translate(self.pos.x, self.pos.y)
        rotate(-self.angle + PI/2)

        fill(200)
        stroke_weight(1)
        stroke(0)
        rect(-self.width/2, -self.height/2, self.width, self.height)
        stroke(255, 0, 0)
        stroke_weight(1)
        line(0, 0, 0, -self.height/2)
        stroke(0)

        if self.boosting:
            w = self.width/3
            fill(200, 100, 100)
            push()
            translate(0, self.height/2)
            triangle(w, 0, -w, 0, 0, self.height/4)
            pop()
            if self.vel.mag() >= (MAX_VEL - 0.01):
                stroke(255)
                stroke_weight(2)
                yend = -self.height / 2 - 10
                xbegin = -self.width/2-10
                line(xbegin, 0, 0, yend)
                line(-xbegin, 0, 0, yend)

        # draw ground truth point
        fill(0)
        stroke_weight(5)
        stroke(0)
        point(0, 0)

        pop()

class GoalPath:
    def __init__(self):
        global wp_thresh
        self.waypoints = []
        self.active_idx = 0
        self.dist_to_pass = wp_thresh
    def add_waypoint(self, x, y):
        self.waypoints.append(Vector(x, y))
    def get_cur_waypoint(self):
        return self.waypoints[self.active_idx]
    def get_next_n(self, N):
        ret = []
        idx = self.active_idx
        for i in range(N):
            ret.append(self.waypoints[idx])
            idx += 1
            idx %= len(self.waypoints)
        return ret
    def draw(self):
        N = len(self.waypoints)
        for i in range(N):
            wpnt = self.waypoints[i]
            no_stroke()
            fill(200, 0, 0)
            if i == self.active_idx:
                fill(0, 150, 0)
            ellipse(wpnt.x, wpnt.y, self.dist_to_pass * 2, self.dist_to_pass * 2)
    def update(self, pos):
        curTarget = self.get_cur_waypoint()
        if Vector.dist(curTarget, pos) < self.dist_to_pass:
            self.active_idx += 1
            self.active_idx %= len(self.waypoints)

def diamondPath():
  path = GoalPath()
  path.add_waypoint(width / 2, 200)
  path.add_waypoint(200, height / 2)
  path.add_waypoint(width / 2, height - 200)
  path.add_waypoint(width - 200, height / 2)
  return path

if __name__ == '__main__':
    run()

