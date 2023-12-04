# from casadi import MX, SX, vertcat, mtimes, Function, sqrt, norm_2
from casadi import *
from p5 import *

rocket = None
path = None
controller = None
rad = 20
U_sol = None
u_idx = None
wp_thresh = 50

GRAVITY = 0.175
BOOST_ACCEL = 0.4
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
    # if key.name.upper() == "S":
    #     simple_input(path.get_next_n(2))
def key_is_down(k):
    global keys
    return keys.get(str(k).upper(), False)

def simple_input(waypnts):
    global pos, vel, accel
    global vel_limit, accel_thrust, vel_dampening
    # Number of frames to compute ahead of time, the horizon width.
    N = 250 
    opti = Opti()

    # ---- decision variables ---------
    # state trajectory
    X       = opti.variable(4,N+1)
    posx    = X[0,:]
    posy    = X[1,:]
    velx    = X[2,:]
    vely    = X[3,:]
    progress = opti.variable(N)
    # ---- control variables ---------
    # acceleration x,y
    U = opti.variable(2,N)
    Ux = U[0,:]
    Uy = U[1,:]


    # ---- objective          ---------
    opti.minimize((posx[-1] - waypnts[0].x) ** 2 + (posy[-1] - waypnts[0].y) ** 2)

    for k in range(N):
       # ---- dynamic constraints --------
       x_k = X[:,k]
       posx_k = x_k[0]
       posy_k = x_k[1]
       velx_k = x_k[2]
       vely_k = x_k[3]
       U_k = U[:,k]
       Ux_k = U_k[0]
       Uy_k = U_k[1]
       x_next = vertcat(
               posx_k + velx_k,
               posy_k + vely_k,
               (velx_k + Ux_k) * vel_dampening,
               (vely_k + Uy_k) * vel_dampening)
       # Apply dynamics as a constraint
       opti.subject_to(X[:,k+1] == x_next)
       # Enforce physics limits
       opti.subject_to((velx_k ** 2 + vely_k ** 2) <= vel_limit ** 2)
       # Enforce actuator limits
       opti.subject_to((Ux_k ** 2 + Uy_k ** 2) <= accel_thrust ** 2)
       #progress
       # cur_dist = (posx_k - waypnts[0].x) ** 2 + (posy_k - waypnts[0].y) ** 2
       # opti.subject_to()

    # ---- boundary conditions --------
    # state conditions
    opti.subject_to(posx[0] == pos.x) # start at current position.
    opti.subject_to(posy[0] == pos.y) # start at current position.
    opti.subject_to(velx[0] == vel.x) # with current vel
    opti.subject_to(vely[0] == vel.y) # with current vel
    opti.subject_to(posx[-1] == waypnts[0].x)  # finish at target
    opti.subject_to(posy[-1] == waypnts[0].y)  # finish at target

    # ---- initial values for solver ---
    opti.set_initial(velx, -1)

    # ---- solve NLP              ------
    p_opts = dict(print_time=False, verbose=False)
    s_opts = dict(print_level=0, tol=1e-4)
    opti.solver("ipopt", p_opts, s_opts) # set numerical backend
    # budget of ~18ms.
    sol = opti.solve()   # actual solve
    global U_sol, u_idx
    U_sol = sol.value(U)
    u_idx = 0

def progress_input(waypnts):
    global pos, vel, accel
    global vel_limit, accel_thrust, vel_dampening
    # Number of frames to compute ahead of time, the horizon width.
    N = 250 
    NW = len(waypnts)
    opti = Opti()

    # ---- decision variables ---------
    # state trajectory
    X       = opti.variable(4,N+1)
    posx    = X[0,:]
    posy    = X[1,:]
    velx    = X[2,:]
    vely    = X[3,:]
    # ---- progress variables ---------
    # each waypoint has a 'progress' state. This state begins at 1, and will be
    # decremented to 0 when the waypoint is marked as completed.
    progress = opti.variable(NW,N+1)
    progress_change = opti.variable(NW,N+1)
    slack_dist = opti.variable(NW,N+1)
    allowed_slack = opti.parameter()
    opti.set_value(allowed_slack, wp_thresh)
    # ---- control variables ---------
    # acceleration x,y
    U = opti.variable(2,N)
    Ux = U[0,:]
    Uy = U[1,:]

    # ---- objective          ---------
    opti.minimize((posx[-1] - waypnts[-1].x) ** 2 + (posy[-1] - waypnts[-1].y) ** 2)

    # f = lambda x,u: vertcat(x[1],u-x[1]) # dx/dt = f(x,u)
    xdot = lambda x, u: vertcat(x[1], x[1] + u[0])

    for k in range(N):
       # ---- dynamic constraints --------
       x_k = X[:,k]
       posx_k = x_k[0]
       posy_k = x_k[1]
       velx_k = x_k[2]
       vely_k = x_k[3]
       U_k = U[:,k]
       Ux_k = U_k[0]
       Uy_k = U_k[1]
       x_next = vertcat(
               posx_k + velx_k,
               posy_k + vely_k,
               (velx_k + Ux_k) * vel_dampening,
               (vely_k + Uy_k) * vel_dampening)
       # Apply dynamics as a constraint
       opti.subject_to(X[:,k+1] == x_next)
       # Enforce physics limits
       opti.subject_to((velx_k ** 2 + vely_k ** 2) <= vel_limit ** 2)
       # Enforce actuator limits
       opti.subject_to((Ux_k ** 2 + Uy_k ** 2) <= accel_thrust ** 2)

       # ---- progress constraints --------
       prog_k = progress[:, k]
       prog_change_k = progress_change[:, k]
       prog_next = prog_k - prog_change_k
       opti.subject_to(progress[:, k+1] == prog_next)
       for j in range(NW):
           # CPC
           waypnt_pos = waypnts[j]
           dist_to_waypnt = (posx_k - waypnt_pos.x) ** 2 + (posy_k - waypnt_pos.y) ** 2
           # allowed_slack = slack_dist[j, k]
           opti.subject_to(prog_change_k[j] * (dist_to_waypnt * allowed_slack) == 0)
           opti.subject_to(opti.bounded(0, allowed_slack, wp_thresh))
           # enforce waypoints are visited in order.
           if j < NW - 1:
               opti.subject_to(prog_k[j] <= prog_k[j+1])
       opti.subject_to(opti.bounded(0, prog_k, 1))
       opti.subject_to(opti.bounded(0, prog_change_k, 1))

    # ---- boundary conditions --------
    # state conditions
    opti.subject_to(posx[0] == pos.x) # start at current position.
    opti.subject_to(posy[0] == pos.y) # start at current position.
    opti.subject_to(velx[0] == vel.x) # with current vel
    opti.subject_to(vely[0] == vel.y) # with current vel
    # opti.subject_to(posx[-1] == waypnts[-1].x)  # finish at target
    # opti.subject_to(posy[-1] == waypnts[-1].y)  # finish at target
    # opti.subject_to(velx[-1] == 0) # at a stop
    # opti.subject_to(vely[-1] == 0) # at a stop
    # opti.subject_to(velx[-2] == 0) # at a stop
    # opti.subject_to(vely[-2] == 0) # at a stop

    # ---- progress conditions --------
    # progress variables all begin at 1
    opti.subject_to(progress[:, 0] == 1)
    # and all end at 0, meaning all waypoints have been visited.
    opti.subject_to(progress[:, -1] == 0)

    # ---- initial values for solver ---
    opti.set_initial(velx, -1)

    # ---- solve NLP              ------
    p_opts = dict(print_time=False, verbose=False)
    s_opts = dict(print_level=0)
    opti.solver("ipopt", p_opts, s_opts) # set numerical backend
    # budget of ~18ms.
    sol = opti.solve()   # actual solve
    print("progress sol", sol.value(progress))
    global U_sol, u_idx
    U_sol = sol.value(U)
    u_idx = 0

def rocket_ideal_input(ctrl):
    pass

def draw():
    global path, rocket, controller
    background(50)
    controller.set_from_user();
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
        self.angular_accel = controller.yaw * YAW_ACCEL
        self.angular_vel += self.angular_accel
        self.angular_vel *= ANG_VEL_DAMPING
        # Clamp angular vel.
        if abs(self.angular_vel) > MAX_ANG_VEL:
            self.angular_vel = MAX_ANG_VEL * (1 if self.angular_vel > 0 else -1)
        elif abs(self.angular_vel) < 0.00001:
            self.angular_vel = 0
        self.angle += self.angular_vel
        if self.angle > PI:
            self.angle = -PI
        elif self.angle < -PI:
            self.angle = PI

        # Translation section
        # Re-compute acceleration from scratch every frame.
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
        self.pos += self.vel
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
