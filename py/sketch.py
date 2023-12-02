# from casadi import MX, SX, vertcat, mtimes, Function, sqrt, norm_2
from casadi import *
from p5 import *

pos = None
vel = None
accel = None
vel_dampening = 0.9
vel_limit = 10
accel_thrust = 1
rad = 20
U_sol = None
u_idx = None
wp_thresh = 50
path = None

target_pos = None

def setup():
    size(1000, 1000)
    global pos, vel, accel, target_pos, path
    pos = Vector(width / 2, height / 2)
    vel = Vector(0, 0)
    accel = Vector(0, 0)
    target_pos = Vector(width - 200, height / 2)
    path = diamondPath()

keys = {}
def key_pressed():
    global keys
    keys[key.name.upper()] = True
def key_released():
    global keys
    keys[key.name.upper()] = False
    if key.name.upper() == "S":
        opti_get_frame_input()
def key_is_down(k):
    global keys
    return keys.get(str(k).upper(), False)

def key_update():
    global accel, accel_thrust
    if key_is_down(LEFT):
        accel.x = -accel_thrust 
    if key_is_down(RIGHT):
        accel.x += accel_thrust 
    if key_is_down("UP"):
        accel.y = -accel_thrust 
    if key_is_down("DOWN"):
        accel.y += accel_thrust 

def pos_update():
    global accel, vel, vel_dampening, vel_limit, pos
    vel += accel
    vel *= vel_dampening
    vel.limit(vel_limit)
    pos += vel
    if pos.x > width:
        pos.x = 0
    if pos.x < 0:
        pos.x = width

def opti_get_frame_input():
    global pos, vel, accel
    global vel_limit, accel_thrust, vel_dampening
    N = 100 # number of frames to compute ahead of time.
    opti = Opti() # Optimization problem

    # ---- decision variables ---------
    X       = opti.variable(4,N+1) # state trajectory
    posx    = X[0,:]
    posy    = X[1,:]
    velx    = X[2,:]
    vely    = X[3,:]
    # acceleration x
    U = opti.variable(2,N)  # control trajectory (throttle x,y)
    Ux = U[0,:]
    Uy = U[1,:]
    # T = opti.variable()      # final time

    # ---- parameters ---------
    goal_px = opti.parameter()
    opti.set_value(goal_px, target_pos.x)

    # ---- objective          ---------
    # opti.minimize(T) # race in minimal time
    opti.minimize((posx[-1] - target_pos.x) ** 2 + (posy[-1] - target_pos.y) ** 2) # minimize dist to the target

    # ---- dynamic constraints --------
    # f = lambda x,u: vertcat(x[1],u-x[1]) # dx/dt = f(x,u)
    xdot = lambda x, u: vertcat(x[1], x[1] + u[0])

    for k in range(N): # loop over control intervals
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
       opti.subject_to(X[:,k+1]==x_next)
       # Enforce physics limits
       opti.subject_to((velx_k ** 2 + vely_k ** 2) <= vel_limit ** 2)
       # Enforce actuator limits
       opti.subject_to((Ux_k ** 2 + Uy_k ** 2) <= accel_thrust ** 2)

    # ---- boundary conditions --------
    opti.subject_to(posx[0]==pos.x) # start at current position.
    opti.subject_to(posy[0]==pos.y) # start at current position.
    opti.subject_to(velx[0]==vel.x) # with current vel
    opti.subject_to(vely[0]==vel.y) # with current vel
    opti.subject_to(posx[-1]==target_pos.x)  # finish at target
    opti.subject_to(posy[-1]==target_pos.y)  # finish at target
    opti.subject_to(velx[-1]==0) # at a stop
    opti.subject_to(vely[-1]==0) # at a stop
    opti.subject_to(velx[-2]==0) # at a stop
    opti.subject_to(vely[-2]==0) # at a stop

    # ---- initial values for solver ---
    opti.set_initial(velx, -1)

    # ---- solve NLP              ------
    p_opts = dict(print_time=False, verbose=False)
    s_opts = dict(print_level=0)
    opti.solver("ipopt", p_opts, s_opts) # set numerical backend
    sol = opti.solve()   # actual solve
    global U_sol, u_idx
    U_sol = sol.value(U)
    u_idx = 0

def ideal_input():
    global u_idx
    if u_idx is None:
        return
    background(100)
    U_k = U_sol[:,u_idx]
    Ux_k = U_k[0]
    Uy_k = U_k[1]
    accel.x = Ux_k
    accel.y = Uy_k
    u_idx += 1
    if u_idx >= U_sol.shape[1]-1:
        u_idx = None

def draw():
    global pos, vel, accel, path
    background(50)
    if mouse_is_pressed:
        target_pos.x = mouse_x
        target_pos.y = mouse_y
    accel.x = 0
    accel.y = 0
    ideal_input();
    key_update()
    pos_update()
    path.update(pos)
    path.draw()
    fill(255)
    ellipse(pos.x, pos.y, rad, rad)
    fill(0, 255, 0)
    ellipse(target_pos.x, target_pos.y, rad, rad)

class GoalPath:
    def __init__(self):
        global wp_thresh
        self.waypoints = []
        self.active_idx = 0
        self.dist_to_pass = wp_thresh
    def add_waypoint(self, x, y):
        self.waypoints.append(Vector(x, y));
    def get_cur_waypoint(self):
        return self.waypoints[self.active_idx];
    def get_next_n(self, N):
        ret = []
        idx = self.active_idx
        for i in range(N):
            ret.push(self.waypoints[idx])
            idx += 1
            idx %= self.waypoints.length
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
