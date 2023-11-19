var enableBot = true;

const GRAVITY = 0.175;
// const GRAVITY = 0;
const BOOST_ACCEL = 0.4;
const YAW_ACCEL = 0.002;
const ANG_VEL_DAMPING = 0.90;
const MAX_ANG_VEL = 0.125;
const MAX_VEL = 10;
const SPACE = 32; // key code
const Vec = p5.Vector;

let c;
let rocket;
let bot;
let path;
let planner;
let controller;
let lapDisplay;
let disabled = false;

let thetaErrSamples;
let derivativeErrSamples;

function clamp(a, min, max) {
  if (a < min){
    return min;
  } else if (a > max){
    return max;
  }
  return a;
}
function trunc(n, d = 1000) {
  return int(n * d) / d;
}

function smoothstep(edge0, edge1, x) {
  x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
  return x * x * (3 - 2 * x);
}

function vecMix(a, b, fac) {
  let aP = p5.Vector.mult(a, 1 - fac); 
  let bP = p5.Vector.mult(b, fac); 
  return p5.Vector.add(aP, bP);
}

function mix(a, b, fac) {
  return a * (1-fac) + b * fac;
}

function mapb(x, min, max) {
  return map(x, 0, 1, min, max);
}

function evalBezier(t, p0, p1, p2, p3) {
  p0 = p0.copy();
  p1 = p1.copy();
  p2 = p2.copy();
  p3 = p3.copy();
  var one = 1 - t;
  p0.mult(Math.pow(one, 3));
  p1.mult(3 * Math.pow(one, 2) * t);
  p2.mult(3 * one * Math.pow(t, 2));
  p3.mult(Math.pow(t, 3));
  return p0.add(p1).add(p2).add(p3);
}

function setup() {
  frameRate(60);
  createCanvas(1000, 1000);
  globalThis.context = this;
  c = createVector;

  rocket = new Rocket(width/2, 200);
  controller = new ControllerInput();
  bot = new AerialBot(rocket);
  path = diamondPath();
  planner = new LinePositionPlanner(path, rocket, bot);
  lapDisplay = new LapTimeDisplay(path, rocket, controller);
}

function draw() {
  background(50);

  if (!disabled) {
    controller.setFromUser();
    if (enableBot){
      bot.update(controller);
    }
    rocket.updateWithInput(controller);
    planner.update();
    path.update(rocket.position());
    lapDisplay.update();
  }

  // textSize(22);
  // fill(255, 0, 0);
  // controller.debugDraw();
  // rocket.debugDraw();

  // const amp = 200;
  // const phaseMult = 0.01;
  // const phi = frameCount * phaseMult;
  // let x = cos(phi) * amp;
  // let y = sin(phi) * amp;
  // botTarget.x = x + width / 2;
  // botTarget.y = y + height / 2;
  // bot.setTarget(botTarget);
  // noFill();
  // stroke(255, 0, 0, 150);
  // ellipse(width/2, height/2, amp * 2, amp * 2);

  // let pnt = evalBezier((sin(frameCount / 200) + 1) * 0.5, c(width / 2, 200), c(200, height / 2), c(width / 2, height - 200), c(width - 200, height / 2));
  // botTarget.x = pnt.x;
  // botTarget.y = pnt.y;
  // noFill();
  // bezier(width / 2, 200, 200, height / 2, width / 2, height - 200, width - 200, height / 2);

  if (mouseIsPressed) {
    bot.setTarget(c(mouseX, mouseY));
  }

  path.draw();
  rocket.draw();
  lapDisplay.draw();
  drawTarget();
}

function keyPressed() {
  if (key == 'p') {
    disabled = true;
  }
}

function drawTarget() {
  stroke(0, 255, 0);
  strokeWeight(7);
  point(bot.getTarget().x, bot.getTarget().y);
  strokeWeight(1);
}

class ControllerInput {
  constructor() {
    // 0 XOR 1.
    this.holdBoost = false;
    // From [-1, 1].
    this.yaw = 0;
  }

  debugDraw() {
    var str = "yaw " + trunc(this.yaw) + " holdBoost " + this.holdBoost;
    fill(255, 0, 0);
    textSize(22);
    text(str, 12, 120);
  }

  setFromUser() {
    this.holdBoost = false;
    this.yaw = 0;
    if (keyIsDown(RIGHT_ARROW)) {
      this.yaw--;
    }
    if (keyIsDown(LEFT_ARROW)) {
      this.yaw++;
    }
    if (keyIsDown(SPACE) || keyIsDown(UP_ARROW)) {
      this.holdBoost = true;
    }
  }
}

class Rocket {
  constructor(x, y) {
    this.pos = createVector(x, y);
    this.vel = createVector(0, 0);
    this.accel = createVector(0, 0);
    this.angle = PI / 2;
    this.angularVel = 0;
    this.angularAccel = 0;
    this.width = 40;
    this.height = this.width * 2.5;
    this.boosting = false;
  }

  updateWithInput(controller) {
    // Angular section
    this.angularAccel = controller.yaw * YAW_ACCEL;
    if (this.angularAccel != 0) {
      this.angularVel += this.angularAccel;
    } else {
      // If the user isn't putting in any controller, dampen the velocity.
      this.angularVel *= ANG_VEL_DAMPING;
    }
    // Clamp angular vel.
    if (abs(this.angularVel) > MAX_ANG_VEL) {
      this.angularVel = MAX_ANG_VEL * (this.angularVel > 0 ? 1 : -1);
    } else if (abs(this.angularVel) < 0.00001) {
      this.angularVel = 0;
    }
    this.angle += this.angularVel;
    if (this.angle > PI) {
      this.angle = -PI;
    } else if (this.angle < -PI) {
      this.angle = PI;
    }

    // Translation section
    // Re-compute acceleration from scratch every frame.
    this.accel.set(); // clear accel
    this.accel.y = GRAVITY;
    this.boosting = controller.holdBoost;
    if (controller.holdBoost) {
      let thrust = createVector(cos(this.angle), -sin(this.angle));
      thrust.mult(BOOST_ACCEL);
      this.accel.add(thrust);
    }
    this.vel.add(this.accel);
    // Clamp vel.
    if (this.vel.mag() > MAX_VEL) {
      this.vel.setMag(MAX_VEL);
    }
    this.pos.add(this.vel);
    this.wrapBounds();
  }

  wrapBounds() {
    if (this.pos.x < 0) {
      this.pos.x = width;
    }
    if (this.pos.x > width) {
      this.pos.x = 0;
    }
  }

  draw() {
    push();
    translate(this.pos.x, this.pos.y);
    rotate(-this.angle + PI/2);

    fill(200);
    strokeWeight(1);
    stroke(0);
    rect(-this.width/2, -this.height/2, this.width, this.height);
    stroke(255, 0, 0);
    strokeWeight(1);
    line(0, 0, 0, -this.height/2);
    stroke(0);

    if (this.boosting){
      var w = this.width/3;
      fill(200, 100, 100);
      push();
      translate(0, this.height/2);
      triangle(w, 0, -w, 0, 0, this.height/4);
      pop();
      if (this.vel.mag() >= (MAX_VEL - 0.01)) {
        stroke(255);
        strokeWeight(2);
        var yend = -this.height / 2 - 10;
        var xbegin = -this.width/2-10;
        line(xbegin, 0, 0, yend);
        line(-xbegin, 0, 0, yend);
      }
    }

    // draw ground truth point
    fill(0);
    strokeWeight(5);
    stroke(0);
    point(0, 0);

    pop();
  }

  debugDraw() {
    var str = "angularVel " + trunc(this.angularVel) + " angle " + trunc(this.orientation());
    str += " vel x " + trunc(this.vel.x) + " y " + trunc(this.vel.y);
    fill(255, 0, 0);
    textSize(22);
    text(str, 12, 320);
  }

  position() {
    return this.pos;
  }

  velocity() {
    return this.vel;
  }

  angularVelocity(){
    return this.angularVel;
  }

  orientation() {
    return this.angle;
  }
}

function diamondPath() {
  let path = new GoalPath();
  path.addWaypoint(width / 2, 200);
  path.addWaypoint(200, height / 2);
  path.addWaypoint(width / 2, height - 200);
  path.addWaypoint(width - 200, height / 2);
  return path;
}

class PIDController {
  constructor(setpoint, Kp, Ki, Kd) {
    this.setpoint = setpoint;
    this.Kp = Kp;
    this.Ki = Ki;
    this.Kd = Kd;
    this.integral = 0;
    this.previousError = 1;
  }

  updateError(err) {
    var proportional = err * this.Kp;
    var integral = this.integral * this.Ki;
    var derivative = (err - this.previousError) * this.Kd;
    this.previousError = err;
    this.integral += err;
    return proportional + derivative + integral;
  }

  update(v) {
    return this.updateError(this.setpoint - v);
  }

  set(s) {
    this.setpoint = s;
  }
}



var xboostContrib = 0.4;
var boostKp = 0.2;

// var maxD = 150;
// var maxD = 200;
// var maxD = 250;

// var facPerc = 0.3;
// var facPerc = 0.4;
// var facPerc = 0.2;

// var xboostContrib = 0.3;
// var xboostContrib = 0.4;
// var xboostContrib = 0.5;

// var boostKp = 0.15;
// var boostKp = 0.2;
// var boostKp = 0.25;

class AerialBot {
  constructor(rocket) {
    this.rocket = rocket;
    this.orientPID = new PIDController(PI / 2, 2, 0, 50);
    this.boostPID = new PIDController(0, boostKp, 0, 5);
    this.xposPID = new PIDController(width / 2, 0.003, 0, 0.1);
    this.target = c(width/2, height/2);
  }

  update(controller){
    if (controller.holdBoost == false) {
      let err = this.target.y - rocket.position().y;
      err -= abs(this.target.x - rocket.position().x) * xboostContrib;
      const boostOut = -this.boostPID.updateError(err);
      controller.holdBoost = boostOut > 0;
    }
    if (controller.yaw == 0) {
      const orientOut = this.orientPID.update(rocket.orientation());
      controller.yaw = clamp(orientOut, -1, 1);

      // tilt off from 90deg.
      let angleTilt = -this.xposPID.update(rocket.position().x);
      const MAX_TILT = radians(55);
      angleTilt = clamp(angleTilt, -MAX_TILT, MAX_TILT);
      this.orientPID.set(angleTilt + PI / 2);
    }
  }

  getTarget() {
    return this.target.copy();
  }

  setTarget(pos) {
    this.target = pos;
    this.xposPID.set(pos.x);
    this.boostPID.set(pos.y);
  }

  thetaErr() {
    return this.orientPID.setpoint - this.rocket.orientation();
  }
}

class SizedArray {
  constructor(N, val) {
    this.samples = [];
    this.N = N;
  }

  push(e) {
    this.samples.push(e);
    if (this.samples.length > this.N) {
      this.samples.shift();
    }
  }

  ref() {
    return this.samples;
  }
}

class GoalPath {
  constructor() {
    this.waypoints = [];
    this.activeIdx = 0;
    this.distToPass = 80;
    this.observers = [];
  }

  addObserver(o) {
    this.observers.push(o);
  }

  addWaypoint(x, y) {
    this.waypoints.push(createVector(x, y));
  }

  getCurrentWaypoint() {
    return this.waypoints[this.activeIdx];
  }

  getNextN(N) {
    var ret = [];
    let idx = this.activeIdx;
    for (let i = 0; i < N; i++) {
      ret.push(this.waypoints[idx]);
      idx++;
      idx %= this.waypoints.length;
    }
    return ret;
  }

  draw() {
    for (let i = 0; i < this.waypoints.length; i++) {
      let wpnt = this.waypoints[i];
      noStroke();
      fill(200, 0, 0);
      if (i == this.activeIdx) {
        fill(0, 150, 0);
      }
      ellipse(wpnt.x, wpnt.y, this.distToPass * 2, this.distToPass * 2);
    }
  }

  update(pos) {
    let curTarget = this.getCurrentWaypoint();
    if (Vec.dist(curTarget, pos) < this.distToPass) {
      this.activeIdx++;
      this.activeIdx %= this.waypoints.length;
      for (let o of this.observers) {
        o.onWaypointHit();
      }
    }
  }
}

class LapTimeDisplay {
  constructor(path, rocket, controller) {
    path.addObserver(this);
    this.path = path;
    this.rocket = rocket;
    this.controller = controller;
    this.curLapTime = 0;
    this.lapTimes = [];
    this.curPath = [];
    this.ghostPaths = [];
  }
  onWaypointHit() {
    if (this.path.activeIdx == 1 && this.curLapTime != 0){
      this.lapTimes.push(this.curLapTime);
      this.curLapTime = 0;
      if (this.lapTimes.length > 5) {
        this.lapTimes.shift();
      }
      this.ghostPaths.push(this.curPath);
      if (this.ghostPaths.length > 3) {
        this.ghostPaths.shift();
      }
      this.curPath = [];
    }
  }
  update() {
    this.curLapTime += deltaTime;
    this.curPath.push({ 
      pos: this.rocket.position().copy(),
      boost: this.controller.holdBoost,
      angle: this.rocket.orientation(),
    });
  }
  draw() {
    textSize(40);
    textAlign(CENTER);
    noStroke();
    fill(0);
    text(trunc(this.curLapTime / 1000, 10), width/2, 40);
    var y = 30;
    const spacing = 30;
    textSize(30);
    textAlign(RIGHT);
    for (let i = 0; i < this.lapTimes.length; i++) {
      let idx = this.lapTimes.length - i - 1;
      text(trunc(this.lapTimes[idx] / 1000, 10), width - 10, y);
      y += spacing;
    }

    noFill();
    colorMode(HSB, 255);
    let hue = 0;
    for (let pth of [...this.ghostPaths, this.curPath]) {
      for (let i = 0; i < pth.length - 1; i++) {
        let curPos = pth[i].pos;
        let nextPos = pth[i + 1].pos;
        stroke(hue, 100, 150);
        stroke(hue, 100, pth[i].boost * 100 + 150);
        strokeWeight(2);
        line(curPos.x, curPos.y, nextPos.x, nextPos.y);
        let dir = Vec.fromAngle(-pth[i].angle).mult(10);
        strokeWeight(1);
        line(curPos.x, curPos.y, curPos.x + dir.x, curPos.y + dir.y);
      }
      hue += 80;
      hue %= 255;
    }
    colorMode(RGB, 255);
  }
}

// 6.3-6.7s
class NaivePositionPlanner {
  constructor(path, rocket, bot) {
    this.path = path;
    this.rocket = rocket;
    this.bot = bot;
  }
  update() {
    let curWay = this.path.getCurrentWaypoint();
    this.bot.setTarget(c(curWay.x, curWay.y));
  }
}

//5.2-4.2s
// 3.8 fastest.
class LinePositionPlanner {
  constructor(path, rocket, bot) {
    path.addObserver(this);
    this.path = path;
    this.rocket = rocket;
    this.bot = bot;
    this.begin = c();
    this.end = c();
    this.fac = 0;
  }
  onWaypointHit() {
    // this.begin = this.end.copy();
    // this.end = this.path.getCurrentWaypoint();
    // this.fac = 0;
  }
  update() {
    // frameRate(15);
    var maxD = 200;
    var facPerc = 0.6;
    
    let pnts = this.path.getNextN(2);
    let d = Vec.dist(this.rocket.position(), pnts[0]);
    d = clamp(d, 0, maxD);
    d = maxD - d;
    d /= maxD;
    
    let nextDir = Vec.sub(pnts[1], pnts[0]);
    nextDir.normalize();
    
    let vel = this.rocket.velocity().copy().normalize();
    let idealDir = Vec.sub(pnts[0], rocket.position()).normalize();
    let dot = Vec.dot(vel, nextDir);
    dot = (dot + 1) / 2;
    // dot = smoothstep(0.8, 1, dot);
    // dot = Math.pow(dot, 2);
    dot = dot * dot;
    // dot *= 2;
    
    vel.setMag(50);
    nextDir.setMag(50);
    idealDir.setMag(50);
    // noFill();
    // ellipse(this.rocket.position().x, this.rocket.position().y, maxD * 2, maxD * 2);
    stroke(0, 255, 0);
    line(100, 100, 100 + vel.x, 100 + vel.y);
    stroke(255, 255, 0);
    line(100, 100, 100 + idealDir.x, 100 + idealDir.y);
    stroke(0, 0, 255);
    line(100, 100, 100 + nextDir.x, 100 + nextDir.y);
    
    let blended = vecMix(pnts[0], pnts[1], d * facPerc * dot);
    this.bot.setTarget(blended);
  }
}

