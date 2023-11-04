const GRAVITY = 0.175;
// const GRAVITY = 0;
const BOOST_ACCEL = 0.4;
const YAW_ACCEL = 0.002;
const ANG_VEL_DAMPING = 0.90;
const MAX_ANG_VEL = 0.125;
const MAX_VEL = 10;
const SPACE = 32; // key code
const Vec = p5.Vector;

let rocket;
let bot;
let path;
let botTarget;
let controller;
let chart;

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
function trunc(n) {
  return int(n * 1000) / 1000.0;
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

function setup() {
  frameRate(60);
  createCanvas(1000, 1000);
  globalThis.context = this;
  rocket = new Rocket(width/2 - 200, height/2);
  controller = new ControllerInput();
  botTarget = createVector(width / 2, height / 2);
  bot = new AerialBot(rocket);
  path = new GoalPath();
  path.addWaypoint(width / 2, 200);
  path.addWaypoint(200, height / 2);
  path.addWaypoint(width / 2, height - 200);
  path.addWaypoint(width - 200, height / 2);

  thetaErrSamples = new SizedArray(60 * 2, 0);
  derivativeErrSamples = new SizedArray(60 * 2, 0);

  // chart = new CanvasJS.Chart("graph", {
  //   interactivityEnabled: false,
  //   axisX: {
  //     title: "Time (ms)",
  //     gridThickness: 0,
  //     tickLength: 0,
  //     lineThickness: 0,
  //     labelFormatter: function(){
  //       return " ";
  //     }
  //   },
  //   axisY: {
  //     title: "Deviation from Upright (rad)",
  //     maximum: PI,
  //     minimum: -PI
  //   },
  //   data: [{
  //     type: "line",
  //     dataPoints: thetaErrSamples.ref()
  //   }]
  // });
  //

  var options = {
    chart: {
      type: 'line',
      height: 350,
      width: width,
      animations: {
        enabled: false,
        // easing: 'linear',
        // dynamicAnimation: {
        //   speed: 1000
        // }
      },
    },
    dataLabels: {
      enabled: false
    },
    stroke: {
      curve: 'smooth',
    },
    tooltip: {
      enabled: false,
    },
    legend: {
      show: false,
    },
    yaxis: {
      min: -PI,
      max: PI,
      labels: {
          show: false,
      },
    },
    xaxis: {
      labels: {
          show: false,
      }
    },
    series: [{
      name: 'theta',
      data: []
    }],
  }

  // chart = new ApexCharts(document.querySelector("#graph"), options);
  // chart.render();
}

function draw() {
  background(50);

  controller.setFromUser();
  bot.update(controller);
  rocket.updateWithInput(controller);

  // textSize(22);
  // fill(255, 0, 0);
  // controller.debugDraw();
  // rocket.debugDraw();

  const amp = 200;
  const phaseMult = 0.01;
  const phi = frameCount * phaseMult;
  let x = cos(phi) * amp;
  let y = sin(phi) * amp;
  botTarget.x = x + width / 2;
  botTarget.y = y + height / 2;
  bot.setTarget(botTarget);
  noFill();
  stroke(255, 0, 0, 150);
  ellipse(width/2, height/2, amp * 2, amp * 2);

  if (mouseIsPressed) {
    botTarget.x = mouseX;
    botTarget.y = mouseY;
  }

  // let curWay = path.getCurrentWaypoint();
  // botTarget.set(curWay.x, curWay.y);
  // path.draw();
  // path.update(rocket.position());

  rocket.draw();
  stroke(0, 255, 0);
  strokeWeight(7);
  point(botTarget.x, botTarget.y);
  strokeWeight(1);

  bot.setTarget(botTarget);
}

// function updateThetaCharts() {
//   thetaErrSamples.push(bot.thetaErr());
//   derivativeErrSamples.push(bot.derivativeThetaErr() * 30);
//   chart.updateSeries([{ data: thetaErrSamples.ref() }, { data: derivativeErrSamples.ref() }])
// }

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
  /// x, y will refer to the bottom middle of the rocket.
  constructor(x, y) {
    this.pos = createVector(x, y);
    this.vel = createVector(0, 0);
    this.accel = createVector(0, 0);
    this.angle = PI / 2;
    this.angularVel = 0;
    // this.angle = mapb(Math.random(), radians(30), radians(180-30));
    // this.angularVel = Math.random() * MAX_ANG_VEL * 0.1;
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
    }

    // draw ground truth point
    fill(0);
    strokeWeight(5);
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

class AerialBot {
  constructor(rocket) {
    this.rocket = rocket;
    this.orientPID = new PIDController(PI / 2, 2, 0, 50);
    this.boostPID = new PIDController(height / 2, 0.2, 0, 4);
    this.xposPID = new PIDController(width / 2, 0.003, 0, 0.2);
  }

  update(controller){
    if (controller.holdBoost == false) {
      const boostOut = -this.boostPID.update(rocket.position().y);
      controller.holdBoost = boostOut > 0;
    }
    if (controller.yaw == 0) {
      const orientOut = this.orientPID.update(rocket.orientation());
      controller.yaw = clamp(orientOut, -1, 1);

      // tilt off from 90deg.
      let angleTilt = -this.xposPID.update(rocket.position().x);
      const MAX_TILT = radians(35);
      angleTilt = clamp(angleTilt, -MAX_TILT, MAX_TILT);
      this.orientPID.set(angleTilt + PI / 2);
    }
  }

  setTarget(pos) {
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
    this.distToPass = 130;
  }

  addWaypoint(x, y) {
    this.waypoints.push(createVector(x, y));
  }

  getCurrentWaypoint() {
    return this.waypoints[this.activeIdx];
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
    }
  }
}