const GRAVITY = 0.25;
// const GRAVITY = 0;
const BOOST_ACCEL = 0.4;
const YAW_ACCEL = 0.002;
const ANG_VEL_DAMPING = 0.90;
const MAX_ANG_VEL = 0.125;
const MAX_VEL = 10;
const VEL_DAMPEN = 0.9;
const MAX_ACCEL = .1;
const SPACE = 32; // key code
const Vec = p5.Vector;
const THRES = 5;

let pos;
let vel;
// spring constant.
const k = .02;
const dampingCoef = Math.sqrt(k);
let goal;
let startPos;

function setup() {
  createCanvas(1000, 1000);
  pos = createVector(width/2 - 100, height/2);
  startPos = pos.copy();
  vel = createVector(0, 0);
  goal = createVector(width/2, height/2);
  globalThis.context = this;
  globalThis.context = this;
}

function vClamp(v, L) {
  if (v.mag() > L) v.setMag(L);
}

function trunc(n) {
  return int(n * 1000) / 1000.0;
}

function draw() {
  background(50);

  // let displacement = Vec.sub(pos, goal);
  // let F = Vec.mult(displacement, -k);
  // let dampF = Vec.mult(vel, -dampingCoef);
  // let a = Vec.add(F, dampF);

  let displacement = Vec.sub(pos, goal);
  // initially, set acceleration max towards the target.
  let a = displacement.copy().mult(-1);
  a.setMag(MAX_ACCEL);

  const startDisp = startPos.x - goal.x;
  const v = vel.x;
  const p = -displacement.x;
  let a0 = MAX_ACCEL;
  // let A = 0.5 * a0 * Math.pow(v, 2);

  if (abs(p) < THRES) {
    // a = displacement.copy().mult(-.01);
    // a = vel.copy().mult(-10);
    a.mult(0);
    background(255, 0, 255);
  } else {
    if (abs(p) < abs(startDisp) / 2) {
      a.mult(-1);
      background(255, 255, 0);
    }
  }

  if (a.mag() < 0.001) {
    vel.mult(VEL_DAMPEN);
  }

  vClamp(a, MAX_ACCEL);
  vClamp(vel, MAX_VEL);
  pos.add(vel);
  vel.add(a);

  fill(255);
  ellipse(pos.x, pos.y, 20, 20);
  fill(255, 0, 0);
  // text("A" + trunc(A), 12, 200);
  text("P" + trunc(p), 12, 300);

  if (mouseIsPressed) {
    pos.x = mouseX;
    // pos.y = mouseY;
    startPos = pos.copy();
    vel.x = 0;
    // vel.y = 0;
  }

  textSize(20);
  fill(255, 0, 0);
  text("a x" + trunc(a.x) + "a y" + trunc(a.y), 12, 100);
  ellipse(width/2, height/2, 5, 5);

  a.setMag(20);
  stroke(255, 0, 0);
  strokeWeight(5);
  line(width/2, height/2 - 200, width/2 + a.x, height/2 - 200 + a.y);
  strokeWeight(1);
  stroke(0);
}

