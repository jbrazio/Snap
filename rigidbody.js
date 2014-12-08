/*

*/

// Global stuff ///////////////////////////////////////////////////////

var RigidBody;
var RigidBodySolver;
var CollisionSolver;


// Point Hack /////////////////////////////////////////////////////////

// I extend the Point class already in Snap!

Point.prototype.length = function() {
    return Math.sqrt(this.x * this.x + this.y * this.y);
}

Point.prototype.normalize = function() {
    return this.divideBy(this.length());
}

Point.prototype.flipY = function() {
    return new Point(this.x, -this.y);
}

Point.prototype.flipX = function() {
    return new Point(-this.x, this.y);
}


// RigidBody  /////////////////////////////////////////////////////////

// I am attached to a SpriteMorph object and keep a reference to it
// When needed I read and write information directly onto the SpriteMorph

function RigidBody(morph, mass, restitution) {
    this.init(morph, mass, restitution);
}

RigidBody.prototype.init = function (morph, mass) {
    this.initialCenter = morph.center();
    this.mass = mass
    this.morph = morph;
    this.velocity = new Point(10,30);
}

RigidBody.prototype.position = function() {
    return this.morph.center();
}

RigidBody.prototype.setPosition = function(x, y) {
    //this.previousPosition = this.position();
    this.morph.setCenter(new Point(x,y));
}

RigidBody.prototype.setVelocity = function(x, y) {
    this.velocity.x = x;
    this.velocity.y = y;
}

RigidBody.prototype.reset = function() {
    this.morph.setCenter(this.initialCenter);
}


// RigidBodySolver  ///////////////////////////////////////////////////

function RigidBodySolver(timescale) {
    this.init();
}

RigidBodySolver.prototype.init = function(gravity, timescale) {
    this.airDensity = 1.2041; //http://en.wikipedia.org/wiki/Density_of_air
    this.gravity = gravity && gravity instanceof Point ? gravity : new Point(0, -9.81);
    this.isRunning = true;
    this.dt = 0.03;
}

RigidBodySolver.prototype.start = function() {
    this.init(this.gravity);
}

RigidBodySolver.prototype.stop = function() {
    this.isRunning = false;
}

RigidBodySolver.prototype.Verlet = function(bodies) {
    var solver = this;
    bodies.forEach(function(body) {
        var p = body.position();
        var v = body.velocity;
        var a = solver.gravity;
        var dt = solver.dt;

        // velocity verlet
        var posx = p.x + v.x*dt + 0.5*a.x*dt*dt;
        var posy = p.y - v.y*dt + 0.5*a.y*dt*dt;

        var vx = v.x + a.x*dt;
        var vy = v.y + a.y*dt;

        body.setPosition(posx, posy);
        body.setVelocity(vx, vy);

        //console.log("p:" + p + " v:" + v);
    });
}

RigidBodySolver.prototype.SatisfyConstraints = function(bodies) {
}

RigidBodySolver.prototype.AccumulateForces = function(bodies) {
}

RigidBodySolver.prototype.Timestep = function(bodies) {
    this.Verlet(bodies);
    this.AccumulateForces(bodies);
    this.SatisfyConstraints(bodies);
}


// CollisionSolver  ///////////////////////////////////////////////////

function CollisionSolver() {

}

CollisionSolver.prototype.findCollisions = function(bodies) {

}