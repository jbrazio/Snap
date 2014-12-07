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

RigidBody.prototype.init = function (morph, mass, restitution) {
    this.initialCenter = morph.center();
    this.mass = mass
    this.morph = morph;
    this.velocity = new Point(2,0);
    this.restitution = restitution;
}

RigidBody.prototype.position = function() {
    return this.morph.center();
}

RigidBody.prototype.setPosition = function(point) {
    this.morph.setCenter(point);
}

RigidBody.prototype.reset = function() {
    this.morph.setCenter(this.initialCenter);
    this.velocity = new Point(0,0);
}


// RigidBodySolver  ///////////////////////////////////////////////////

function RigidBodySolver(timescale) {
    this.init();
}

RigidBodySolver.prototype.init = function(point, timescale) {
    this.airDensity = 1.2041; //http://en.wikipedia.org/wiki/Density_of_air
    this.gravity = point && point instanceof Point ? point : new Point(0, -9.81);
    this.isRunning = true;
    this.time = Date.now();
    this.timescale = timescale ? timescale : 0.0001;
}

RigidBodySolver.prototype.start = function() {
    this.init(this.gravity);
}

RigidBodySolver.prototype.stop = function() {
    this.isRunning = false;
}

RigidBodySolver.prototype.dragForce = function(velocity, Cd, density, area) {
    //  D = Cd * 0.5 * r * V^2 * A
    var d = v.multiplyBy(v).scaleBy(Cd * 0.5 * density * area).neg();
    return d;
}

RigidBodySolver.prototype.simulate = function(bodies) {
    // update elapsed time
    var dt = (Date.now() - this.time) * this.timescale;
    this.time += dt;
    var solver = this;
    bodies.forEach(function(body) {

        /*
        Drag Force calculation based on:

        http://www.grc.nasa.gov/WWW/k-12/airplane/drageq.html
        http://www.grc.nasa.gov/WWW/k-12/airplane/falling.html
        http://youtu.be/-ISmOwjCvoE

        GLOSSARY:
          m = object mass
          g = gravity
          w = object weight (m * g)
          d = drag
          Cd = drag coefficient (1.05 for a box - http://youtu.be/-ISmOwjCvoE)
          r = air density (1.2041 - http://en.wikipedia.org/wiki/Density_of_air)
          v = velocity
          A = area
          F = net external Force

          1)  w = m * g
          2)  D = Cd * 0.5 * r * V^2 * A
          3)  F = W - D
          4)  F = m * a
          5)  a = F / m
          6)  a = (W - D) / m;

        */


        // compute drag force
        var m  = body.mass;
        var v_ = body.velocity;
        var g_ = solver.gravity;
        var w_ = g_.scaleBy(m);
        var d_ = solver.dragForce(g_, 1.05, this.airDensity, 0.1);
        var a_ = w_.subtract(d_).divideBy(m);
        var v_ = v_.add(a_.scaleBy(dt));
        var p_ = body.position();

        console.log("v:" + v_ + " d:" + d_);

        body.setPosition(p_.add(v_.scaleBy(dt*dt).flipY()));
        body.velocity = v_;

//        console.log("p:" + p + " g:" + g_ + " d:" + d_ + " a:" + a_ + " v:" + v_);
    });
}


// CollisionSolver  ///////////////////////////////////////////////////

function CollisionSolver() {

}

CollisionSolver.prototype.findCollisions = function(bodies) {

}