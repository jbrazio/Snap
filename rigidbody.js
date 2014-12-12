<<<<<<< Updated upstream
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
=======
/*

*/

// Global stuff ///////////////////////////////////////////////////////

var RigidBody;
var RigidBodyWorld;
var RigidBodySolver;
var CollisionSolver;


// Point Hack /////////////////////////////////////////////////////////

// I extend the Point class already in Snap!

function Vector2d(x,y) {
    return new Point(x, y);
}

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

Point.prototype.accum = function(point) {
    this.x += point.x;
    this.y += point.y;
}

function RigidBody (morph, mass) {

    //            INTERNAL DATA
    // ------------------------------------------------------------------------
    // Constants:
    //     this.morph          SpriteMorph    Snap! SpriteMorph object
    //     this.startX         Point          Associated morphs' start location
    // ------------------------------------------------------------------------
    this.morph = morph;
    this.startX = morph.center();


    //            PHYSICS DATA
    // ------------------------------------------------------------------------
    // Constants:
    //     this.m;   scalar   Mass of the body
    // ------------------------------------------------------------------------
    // State:
    //     this.x    Point    Position of the body
    //     this.P    Point    Linear momentum of the body = m * v
    // ------------------------------------------------------------------------
    // Derived:
    //     this.v    Point    Velocity of the body
    // ------------------------------------------------------------------------
    // Computed:
    //     this.f    Point    Force applied on the body
    // ------------------------------------------------------------------------
    this.m = Math.max(0, mass ? mass : 1);
    this.P = new Vector2d(0, 0)
    this.f = new Vector2d(0, 0);
    this.v = new Vector2d(0, 0);
}

// For ease of use define the object property 'x' so that one can assign
// a position directly like other member properties, while delegating
// to the morph's center() && setCenter() methods
Object.defineProperty(RigidBody.prototype, 'x', {
    get: function() { return this.morph.center() },
    set: function(point) { this.morph.setCenter(point); },
    enumerable: true,
    configurable: true
});

RigidBody.prototype.reset = function() {
    this.P = new Vector2d(0, 0);
    this.f = new Vector2d(0, 0);
    this.v = new Vector2d(0, 0);
    this.x = this.startX;
}

RigidBody.prototype.integrate = function(dt) {

    // Euler's Integration
    this.x = this.x.add(this.v.scaleBy(dt).flipY());            // x = x + v*dt
    this.P.accum(this.f.scaleBy(dt));                   // P = P + f*dt
    var kdf = this.v.scaleBy(RigidBodySolver.kdl * dt); // kdf = v * kdl*dt
    this.P.accum(kdf.neg());                           // P = P + (-kdf)

    // Aux computation
    this.v = this.P.divideBy(this.m);  // v = P / m   (as  P = m * v)
}

// RigidBodySolver  ///////////////////////////////////////////////////

function RigidBodySolver(morph) {

    //            INTERNAL DATA
    // ------------------------------------------------------------------------
    // Constants:
    //     this.morph        StageMorph    Snap! StageMorph object
    //     this.isRunning    Boolean       Flag if simulation is running
    // ------------------------------------------------------------------------
    // State:
    //     this.time         Long          Last time step() was run
    // ------------------------------------------------------------------------
    this.morph = morph;
    this.isRunning = true;
    this.time = Date.now();

    //            PHYSICS DATA
    // ------------------------------------------------------------------------
    // Constants:
    //     this.g           Point     Gravity direction and strength vector
    //     this.numSteps    scalar    Number of computation iterations per dt
    //     this.kdl         scalar    Linear Damping (Linear Momentum reduction)
    // ------------------------------------------------------------------------
    this.g = new Vector2d(0, -9.8);
    this.numSteps = 10;
    this.kdl = 0.1;
}

RigidBodySolver.prototype.start = function() {
    this.isRunning = true;
    this.time = Date.now();
}

RigidBodySolver.prototype.stop = function() {
    this.isRunning = false;
}

RigidBodySolver.prototype.step = function(bodies, time) {

    var time = Date.now();
    var dt = (time - this.time) / 300;
    this.time = time;
    var dt2 = dt / this.numSteps;
    var solver = this;

console.log(dt2);

    for (var j = 0; j < this.numSteps; j++) {

        bodies.forEach(function (body) {
        
            // Accumulate gravity on bodies
            body.f.accum(solver.g.scaleBy(body.m));  // f = m * g

            // Compute body stuff
            if (body.m >= 0) {
                body.integrate(dt2);
            }
        });
    }
}
>>>>>>> Stashed changes
