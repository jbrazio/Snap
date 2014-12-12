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
