/*

*/

// Global stuff ///////////////////////////////////////////////////////

var RigidBody;
var RigidBodySolver;
var CollisionSolver;


// Point Hack /////////////////////////////////////////////////////////

// I extend the Point class already in Snap!

function Vector2d(x,y) {
    return new Point(x, y);
}

Point.prototype.magnitude = function() {
    return Math.sqrt(this.x * this.x + this.y * this.y);
}

Point.prototype.normalize = function() {
    return this.divideBy(this.magnitude());
}

Point.prototype.flipY = function() {
    return new Point(this.x, -this.y);
}

Point.prototype.flipX = function() {
    return new Point(-this.x, this.y);
}

Point.prototype.plus = function(point) {
    this.x += point.x;
    this.y += point.y;
    return this;
}

Point.prototype.minus = function(point) {
    this.x -= point.x;
    this.y -= point.y;
    return this;
}






function RigidBody (morph, mass, restitution) {

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
    //     this.e;   scalar   Restitution coefficient (friction / bouncyness)
    // ------------------------------------------------------------------------
    // State:
    //     this.p          Point    Position of the body
    //     this.springs    Array    List of spring points
    // ------------------------------------------------------------------------
    // Derived:
    //     this.v    Point    Velocity of the body
    // ------------------------------------------------------------------------
    // Computed:
    //     this.f    Point    Force applied on the body
    //     this.a    Point    Acceleration computed by as a = F / m (F = m x a)
    // ------------------------------------------------------------------------
    this.m = Math.max(0, mass ? mass : 0);
    this.e = Math.max(0, restitution ? restitution : 0);
    this.springs = new Array();
    this.f = new Vector2d(0, 0);
    this.v = new Vector2d(0, 0);
    this.a = new Vector2d(0, 0);

    // Setup event handler for morph drag & drop
    this.setupDropHandler();
}

// For ease of use define the object property 'x' so that one can assign
// a position directly like other member properties, while delegating
// to the morph's center() && setCenter() methods
Object.defineProperty(RigidBody.prototype, 'p', {
    get: function() { return this.morph.center() },
    set: function(point) { this.morph.setCenter(point); },
    enumerable: true,
    configurable: true
});

RigidBody.prototype.removeSpring = function(body) {
    for (var i = 0; i < this.springs.length; i++) {
        var spring = this.springs[i];
        if (spring && spring.obj == body) {
            this.springs.splice(i,1);
        }
    }
}

RigidBody.prototype.addSpring = function(other, stiffness, length) {
    this.removeSpring(other);
    this.springs.push({
        obj: other,
        stiffness: stiffness,
        length: length
    });
}

RigidBody.prototype.springForce = function(p, stiffness, length) {
    var springForce = new Vector2d(0, 0),
        distance = this.p.subtract(p);
    distance = distance.normalize().scaleBy(distance.magnitude() - length);
    springForce = distance.scaleBy(-1 * stiffness);
    springForce.minus(this.v.scaleBy(0.1));
    return springForce;
}

RigidBody.prototype.springsNetForce = function() {
    var springsNetForce = new Vector2d(0, 0);
    for (var i = 0; i < this.springs.length; i++) {
        var spring = this.springs[i],
            springForce = this.springForce(spring.obj.p, spring.stiffness, spring.length);
        springsNetForce.plus(springForce);
    }
    return springsNetForce;
}

RigidBody.prototype.setupDropHandler = function() {
    var rb = this;
    var morph = this.morph;
    if (!(morph.rigidBody instanceof RigidBody)) {
        var dropped = morph.justDropped;
        morph.justDropped = function() {
            if (morph.rigidBody instanceof RigidBody) {
                morph.rigidBody.justDropped();
            }
            dropped.call(morph);
        }
    }
}

RigidBody.prototype.justDropped = function() {
    this.reset();
}

RigidBody.prototype.reset = function() {
    this.f = new Vector2d(0, 0);
    this.v = new Vector2d(0, 0);
    this.a = new Vector2d(0, 0);
}

RigidBody.prototype.updatePositions = function(dt) {
    this.p = this.p.add(this.v.scaleBy(dt)).add(this.a.scaleBy(0.5*dt*dt));
}

RigidBody.prototype.update = function(dt) {

    // var a = this.f.divideBy(this.m);
    // this.v.plus(a.scaleBy(dt));
    // this.p = this.p.add(this.v.scaleBy(dt));
    var new_a = this.f.divideBy(this.m);
    var avg_a = this.a.add(new_a).scaleBy(0.5);
    this.v.plus(avg_a.scaleBy(dt));
    this.a = new_a;
}

RigidBody.prototype.overlapsBoundingBox = function(other) {
    return this.p.intersects(other.p);
}

RigidBody.prototype.overlapsPixelPerfect = function(other) {

    var pos = this.morph.isTouching(other.morph);
    return pos;
}






// RigidBodySolver  ///////////////////////////////////////////////////

function RigidBodySolver() {
    //            INTERNAL DATA
    // ------------------------------------------------------------------------
    // Constants:
    //     morph        StageMorph    Snap! StageMorph object
    //     isRunning    Boolean       Flag if simulation is running
    // ------------------------------------------------------------------------
    // State:
    //     lastTime         Long          Last time step() was run
    //     timeLeftOver     Long          Time not used in computation
    // ------------------------------------------------------------------------
    this.morph = null;
    this.isRunning = true;
    this.lastTime = Date.now();
    this.timeLeftOver = 0;

    //            PHYSICS DATA
    // ------------------------------------------------------------------------
    // Constants=
    //     g                Point     Gravity direction and strength vector
    //     stepSize         scalar    Size of the computation step in ms
    //     stepSizeScale    scalar    Scale for the stepsize (basically screen scale)
    // ------------------------------------------------------------------------
    this.g = new Vector2d(0, 9.8);
    this.stepSize = 4; // 1s / 0.016s = 62,5 fps
    this.stepSizeScale = 0.01;
}

RigidBodySolver.prototype.create = function() {
    var obj = Object.create(this);
    return obj;
}

RigidBodySolver.prototype.start= function() {
    this.isRunning = true;
    this.time = Date.now();
}

RigidBodySolver.prototype.stop = function() {
    this.isRunning = false;
}

RigidBodySolver.prototype.weightForce = function(body) {

    // Weight Force ( Fw = m * g)
    var Fw = this.g.scaleBy(body.m);
    return Fw;
}

RigidBodySolver.prototype.airDragForce = function(body) {

    // Air Drag Force ( FD = 1/2 * rho * v² * Cd * A)
    var Cd = 0.47;
    var A = body.morph.width() * 0.001;
    var rho = 1.2041;
    var v = body.v.magnitude();
    var vv = v*v;
    var Fd = body.v.normalize().scaleBy(-1 * 0.5 * Cd * rho * vv * Cd * A);
    return Fd;
}

RigidBodySolver.prototype.applyForces = function(bodies, dt) {

    var solver = this;
    bodies.forEach(function (body) {
        if (body instanceof RigidBody && body.m > 0) {
            body.f = new Vector2d(0, 0);
            body.f.plus(solver.weightForce(body));
            body.f.plus(solver.airDragForce(body));
            body.f.plus(body.springsNetForce());
        }
    });
}

RigidBodySolver.prototype.handleCollisions = function(collisions) {

    var solver = this;

    collisions.forEach(function(collision) {
        var bodyA = collision.a;
        var bodyB = collision.b;

        if (bodyA && bodyA.m > 0) {
            bodyA.v.scaleBy(-0.5);
            bodyA.v = new Vector2d(0, 0);
        }

        if (bodyB && bodyB.m > 0) {
            bodyB.v.scaleBy(-0.5);
            bodyB.v = new Vector2d(0, 0);
        }
    });
}

RigidBodySolver.prototype.detectCollisionsPhase1 = function(bodies) {

    var collisions = [];
    for (var i = 0; i < bodies.length; i++) {
        var bodyA = bodies[i];
        for (var j = i+0; j < bodies.length; j++) {
            var bodyB = bodies[j];
            if (i != j && (bodyA.m > 0 || bodyB.m > 0)) {
                if (bodyA.overlapsBoundingBox(bodyB)) {
                    if (bodyA.overlapsPixelPerfect(bodyB)) {
                        collisions.push({ a: bodyA, b: bodyB });
                    }
                }
            }
        }
    }
    return collisions;
}

RigidBodySolver.prototype.solveCollisions = function(collisions) {

}

RigidBodySolver.prototype.step = function(bodies) {

    var now = Date.now(),
        elapsedTime = (now - this.lastTime) + this.timeLeftOver,
        timesteps = Math.floor(elapsedTime / this.stepSize),
        dt = this.stepSize * this.stepSizeScale;
    this.lastTime = now;
    this.timeLeftOver = elapsedTime - timesteps * this.stepSize;

    for (var i = 0; i < timesteps; i++) {

        // Velocity Verlet Start - Update Positions
        bodies.forEach(function (body) {
            if (body instanceof RigidBody && body.m > 0) {
                body.updatePositions(dt);
            }
        })

        this.applyForces(bodies, dt);

        //var collisions = [];
        //collisions = this.detectCollisions(bodies);
        //this.solveCollisions(collisions);

        // Velocity Verlet Conclusion - Compute average acceleration & velocity
        bodies.forEach(function (body) {
            if (body instanceof RigidBody && body.m > 0) {
                body.update(dt);
            }
        })
        //this.updateBodies(bodies, dt);
    }
}
