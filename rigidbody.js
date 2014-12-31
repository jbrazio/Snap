/*

*/

// Global stuff ///////////////////////////////////////////////////////

var RigidBody;
var RigidBodySolver;
var CollisionSolver;


var RB_DEBUG = false;

// Point Hack /////////////////////////////////////////////////////////

// I extend the Point class already in Snap!

function Vector2d(x,y) {
    return new Point(x, y);
}

Point.prototype.magnitude = function () {
    return Math.sqrt(this.x * this.x + this.y * this.y);
}

Point.prototype.normalize = function () {
    return this.divideBy(this.magnitude());
}

Point.prototype.flipY = function () {
    return new Point(this.x, -this.y);
}

Point.prototype.flipX = function () {
    return new Point(-this.x, this.y);
}

Point.prototype.plus = function (point) {
    this.x += point.x;
    this.y += point.y;
    return this;
}

Point.prototype.minus = function (point) {
    this.x -= point.x;
    this.y -= point.y;
    return this;
}

Point.prototype.swap = function () {
    return new Point (-1 * this.y, this.x);
}

Point.prototype.swap2 = function () {
    return new Point (this.y, -1 * this.x);
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
    get: function () { return this.morph.center() },
    set: function (point) { this.morph.setCenter(point); },
    enumerable: true,
    configurable: true
});

RigidBody.prototype.removeSpring = function (body) {
    for (var i = 0; i < this.springs.length; i++) {
        var spring = this.springs[i];
        if (spring && spring.obj == body) {
            this.springs.splice(i,1);
        }
    }
}

RigidBody.prototype.addSpring = function (other, stiffness, length) {
    this.removeSpring(other);
    this.springs.push({
        obj: other,
        stiffness: stiffness,
        length: length
    });
}

RigidBody.prototype.springForce = function (p, stiffness, length) {
    var springForce = new Vector2d(0, 0),
        distance = this.p.subtract(p);
    distance = distance.normalize().scaleBy(distance.magnitude() - length);
    springForce = distance.scaleBy(-1 * stiffness);
    springForce.minus(this.v.scaleBy(0.1));
    return springForce;
}

RigidBody.prototype.springsNetForce = function () {
    var springsNetForce = new Vector2d(0, 0);
    for (var i = 0; i < this.springs.length; i++) {
        var spring = this.springs[i],
            springForce = this.springForce(spring.obj.p, spring.stiffness, spring.length);
        springsNetForce.plus(springForce);
    }
    return springsNetForce;
}

RigidBody.prototype.setupDropHandler = function () {
    var rb = this;
    var morph = this.morph;
    if (!(morph.rigidBody instanceof RigidBody)) {
        var dropped = morph.justDropped;
        morph.justDropped = function () {
            if (morph.rigidBody instanceof RigidBody) {
                morph.rigidBody.justDropped();
            }
            dropped.call(morph);
        }
    }
}

RigidBody.prototype.justDropped = function () {
    this.reset();
}

RigidBody.prototype.reset = function () {
    this.f = new Vector2d(0, 0);
    this.v = new Vector2d(0, 0);
    this.a = new Vector2d(0, 0);
    this.springs = [];
}

RigidBody.prototype.updatePositions = function (dt) {
    this.p = this.p.add(this.v.scaleBy(dt));//.add(this.a.scaleBy(0.5*dt*dt));
}

RigidBody.prototype.update = function (dt) {

    // var a = this.f.divideBy(this.m);
    // this.v.plus(a.scaleBy(dt));
    // this.p = this.p.add(this.v.scaleBy(dt));
    var new_a = this.f.divideBy(this.m);
    var avg_a = this.a.add(new_a).scaleBy(0.5);
    this.v.plus(avg_a.scaleBy(dt));
    this.a = avg_a;
}

RigidBody.prototype.overlapsBoundingBox = function (other) {
    return this.morph.bounds.intersects(other.morph.bounds);
}

RigidBody.prototype.collisionData = function (other) {
    return this._collisionData(other);
}

RigidBody.prototype._collisionData = function (other) {
    var oRect = this.morph.bounds.intersect(other.morph.bounds),
        oImg = this.morph.overlappingImage(other.morph),
        data = oImg.getContext('2d')
                   .getImageData(1, 1, oImg.width, oImg.height)
                   .data,
        p = undefined;

    // compute contact points within each sprite
    for (var i = 3; i < data.length; i+=4) {
        if (data[i] !== 0) {
            var pos = Math.floor(i/4),
                y = Math.floor(pos/oImg.width),
                x = pos - y * oImg.width;
                p = new Vector2d(oRect.left()+x,
                                 oRect.top()+y);
            break;
        }
    }

    if (isNil(p)) {
        return null;
    }

    function findClosestCorner(morph, p) {
        var corners = [morph.topLeft(), morph.topRight(), morph.bottomLeft(), morph.bottomRight()],
            n = undefined;
        corners.forEach(function (corner) {
            if (isNil(n) || corner.subtract(p).magnitude < n.magnitude()) {
                n = corner.subtract(p);
            }
        });
        var n1 = n.swap(), n2 = n.swap2();
        n = morph.center().subtract(n1).magnitude() < morph.center().subtract(n2).magnitude() ? n1.normalize() : n2.normalize();
        return n;
    }

    an = findClosestCorner(this.morph, p);
    bn = findClosestCorner(other.morph, p);

    return {
        a: this,
        b: other,
        p: p,
        an: an,
        bn: bn
    }
}






// RigidBodySolver  ///////////////////////////////////////////////////

function RigidBodySolver(morph) {
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
    this.morph = morph;
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

RigidBodySolver.prototype.create = function () {
    var obj = Object.create(this);
    return obj;
}

RigidBodySolver.prototype.start= function () {
    this.isRunning = true;
    this.time = Date.now();
}

RigidBodySolver.prototype.stop = function () {
    this.isRunning = false;
}

RigidBodySolver.prototype.weightForce = function (body) {

    // Weight Force ( Fw = m * g)
    var Fw = this.g.scaleBy(body.m);
    return Fw;
}

RigidBodySolver.prototype.airDragForce = function (body) {

    // Air Drag Force ( FD = 1/2 * rho * v² * Cd * A)
    var Cd = 0.47;
    var A = 0.5; //body.morph.width() * 0.001;
    var rho = 1.2041;
    var v = body.v.magnitude();
    var vv = v*v;
    var Fd = body.v.normalize().scaleBy(-1 * 0.5 * Cd * rho * vv * Cd * A);
    return Fd;
}

RigidBodySolver.prototype.applyForces = function (bodies, dt) {

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

RigidBodySolver.prototype.detectCollisions = function (bodies) {


    var collisions = [];
    for (var i = 0; i < bodies.length; i++) {
        var bodyA = bodies[i];
        for (var j = i+0; j < bodies.length; j++) {
            var bodyB = bodies[j];
            if (i != j && (bodyA.m > 0 || bodyB.m > 0)) {
                if (bodyA.overlapsBoundingBox(bodyB)) {
                    var collisionData = bodyA.collisionData(bodyB);
                    if (!isNil(collisionData)) {
                        collisions.push(collisionData);
                    }
                }
            }
        }
    }
    return collisions;
}

RigidBodySolver.prototype.solveCollisions = function (collisions, dt) {

    var me = this;
    collisions.forEach(function (collision) {

        var a = collision.a,
            an = collision.an,
            b = collision.b,
            bn = collision.bn,
            p = collision.p,
            e = 0.5 * (a.e + b.e);
        a.v = an.scaleBy(b.v.magnitude()*e);
        b.v = bn.scaleBy(a.v.magnitude()*e);

        me.drawCollisionInfo(a, an, b, bn, p, e);
    });
}

RigidBodySolver.prototype.drawCollisionInfo = function (a, an, b, bn, p, e) {

    if (!RB_DEBUG) { return; }

    var stage = this.morph.parentThatIsA(StageMorph),
        ctx = stage.image.getContext("2d");

    ctx.fillStyle="white";
    ctx.fillRect(0,0, ctx.canvas.width, ctx.canvas.height);
    ctx.strokeWidth=10;

    ctx.beginPath();
    ctx.strokeStyle="red";
    //ctx.rect(p.x-stage.left(), p.y-stage.top(), 2,2);
    ctx.moveTo(p.x-stage.left(), p.y-stage.top());
    ctx.lineTo(p.x-stage.left()+an.scaleBy(100).x, p.y-stage.top()+an.scaleBy(100).y);
    ctx.stroke();

    ctx.beginPath();
    ctx.strokeStyle="green";
    //ctx.rect(p.x-stage.left(), p.y-stage.top(), 2, 2);
    ctx.moveTo(p.x-stage.left(), p.y-stage.top());
    ctx.lineTo(p.x-stage.left()+bn.scaleBy(100).x, p.y-stage.top()+bn.scaleBy(100).y);

    ctx.stroke();
}

RigidBodySolver.prototype.step = function (bodies) {

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

        var collisions = [];
        collisions = this.detectCollisions(bodies);
        this.solveCollisions(collisions);

        // Velocity Verlet Conclusion - Compute average acceleration & velocity
        bodies.forEach(function (body) {
            if (body instanceof RigidBody && body.m > 0) {
                body.update(dt);
            }
        })
    }
}