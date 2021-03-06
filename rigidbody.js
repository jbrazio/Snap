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

Point.prototype.dot = function(point) {
    return new Point (this.x * point.x, this.y * point.y);
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
    //     this.i          Point    Impulse vector
    // ------------------------------------------------------------------------
    // Derived:
    //     this.v    Point    Velocity of the body
    // ------------------------------------------------------------------------
    // Computed:
    //     this.f    Point    Force applied on the body
    //     this.a    Point    Acceleration computed by as a = F / m (F = m x a)

    //     this.w    Scalar   Omega || Angular velocity
    //     this.al   Scalar   Alpha || Angular acceleration
    // ------------------------------------------------------------------------
    this.m = Math.max(0, mass ? mass : 0);
    this.e = Math.max(0, restitution ? restitution : 0);
    this.springs = new Array();
    this.i = new Vector2d(0, 0);
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
    var springs = this.springs;
    this.reset();
    this.springs = springs;
}

RigidBody.prototype.reset = function () {
    this.f = new Vector2d(0, 0);
    this.v = new Vector2d(0, 0);
    this.a = new Vector2d(0, 0);
    this.springs = [];
    this.i = new Vector2d(0, 0);
}

RigidBody.prototype.updatePositions = function (dt) {
    var dr = this.v.scaleBy(dt).add(this.a.scaleBy(0.5*dt*dt));
//    var dr = this.v.add(this.a.scaleBy(0.5*dt*dt));
    this.p = this.p.add(dr.scaleBy(1));
}

RigidBody.prototype.update = function (dt) {
    var new_a = this.f.divideBy(this.m);
    var avg_a = this.a.add(new_a).scaleBy(0.5);
    this.v = this.v.add(avg_a.scaleBy(dt));
    this.a = avg_a;
}

RigidBody.prototype.overlapsBoundingBox = function (other) {
    return this.morph.bounds.intersects(other.morph.bounds);
}

RigidBody.prototype.boundingCircleRadius = function() {
    var w = this.morph.fullImage().width,
        h = this.morph.fullImage().height;
//    return Math.sqrt((0.5*w + 0.5*w) + (0.5*h + 0.5*h));
    return Math.max(w,h)*0.5;
}

RigidBody.prototype.collisionData = function (other) {
    return this._collisionData(other);
}

RigidBody.prototype._collisionData = function (other) {
    var distanceVector = other.p.subtract(this.p),
        distance = distanceVector.magnitude(),
        boundingCircleRadius1 = this.boundingCircleRadius(),
        boundingCircleRadius2 = other.boundingCircleRadius(),
        collisionDistance = boundingCircleRadius1 + boundingCircleRadius2,
        depth = collisionDistance - distance;

    if (distance > collisionDistance) {
        return null;
    }

    var an = distanceVector.normalize(),
        bn = this.p.subtract(other.p).normalize(),
        p  = this.p.add(an.scaleBy(depth));

    return {
        a: this,
        b: other,
        p: p,
        an: an,
        bn: bn,
        depth: depth
    }
}

RigidBody.prototype._collisionData2 = function (other) {
    var oRect = this.morph.bounds.intersect(other.morph.bounds),
        oImg = this.morph.overlappingImage(other.morph),
        data = oImg.getContext('2d')
                   .getImageData(1, 1, oImg.width, oImg.height)
                   .data,
        p = undefined;

    // compute contact pointXS within overlapped image
    for (var i = 3; i < data.length; i+=4) {
        if (data[i] > 128) {
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
            if (isNil(n) || corner.subtract(p).magnitude() < n.subtract(p).magnitude()) {
                n =  corner.subtract(p);
            }
        });
        var n1 = n.swap(), n2 = n.swap2();
        n = morph.center().subtract(n1).magnitude() < morph.center().subtract(n2).magnitude() ? n1: n2;
        return n;
    }
    an = findClosestCorner(this.morph, p);
    bn = findClosestCorner(other.morph, p);

    return {
        a: this,
        b: other,
        p: p,
        an: an,
        bn: bn,
        rect: oRect
    }
}

RigidBody.prototype.impulseForce = function(dt) {
    var i = this.i;
    this.i = new Vector2d(0, 0);
    return i;
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
    this.lastTime = Date.now();
    this.timeLeftOver = 0;
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
    var new_f = new Vector2d(0,0);
    bodies.forEach(function (body) {
        if (body instanceof RigidBody && body.m > 0.0) {
            new_f = new Vector2d(0, 0);
            new_f.plus(solver.weightForce(body));
            new_f.plus(solver.airDragForce(body));
            new_f.plus(body.springsNetForce());
            new_f.plus(body.impulseForce());
            body.f = new_f;
        }
    });
}

RigidBodySolver.prototype.detectCollisions = function (bodies) {


    var collisions = [];
    for (var i = 0; i < bodies.length; i++) {
        var bodyA = bodies[i];
        for (var j = i+0; j < bodies.length; j++) {
            var bodyB = bodies[j];
            if (i != j && (bodyA.m > 0.0 || bodyB.m > 0.0)) {
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
            e = Math.min(a.e, b.e),
            d = collision.depth;

        a.v = a.v.subtract(bn.scaleBy(a.v.magnitude()*-1*e));
        b.v = b.v.subtract(an.scaleBy(b.v.magnitude()*-1*e));
        //a.v = an.scaleBy(b.v.magnitude()*e);
        //b.v = bn.scaleBy(a.v.magnitude()*e);
        //a.v = an.normalize().scaleBy(-1*e);
        //b.v = bn.normalize().scaleBy(-1*e);

        me.drawCollisionInfo(a, an, b, bn, p, e);
    });
}

RigidBodySolver.prototype.drawCollisionInfo = function (a, an, b, bn, p, e, rect) {

    if (!RB_DEBUG) { return; }

    var stage = this.morph.parentThatIsA(StageMorph),
        ctx = stage.penTrails().getContext("2d"),
        scale = ctx.canvas.width / stage.width(),
        aImg = a.morph.fullImage(),
        bImg = b.morph.fullImage();

    ctx.save();

    ctx.fillStyle="white";
    ctx.fillRect(0,0, stage.width(), stage.height());
    ctx.scale(scale, scale);

    ctx.fillStyle="red";
    ctx.beginPath();
    ctx.fillRect(a.p.x-stage.left(), a.p.y-stage.top(), 2, 2);

    ctx.beginPath();
    ctx.fillRect(b.p.x-stage.left(), b.p.y-stage.top(), 2, 2);

    ctx.strokeStyle="orange";
    ctx.beginPath();
    ctx.moveTo(a.p.x-stage.left(), a.p.y-stage.top());
    ctx.lineTo(a.p.x-stage.left()+bn.x*10, a.p.y-stage.top()+bn.y*10);
    ctx.stroke();

    ctx.strokeStyle="green";
    ctx.beginPath();
    ctx.moveTo(b.p.x-stage.left(), b.p.y-stage.top());
    ctx.lineTo(b.p.x-stage.left()+an.x*10, b.p.y-stage.top()+an.y*10);
    ctx.stroke();
/*
    ctx.strokeStyle="blue";
    ctx.beginPath();
    ctx.arc(a.p.x-stage.left(),a.p.y-stage.top(),a.boundingCircleRadius(),0,2*Math.PI);
    ctx.stroke();
    ctx.beginPath();
    ctx.arc(b.p.x-stage.left(),b.p.y-stage.top(),b.boundingCircleRadius(),0,2*Math.PI);
    ctx.stroke();
*/
    ctx.fillStyle="pink";
    ctx.beginPath();
    ctx.fillRect(p.x-stage.left(), p.y-stage.top(),2,2);

    ctx.restore();
    stage.changed();
}

RigidBodySolver.prototype.drawForces = function(bodies) {

    if (!RB_DEBUG) { return; }


    var stage = this.morph.parentThatIsA(StageMorph),
        ctx = stage.penTrails().getContext("2d"),
        scale = ctx.canvas.width / stage.width();

    ctx.save();
//    ctx.fillStyle="white";
//    ctx.fillRect(0,0, stage.width()*scale, stage.height()*scale);

    ctx.scale(scale, scale);

    bodies.forEach(function(body) {

        ctx.strokeStyle="red";
        ctx.beginPath();
        ctx.moveTo(body.p.x-stage.left()-1, body.p.y-stage.top());
        ctx.lineTo(body.p.x-stage.left()-1+body.f.x/scale, body.p.y-stage.top()+body.f.y/scale);
        ctx.stroke();

        ctx.strokeStyle="blue";
        ctx.beginPath();
        ctx.moveTo(body.p.x-stage.left()+1, body.p.y-stage.top());
        ctx.lineTo(body.p.x-stage.left()+1+body.v.x/scale, body.p.y-stage.top()+body.v.y/scale);
        ctx.stroke();

        //ctx.drawImage(body.morph.fullImage(), 0, 0, 100, 100);
    });

    ctx.restore();
    stage.changed();
}

RigidBodySolver.prototype.step = function (bodies) {

    var now = Date.now(),
        elapsedTime = (now - this.lastTime) + this.timeLeftOver,
        timesteps = Math.floor(elapsedTime / this.stepSize),
        dt = this.stepSize * this.stepSizeScale;
    this.lastTime = now;
    this.timeLeftOver = elapsedTime - timesteps * this.stepSize;

    timesteps = Math.max(Math.min(timesteps,10),1);

    //console.log(now, elapsedTime, timesteps, this.lastTime, this.timeLeftOver);

    for (var i = 0; i < timesteps; i++) {

        // Velocity Verlet Start - Update Positions
        bodies.forEach(function (body) {
            if (body instanceof RigidBody && body.m > 0.0) {
                body.updatePositions(dt);
            }
        })

        this.applyForces(bodies, dt);

        var collisions = [];
        collisions = this.detectCollisions(bodies);
        this.solveCollisions(collisions);

        // Velocity Verlet Conclusion - Compute average acceleration & velocity
        bodies.forEach(function (body) {
            if (body instanceof RigidBody && body.m > 0.0) {
                body.update(dt);
            }
        });

        //this.drawForces(bodies);
    }
}