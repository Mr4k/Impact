const epsilon = 0.000001;

function RigidBody(geom) {
    this.geom = geom;
    this.geom.computeBoundingSphere();
    this.geom.computeFaceNormals();
    this.acceleration = new THREE.Vector3(0,0,0);
    this.velocity = new THREE.Vector3(0,0,0);
    this.angularAcceleration = new THREE.Vector3(0,0,0);
    this.angularVelocity = new THREE.Vector3(0,0,0);
    this.inertialTensor = new THREE.Matrix4();
    var medges = [];
    //create edges from faces
    for (var i=0; i<geom.vertices.length; i++){
        medges.push([]);
    }
    for (var i = 0; i < geom.faces.length; i++){
        var arr = [geom.faces[i].a,geom.faces[i].b,geom.faces[i].c];
        for (var j=0;j<3;j++){
            var a = arr[j];
            var b = arr[(j+1)%3];
            if (b < a){
                var c = b;
                b = a;
                a = c;
            }
            //this could be faster via binary search or something
            var has = false;
            for (var k = 0;k<medges[a].length;k++){
                if (medges[a][k] == b){
                    has = true;
                    break;
                }
            }
            if (!has)
                medges[a].push(b);
        }
    }
    this.edges = [];
    for (var i=0; i<geom.vertices.length; i++){
        for (var j=0;j<medges[i].length;j++){
            this.edges.push([i,medges[i][j]]);
        }
    }
    //compute center of mass (and for now assume all vertices have equal weight)
    this.com = new THREE.Vector3(0,0,0);
    for (var i=0; i<geom.vertices.length; i++){
        this.com.add(geom.vertices[i]);
    }
    this.com.multiplyScalar(1.0/geom.vertices.length);
}

//the main update function used for rigidbodies
RigidBody.prototype.translationStep = function(delta, otherBodies){

    //handle translation first
    //scan ahead to move by delta and see if anyone is blocking you
    var translationalDelta = -1
    var objectHit = null
    for (var i = 0; i < otherBodies.length; i++){
        if (otherBodies[i] == this)
            continue
        if (otherBodies[i].com.clone().sub(this.com).length() > otherBodies[i].geom.boundingSphere.radius 
            + this.geom.boundingSphere.radius + this.velocity.length() * delta)
            continue
        var c = collide(this, otherBodies[i], delta, false)
        if ((c < translationalDelta && c >= 0) || (translationalDelta < 0 && c >= 0)){
            translationalDelta = c
            objectHit = otherBodies[i]
        }
    }
    if (translationalDelta < 0)
        translationalDelta = delta
    translationalDelta -= 2*epsilon
    translationalDelta = Math.max(translationalDelta,0)
    //move the body foward
    var tr = this.velocity.clone().multiplyScalar(translationalDelta);
    var mat = new THREE.Matrix4().makeTranslation(tr.x,tr.y,tr.z);
    this.geom.applyMatrix(mat);
    this.com.add(tr)
    return objectHit;
}

RigidBody.prototype.rotationStep = function(delta, otherBodies){
    //now handle rotation
    //scan ahead by delta
    var rotationalDelta = -1
    var objectHit = null
    var s = this.angularVelocity.length();
    for (var i = 0; i < otherBodies.length; i++){
        if (otherBodies[i] == this)
            continue
        var c = collide(this, otherBodies[i], delta, true)
        if ((c < rotationalDelta && c >= 0) || (rotationalDelta < 0 && c >= 0)){
            rotationalDelta = c
            objectHit = otherBodies[i]
        }
    }
    if (rotationalDelta < 0)
        rotationalDelta = delta
    rotationalDelta -= 2*epsilon
    rotationalDelta = Math.max(rotationalDelta,0)

    //rotate the body
    mat = new THREE.Matrix4().makeTranslation(this.com.x,this.com.y,this.com.z);
    mat.multiply(new THREE.Matrix4().makeRotationAxis(this.angularVelocity.clone().normalize(),rotationalDelta));
    mat.multiply(new THREE.Matrix4().makeTranslation(-this.com.x,-this.com.y,-this.com.z));
    this.geom.applyMatrix(mat);
    return objectHit
}
/*THESE FUNCTIONS ARE FOR CACULATING THE INERTIAL TENSOR AND TRUE CENTER OF MASS
  CURRENTLY THESE FEATURES ARE NOT SUPPORTED AND THESE FUNCTIONS ARE INCOMPLETE
function RigidBody.prototype.makeInertialTensor(){
    this.inertialTensor = 
}

function decomposeRigidBody(rb){
    var tetrahedra = [];
    
}

function transformTetrahedron(a,b,c,d){
    return new THREE.Matrix4(b.x,c.x,d.x,0,b.y,c.y,d.y,0,b.z,c.z,d.z,0,0,0,0,1);
}*/

function pointInTriangle(p, a,b,c){
    // Compute vectors        
    var v0 = c.clone().sub(a);
    var v1 = b.clone().sub(a);
    var v2 = p.clone().sub(a);

    // Compute dot products
    var dot00 = v0.dot(v0);
    var dot01 = v0.dot(v1);
    var dot02 = v0.dot(v2);
    var dot11 = v1.dot(v1);
    var dot12 = v1.dot(v2);

    // Compute barycentric coordinates
    var invDenom = 1 / (dot00 * dot11 - dot01 * dot01)
    var u = (dot11 * dot02 - dot01 * dot12) * invDenom
    var v = (dot00 * dot12 - dot01 * dot02) * invDenom
    // Check if point is in triangle
    return [u,v];
}

function pointInTriangleFull(p,a,b,c){
    var uv = pointInTriangle(p,a,b,c);
    return (uv[0]>=0&&uv[1]>=0&&uv[0]+uv[1]<1);
}

function pointToTriTrans(vel,delta,point,a,b,c,normal){
    var nD = normal.dot(vel);
    if (Math.abs(nD) < epsilon){
        //I think this case would be an edge to edge
        return -1;
    } else {
        var t = -normal.dot(point.clone().sub(c))/nD;
        if (t >= 0 && t<=delta)
        {
            var pt = point.clone().add(vel.multiplyScalar(t));
            var r = pointInTriangle(pt,a,b,c);
            if (r[0] >=0 && r[1]>=0 && r[0]+r[1] < 1)
                return t;
            else
                return -1;
        }
        else
            return -1;
    }
}

function edgeToEdgeTrans(vel,delta,a,b,c,d){
    var normal = new THREE.Vector3(0,0,0);
    var ta = a.clone().sub(b);
    var tc = c.clone().sub(b);
    var td = d.clone().sub(b);
    var tb = new THREE.Vector3(0,0,0);
    normal = normal.crossVectors(ta,vel);
    if (normal.lengthSq() < epsilon)
    {
        return -1;
    }
    var cSubD = tc.clone().sub(td);
    if (Math.abs(normal.dot(cSubD)) < epsilon) {
        return -1;
    }
    else {
        var l = -td.clone().dot(normal)/cSubD.dot(normal);
        if (l < 0 || l > 1)
            return -1;
        var p = td.clone().add(cSubD.multiplyScalar(l));
        var vs = vel.clone().multiplyScalar(delta);
        var v1 = ta;
        var v2 = tb;
        var v3 = tb.clone().add(vs);
        var v4 = ta.clone().add(vs);
        var p1 = pointInTriangle(p,v1,v2,v3);
        var p2 = pointInTriangle(p,v1,v4,v3);
        //return p.add(b);
        if (p1[0]+p1[1] < 1 && p1[0] >= 0 && p1[1] >=0 ){              
            return p1[0]*delta;
        } else if (p2[0]+p2[1] < 1 && p2[0] >= 0 && p2[1] >= 0){
            //not a typo
            return p1[0]*delta;
        } else {
            return -1;
        }
    }    
}

function makeRotationSpace(rots,com){
    var rot = rots.clone().normalize();
    var rotXY = new THREE.Vector3(rot.x,rot.y,0);
    var rotZ = -Math.atan2(rotXY.y,rotXY.x);
    var rotM = new THREE.Matrix4().makeRotationZ(rotZ);
    rot.applyMatrix4(rotM);
    var rotXZ = new THREE.Vector3(rot.x,0,rot.z); 
    var rotY = -Math.PI*2+Math.atan2(rotXZ.z,rotXZ.x) - Math.PI/2;
    var rotMm = new THREE.Matrix4().makeRotationY(rotY);
    //rot.applyMatrix4(rotMm);
    rotMm.multiply(rotM);
    rotMm.multiply(new THREE.Matrix4().makeTranslation(-com.x,-com.y,-com.z));
    var rotMmInv = new THREE.Matrix4().getInverse(rotMm);
    return [rotMm,rotMmInv];
}

//give it points in rotation space
function pointToTriRot(mul,delta,tp,ta,tb,tc,normal){
    if (normal.dot(new THREE.Vector3(0,0,1)) > 1-epsilon){
        return -1;
    } else {
        //set up the plane
        var d = ta.dot(normal);
        m = Math.sqrt(tp.x*tp.x+tp.y*tp.y);
        var g = (d-normal.z*tp.z)/m;
        
        //solve for y
        var a = normal.x;
        var b = normal.y;
        var preRad = 4*g*g*b*b-4*(b*b+a*a)*(g*g-a*a);
        if (preRad < 0)
            return -1;
        var rad = Math.sqrt(preRad);
        var yp = (2*g*b+rad)/2/(b*b+a*a);
        var ym = (2*g*b-rad)/2/(b*b+a*a);
        var xp = (g-yp*b)/a;
        var xm = (g-ym*b)/a;
        var t0 = Math.atan2(yp,xp);
        var t1 = Math.atan2(ym,xm);
        var angleAdder = -Math.atan2(tp.y,tp.x);
        t1+=angleAdder;
        t0+=angleAdder;
        while(t0 >= Math.PI*2) t0-=Math.PI*2;
        while(t1 >= Math.PI*2) t1-=Math.PI*2;
        while(t0 < 0) t0+=Math.PI*2;
        while(t1 < 0) t1+=Math.PI*2;
        if (mul < 0) { t0 = Math.PI*2-t0; t1 = Math.PI*2-t1; }
        if (t1 < delta  && t1 > 0 && pointInTriangleFull(new THREE.Vector3(xm*m,ym*m,tp.z),ta,tb,tc)){
            if (t0 < delta && t0 > 0 && t0 < t1 && pointInTriangleFull(new THREE.Vector3(xp*m,yp*m,tp.z),ta,tb,tc)){
                return t0;
            } else{
                return t1;
            }
        } else if (t0 < delta && t0 > 0 && pointInTriangleFull(new THREE.Vector3(xp*m,yp*m,tp.z),ta,tb,tc)){
            return t0;
        }
        else
            return -1;
    }
}

function solveLines(a,b,c,d){
    var ab = a.clone().sub(b);
    var cd = c.clone().sub(d);
    //TODO: handle edge cases
    
    var dx1 = ab.x;
    var dx2 = cd.x;
    var dy1 = ab.y;
    var dy2 = cd.y;
    var x1 = b.x;
    var x2 = d.x;
    var y1 = b.y;
    var y2 = d.y;
    
    var gamma = (x1+(y2-y1)*dx1/dy1-x2)/(dx2-dy2/dy1*dx1);
    var lambda = (y2+gamma*dy2-y1)/dy1;
    //console.log(gamma,lambda);
    if (gamma < 0 || gamma > 1 || isNaN(gamma))
        return [-1,0,0];
    else if (lambda < 0 || lambda > 1 || isNaN(lambda))
        return [-1,0,0];
    else 
        return [1,gamma,lambda];
}

//what have I done...
function edgeToEdgeRot(delta,ta,tb,tc,td){
    var rot = new THREE.Vector3(0,0,1);
    var tcd = td.clone().sub(tc);
    //a-b is the rotating segment
    //oh god pray to mathematica
    var k = -(ta.y*tb.x*tcd.z-ta.x*tb.y*tcd.z+ta.z*tcd.y*td.x
    -tb.z*tcd.y*td.x-ta.z*tcd.x*td.y+tb.z*tcd.x*td.y);
    var i = ta.z*tb.y*tcd.x-ta.y*tb.z*tcd.x-ta.z*tb.x*tcd.y+ta.x*tb.z*tcd.y-
    ta.y*tcd.z*td.x+tb.y*tcd.z*td.x+ta.x*tcd.z*td.y-tb.x*tcd.z*td.y+ta.y*tcd.x*td.z
    -tb.y*tcd.x*td.z-ta.x*tcd.y*td.z+tb.x*tcd.y*td.z;
    var j = ta.z*tb.x*tcd.x-ta.x*tb.z*tcd.x+ta.z*tb.y*tcd.y-ta.y*tb.z*tcd.y-
    ta.x*tcd.z*td.x+tb.x*tcd.z*td.x-ta.y*tcd.z*td.y+tb.y*tcd.z*td.y+ta.x*tcd.x*td.z-
    tb.x*tcd.x*td.z+ta.y*tcd.y*td.z-tb.y*tcd.y*td.z;
    //solve icos(t)+jsin(t)+k=0
    var rad = 4*k*k*j*j-4*(j*j+i*i)*(k*k-i*i);
    if (rad < 0)
        return -1;
    rad = Math.sqrt(rad);
    var yp = (2*k*j+rad)/(2*(j*j+i*i));
    var ym = (2*k*j-rad)/(2*(j*j+i*i));
    var xp = (k-yp*j)/i;
    var xm = (k-ym*j)/i;
    var t0 = Math.atan2(yp,xp);
    var t1 = Math.atan2(ym,xm);
    //now we can reapply the rotation matrix in-order to find the correct line positions
    if (t0 < 0) t0+=Math.PI*2;
    if (t1 < 0) t1+=Math.PI*2;
    var endTa;
    var endTb;
    var endRay;
    var l;
    if (t0 <= delta){
        endTa = new THREE.Vector3(xp*ta.x-yp*ta.y,xp*ta.y+yp*ta.x,ta.z);
        endTb = new THREE.Vector3(xp*tb.x-yp*tb.y,xp*tb.y+yp*tb.x,tb.z);
        l = solveLines(tc,td,endTa,endTb);
        if (l[0] < 0){
            endTa = new THREE.Vector3(xm*ta.x-ym*ta.y,xm*ta.y+ym*ta.x,ta.z);
            endTb = new THREE.Vector3(xm*tb.x-ym*tb.y,xm*tb.y+ym*tb.x,tb.z);
            l = solveLines(tc,td,endTa,endTb);
            if (l[0] < 0){
                return -1;
            }
            else
                return t1;
        } else
            return t0;
    } else if (t1 <= delta){
        endTa = new THREE.Vector3(xm*ta.x-ym*ta.y,xm*ta.y+ym*ta.x,ta.z);
        endTb = new THREE.Vector3(xm*tb.x-ym*tb.y,xm*tb.y+ym*tb.x,tb.z);
        l = solveLines(tc,td,endTa,endTb);
        if (l[0] < 0){
            return -1;
        }
        else
            return t1;
    }
}


function collide(rigidbody1,rigidbody2,delta,rot){
    //translational collision
    var smallestT = -1;
    
    //prep rotation collision
    var mat;
    if (rot){
        mat = makeRotationSpace(rigidbody1.angularVelocity,rigidbody1.com);
        rigidbody1.geom.applyMatrix(mat[0]);
        rigidbody2.geom.applyMatrix(mat[0]);
        delta*=rigidbody1.angularVelocity.length();
    }
    
    
    //vertex to face
    var i;
    var il;
    for (i = 0, il = rigidbody1.geom.vertices.length; i < il; i ++)
    {
        for (j = 0, jl = rigidbody2.geom.faces.length; j < jl; j++)
        {
            /*if (rigidbody2.geom.faces[j].normal.dot(rigidbody1.velocity) < 0 && !rot)
                continue;*/
            
            var t;
            if (!rot)
                t = pointToTriTrans(rigidbody1.velocity.clone(),delta,
                rigidbody1.geom.vertices[i],rigidbody2.geom.vertices[rigidbody2.geom.faces[j].a]
                ,rigidbody2.geom.vertices[rigidbody2.geom.faces[j].b],
                rigidbody2.geom.vertices[rigidbody2.geom.faces[j].c],rigidbody2.geom.faces[j].normal);
            else
                t = pointToTriRot(1,delta,rigidbody1.geom.vertices[i],rigidbody2.geom.vertices[rigidbody2.geom.faces[j].a]
                ,rigidbody2.geom.vertices[rigidbody2.geom.faces[j].b],
                rigidbody2.geom.vertices[rigidbody2.geom.faces[j].c],rigidbody2.geom.faces[j].normal);
            if (t > 0 && (t<smallestT || smallestT < 0)){
                //console.log('vert candidate',t);
                smallestT = t;
            }
        }
    }
    
    for (i = 0, il = rigidbody2.geom.vertices.length; i < il; i ++)
    {
        for (j = 0, jl = rigidbody1.geom.faces.length; j < jl; j++)
        {
            /*if (rigidbody1.geom.faces[j].normal.dot(rigidbody1.velocity) > 0 && !rot){
                continue;
            }*/
            var t;
            if (!rot)
                t = pointToTriTrans(rigidbody1.velocity.clone().multiplyScalar(-1),
                delta,rigidbody2.geom.vertices[i],rigidbody1.geom.vertices[rigidbody1.geom.faces[j].a]
                ,rigidbody1.geom.vertices[rigidbody1.geom.faces[j].b],
                rigidbody1.geom.vertices[rigidbody1.geom.faces[j].c],rigidbody1.geom.faces[j].normal);
            else
                t = pointToTriRot(-1,delta,rigidbody2.geom.vertices[i],
                rigidbody1.geom.vertices[rigidbody1.geom.faces[j].a]
                ,rigidbody1.geom.vertices[rigidbody1.geom.faces[j].b],
                rigidbody1.geom.vertices[rigidbody1.geom.faces[j].c],rigidbody1.geom.faces[j].normal);
            if (t > 0 && (t<smallestT || smallestT < 0)){
                //console.log('vert2 candidate',t);
                smallestT = t;
            }
        }
    }
    
    //console.log("edge checks");
    //edge to edge
    for (i=0,il = rigidbody1.edges.length;i<il;i++){
        for (j=0,jl = rigidbody2.edges.length;j<jl;j++){
            var t = -1;
            if (!rot)
                t = edgeToEdgeTrans(rigidbody1.velocity,delta,rigidbody1.geom.vertices[rigidbody1.edges[i][0]],
                rigidbody1.geom.vertices[rigidbody1.edges[i][1]],
                rigidbody2.geom.vertices[rigidbody2.edges[j][0]],
                rigidbody2.geom.vertices[rigidbody2.edges[j][1]]);
            else
                t = edgeToEdgeRot(delta,rigidbody1.geom.vertices[rigidbody1.edges[i][0]],
                rigidbody1.geom.vertices[rigidbody1.edges[i][1]],
                rigidbody2.geom.vertices[rigidbody2.edges[j][0]],
                rigidbody2.geom.vertices[rigidbody2.edges[j][1]]);
            if (t > 0 && (t<smallestT || smallestT < 0)){
                //console.log("edge candidate",t);
                smallestT = t;
            }
        }
    }
    
    if (rot){
        rigidbody1.geom.applyMatrix(mat[1]);
        rigidbody2.geom.applyMatrix(mat[1]);
    }
    

    if (smallestT > delta)
        return -1;

    return smallestT;
}