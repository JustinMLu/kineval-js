//////////////////////////////////////////////////
/////     QUATERNION TRANSFORM ROUTINES 
//////////////////////////////////////////////////

// STENCIL: reference quaternion code has the following functions:
//   quaternion_from_axisangle
//   quaternion_normalize
//   quaternion_to_rotation_matrix
//   quaternion_multiply

// **** Function stencils are provided below, please uncomment and implement them ****//

kineval.quaternionFromAxisAngle = function quaternion_from_axisangle(axis,angle) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
    var cosAngle = Math.cos(angle/2); // half angles
    var sinAngle = Math.sin(angle/2); // half angles

    q.a = cosAngle; // real
    q.b = axis[0] * sinAngle; // imaginary
    q.c = axis[1] * sinAngle; // imaginary
    q.d = axis[2] * sinAngle; // imaginary

    //  RETURNS (C, XS, YS, ZS) WHERE C = COS(ANGLE/2) AND S = SIN(ANGLE/2)
    return q;
}

kineval.quaternionNormalize = function quaternion_normalize(q1) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
    var l2 = Math.sqrt(Math.pow(q1.a, 2) 
            + Math.pow(q1.b, 2) 
            + Math.pow(q1.c, 2) 
            + Math.pow(q1.d, 2)
            );
    
    q.a = q1.a / l2;
    q.b = q1.b / l2;
    q.c = q1.c / l2;
    q.d = q1.d / l2;

    return q;
    
}

kineval.unitQuaternionFromAxisAngle = function unit_quaternion_from_axisangle(axis,angle) {
    return kineval.quaternionNormalize(kineval.quaternionFromAxisAngle(axis, angle));
}


// ARE YOU HAPPY NOW AUTOGRADER??????
kineval.quaternionMultiply = function quaternion_multiply(q1,q2) {
    // returns quaternion q as dic, with q.a as real number, q.b as i component, q.c as j component, q.d as k component
    var q = {};
    
    q.a = (q1.a*q2.a) - (q1.b*q2.b) - (q1.c*q2.c) - (q1.d*q2.d);

    q.b = (q1.a*q2.b) + (q1.b*q2.a) + (q1.c*q2.d) - (q1.d*q2.c);

    q.c = (q1.a*q2.c) - (q1.b*q2.d) + (q1.c*q2.a) + (q1.d*q2.b);
    
    q.d = (q1.a*q2.d) + (q1.b*q2.c) - (q1.c*q2.b) + (q1.d*q2.a);
    
    return q;
}

// please work
kineval.quaternionToRotationMatrix = function quaternion_to_rotation_matrix (q) {
    // returns 4-by-4 2D rotation matrix
    var q0 = q.a;
    var q1 = q.b;
    var q2 = q.c;
    var q3 = q.d;
    var q0_2 = q.a * q.a;
    var q1_2 = q.b * q.b;
    var q2_2 = q.c * q.c;
    var q3_2 = q.d * q.d;

    return [
        [ (q0_2+q1_2-q2_2-q3_2) , 2*(q1*q2-q0*q3) , 2*(q0*q2+q1*q3) , 0 ],
        [ 2*(q1*q2+q0*q3) , (q0_2-q1_2+q2_2-q3_2) , 2*(q2*q3-q0*q1) , 0 ],
        [ 2*(q1*q3-q0*q2) , 2*(q2*q3+q0*q1) , (q0_2-q1_2-q2_2+q3_2) , 0 ],
        [ 0, 0, 0, 1]
        ];

}