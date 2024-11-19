
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | inverse kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // compute joint angle controls to move location on specified link to Cartesian location
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        // if update requested, call ik iterator and show endeffector and target
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);
        if (kineval.params.trial_ik_random.execute)
            kineval.randomizeIKtrial();
        else // KE: this use of start time assumes IK is invoked before trial
            kineval.params.trial_ik_random.start = new Date();
    }

    kineval.params.update_ik = false; // clear IK request for next iteration
}

kineval.randomizeIKtrial = function randomIKtrial () {

    // update time from start of trial
    cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime()-kineval.params.trial_ik_random.start.getTime();

    endeffector_world = matrix_multiply(robot.joints[robot.endeffector.frame].xform,robot.endeffector.position);

    // compute distance of endeffector to target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
        Math.pow(kineval.params.ik_target.position[0][0] - endeffector_world[0][0], 2.0)
        + Math.pow(kineval.params.ik_target.position[1][0] - endeffector_world[1][0], 2.0)
        + Math.pow(kineval.params.ik_target.position[2][0] - endeffector_world[2][0], 2.0) );

    // if target reached, increment scoring and generate new target location
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2*(Math.random()-0.5);
        kineval.params.ik_target.position[1][0] = 1.2*(Math.random()-0.5)+1.5;
        kineval.params.ik_target.position[2][0] = 0.7*(Math.random()-0.5)+0.5;
        
        kineval.params.trial_ik_random.targets++;
        textbar.innerHTML = "IK trial Random: target " + kineval.params.trial_ik_random.targets + " reached at time " + kineval.params.trial_ik_random.time;
    }

    // STENCIL: see instructor for random time trial code
}

kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // STENCIL: implement inverse kinematics iteration

    // [NOTICE]: Please assign the following 3 variables to test against CI grader

    // ---------------------------------------------------------------------------
    robot.dx = []              // Error term, matrix size: 6 x 1, e.g., [[1],[1],[1],[0],[0],[0]]
    robot.jacobian = []        // Jacobian matrix of current IK iteration matrix size: 6 x N
    robot.dq = []              // Joint configuration change term (don't include step length)  
    // ---------------------------------------------------------------------------

    // Explanation of above 3 variables:
    // robot.dq = T(robot.jacobian) * robot.dx  // where T(robot.jacobian) means apply some transformations to the Jacobian matrix, it could be Transpose, PseudoInverse, etc.
    // dtheta = alpha * robot.dq   // alpha: step length
    
    // robot.dx: Init as a 6x1 matrix

    for (let i = 0; i < 6; i++) {
        robot.dx[i] = [0]; // square 
    }
    
    // Array of all robot joints
    var curJoint = endeffector_joint;
    var jointArray = [];
    jointArray.push(curJoint);

    // Add all joints until base
    while (robot.joints[curJoint].parent != robot.base) {
        curJoint = robot.links[robot.joints[curJoint].parent].parent; // Traverse upwards
        jointArray.push(curJoint);
    }

    // Reverse it so that base is last in array
    jointArray = jointArray.reverse();

    // Declare & init jacobian stuff
    var jacob = [[], [], [], [], [], []];

    // Get transform for end effector position  with respect to its joint
    var p_tool = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local);
    
    // Use lecture slides for this
    for (let i = 0; i < jointArray.length; i++) {
        var joint_name = jointArray[i];
        var joint = robot.joints[joint_name];

        var rotAxis = [[joint.axis[0]], [joint.axis[1]], [joint.axis[2]], [1]];
        var k_i = matrix_multiply(joint.xform, rotAxis);
        let o_i = matrix_multiply(joint.xform, [[0], [0], [0], [1]]);

        var J_vi_left = [];
        var J_vi_right = [];

        for (let j = 0; j < 3; j++) {
            J_vi_left.push(k_i[j][0] - o_i[j][0]);
            J_vi_right.push(p_tool[j][0] - o_i[j][0]);
        }
        var J_vi = vector_cross(J_vi_left, J_vi_right);  
        
        var jacobian_parameters = []
        if (robot.joints[joint_name].type === "revolute" || robot.joints[joint_name].type === "continuous" || robot.joints[joint_name].type === undefined) {
            jacobian_parameters = [J_vi[0], J_vi[1], J_vi[2], J_vi_left[0], J_vi_left[1], J_vi_left[2]];
        }
        else {
            jacobian_parameters = [J_vi_left[0], J_vi_left[1], J_vi_left[2], 0, 0, 0];
        }

        for (let j = 0; j < 6; j++) {
            jacob[j][i] = jacobian_parameters[j];
        }
    }

    robot.jacobian = jacob;

    for (let i = 0; i < 3; i++) {
        robot.dx[i] = [endeffector_target_world.position[i][0] - p_tool[i][0]];
    }

    // IF NOT PSEUDOINVERSE USE TRANSPOSE
    if (!kineval.params.ik_pseudoinverse) {
        robot.dq = matrix_multiply(matrix_transpose(robot.jacobian), robot.dx);
    }
    else {
        robot.dq = matrix_multiply(matrix_pseudoinverse(robot.jacobian), robot.dx);
    }

    for (let i = 0; i < jointArray.length; i++) {
        let output = robot.dq[i] * kineval.params.ik_steplength;
        robot.joints[jointArray[i]].control = output;
    }
}



