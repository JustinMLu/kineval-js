/*
  This function checks if the user has requested an IK update 
  and, if so, calls iterateIK() for a single iteration. It also 
  manages random trials if that feature is enabled.
*/
kineval.robotInverseKinematics = function robot_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // Only run IK if we have an update request or persistent IK enabled
    if ((kineval.params.update_ik)||(kineval.params.persist_ik)) { 
        
        // Run one iteration of IK
        kineval.iterateIK(endeffector_target_world, endeffector_joint, endeffector_position_local);

        // If random IK trials are running, possibly pick a new random target
        if (kineval.params.trial_ik_random.execute) {
            kineval.randomizeIKtrial();
        } 
        else {
            kineval.params.trial_ik_random.start = new Date();
        }
    }

    // Clear the "single update" flag so we don't repeat unless requested again
    kineval.params.update_ik = false;
}


/*
  This function randomly chooses a new IK target when the end-effector 
  is within a small distance of the current target. It also tracks 
  how many random targets have been reached and how long it took.
*/
kineval.randomizeIKtrial = function randomIKtrial () {

    // Update the elapsed time since we started this random trial
    var cur_time = new Date();
    kineval.params.trial_ik_random.time = cur_time.getTime() - kineval.params.trial_ik_random.start.getTime();

    // Get the current end-effector position in world coordinates
    var cur_position = matrix_multiply(robot.joints[robot.endeffector.frame].xform,
                                            robot.endeffector.position);

    // Calculate the distance between the end-effector and the current target
    kineval.params.trial_ik_random.distance_current = Math.sqrt(
        Math.pow(kineval.params.ik_target.position[0][0] - cur_position[0][0], 2) +
        Math.pow(kineval.params.ik_target.position[1][0] - cur_position[1][0], 2) +
        Math.pow(kineval.params.ik_target.position[2][0] - cur_position[2][0], 2)
    );

    // If the end-effector is close enough, generate a new random target
    if (kineval.params.trial_ik_random.distance_current < 0.01) {
        kineval.params.ik_target.position[0][0] = 1.2 * (Math.random() - 0.5);
        kineval.params.ik_target.position[1][0] = 1.2 * (Math.random() - 0.5) + 1.5;
        kineval.params.ik_target.position[2][0] = 0.7 * (Math.random() - 0.5) + 0.5;
        
        // Increase the target count and give user feedback
        kineval.params.trial_ik_random.targets++;
        textbar.innerHTML = "IK trial Random: target " + 
            kineval.params.trial_ik_random.targets + 
            " reached at time " + 
            kineval.params.trial_ik_random.time;
    }

    // Additional logic for random trials can be inserted here if desired
}


/*
  This function performs a single iteration of inverse kinematics (IK).
  It constructs the Jacobian, calculates the error (robot.dx), and 
  updates the joint controls based on the chosen IK method (transpose or pseudoinverse).
*/
kineval.iterateIK = function iterate_inverse_kinematics(endeffector_target_world, endeffector_joint, endeffector_position_local) {

    // Per the stencil, we store these variables in robot for CI/testing:
    // robot.dx: the 6x1 error vector
    // robot.jacobian: the 6xN Jacobian matrix
    // robot.dq: the Nx1 joint angle update (before multiplying by step length)
    robot.dx = [];
    robot.jacobian = [];
    robot.dq = [];

    // Initialize the 6x1 error vector with zeros (3 for position + 3 for orientation)
    for (let i = 0; i < 6; i++) {
        robot.dx[i] = [0];
    }
    
    // Collect all joints from the end-effector back up to the base
    var curJoint = endeffector_joint;
    var jointArray = [];
    jointArray.push(curJoint);

    // Traverse upward until we hit the robot's base
    while (robot.joints[curJoint].parent != robot.base) {
        // 'parent' is the next link, so find the joint that leads us upward
        curJoint = robot.links[robot.joints[curJoint].parent].parent;
        jointArray.push(curJoint);
    }

    // Reverse the list so it starts from the base and ends at the end-effector
    jointArray = jointArray.reverse();

    // Prepare a 2D array for the 6xN Jacobian
    var jacob = [[], [], [], [], [], []];

    // Compute the current end-effector position in world coordinates
    var p_tool = matrix_multiply(robot.joints[endeffector_joint].xform, endeffector_position_local);

    // Fill each column of the Jacobian for each joint in the chain
    for (let i = 0; i < jointArray.length; i++) {
        var joint_name = jointArray[i];
        var joint = robot.joints[joint_name];

        // Convert the joint’s local rotation axis into the world frame
        var rotAxis = [[joint.axis[0]], [joint.axis[1]], [joint.axis[2]], [1]];
        var k_i = matrix_multiply(joint.xform, rotAxis);

        // Compute the joint origin in world coordinates
        var o_i = matrix_multiply(joint.xform, [[0], [0], [0], [1]]);

        // Prepare the cross product for the linear part: 
        // cross( jointAxis_in_world, (endEffectorPos - jointPos) )
        var J_vi_left = [];
        var J_vi_right = [];
        for (let j = 0; j < 3; j++) {
            J_vi_left.push(k_i[j][0] - o_i[j][0]);      // direction of joint axis
            J_vi_right.push(p_tool[j][0] - o_i[j][0]); // vector from joint to end-effector
        }
        var J_vi = vector_cross(J_vi_left, J_vi_right);  

        // Build the 6-element column vector:
        // For revolute joints: [ cross(axis, r), axis ]
        // For prismatic joints: [ axis, 0 ]
        var jacobian_parameters = [];
        if (joint.type === "revolute" || 
            joint.type === "continuous" ||
            joint.type === undefined) {
            
            // Rotational
            jacobian_parameters = [
                J_vi[0],     // linear X
                J_vi[1],     // linear Y
                J_vi[2],     // linear Z
                J_vi_left[0],// angular X
                J_vi_left[1],// angular Y
                J_vi_left[2] // angular Z
            ];
        } else {
            // Prismatic
            jacobian_parameters = [
                J_vi_left[0], 
                J_vi_left[1], 
                J_vi_left[2], 
                0, 
                0, 
                0
            ];
        }

        // Place these values into the Jacobian’s ith column
        for (let j = 0; j < 6; j++) {
            jacob[j][i] = jacobian_parameters[j];
        }
    }

    // Store the computed Jacobian in the robot object (e.g., for debugging/visualizing)
    robot.jacobian = jacob;

    // Compute the position error for the first 3 entries in robot.dx
    // (ignoring orientation in this code snippet)
    for (let i = 0; i < 3; i++) {
        robot.dx[i] = [
            endeffector_target_world.position[i][0] - p_tool[i][0]
        ];
    }

    // Choose either the transpose method or pseudoinverse for solving IK
    if (!kineval.params.ik_pseudoinverse) {
        // Use Jacobian Transpose
        robot.dq = matrix_multiply(matrix_transpose(robot.jacobian), robot.dx);
    } else {
        // Use Moore-Penrose Pseudoinverse
        robot.dq = matrix_multiply(matrix_pseudoinverse(robot.jacobian), robot.dx);
    }

    // Apply a step length factor and update each joint's control
    for (let i = 0; i < jointArray.length; i++) {
        var output = robot.dq[i] * kineval.params.ik_steplength;
        robot.joints[jointArray[i]].control = output;
    }
};
