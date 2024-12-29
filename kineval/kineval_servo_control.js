
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | arm servo control

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Justin Lu, Chad Jenkins, and the 
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.setpointDanceSequence = function execute_setpoints() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_dance) return; 

    // STENCIL: implement FSM to cycle through dance pose setpoints
    kineval.params.dance_pose_index %= kineval.params.dance_sequence_index.length;
    var num_success = 0;

    for (var joint in robot.joints) {
        // Get targets from: dance sequence index[dance pose index] --> returns setpoint index --> get from kineval setpoints
        var targets = kineval.setpoints[kineval.params.dance_sequence_index[kineval.params.dance_pose_index]];
        kineval.params.setpoint_target[joint] = targets[joint];
        if (Math.abs(targets[joint] - robot.joints[joint].angle) < 0.0075) num_success++;
    }

    // If all joints moved correctly to target, go to the next setpoint
    if (num_success === Object.keys(robot.joints).length) {
        kineval.params.dance_pose_index = (kineval.params.dance_pose_index + 1) % kineval.params.dance_sequence_index.length;
    }
}

kineval.setpointClockMovement = function execute_clock() {

    // if update not requested, exit routine
    if (!kineval.params.update_pd_clock) return; 

    var curdate = new Date();
    for (x in robot.joints) {
        kineval.params.setpoint_target[x] = curdate.getSeconds()/60*2*Math.PI;
    }
}


kineval.robotArmControllerSetpoint = function robot_pd_control () {

    // if update not requested, exit routine
    if ((!kineval.params.update_pd)&&(!kineval.params.persist_pd)) return; 

    kineval.params.update_pd = false; // if update requested, clear request and process setpoint control

    // STENCIL: implement P servo controller over joints
    for (joint in robot.joints) {
        var target = kineval.params.setpoint_target[joint];
        var cur = robot.joints[joint].angle;

        var error = target - cur; // separated for debug
        robot.joints[joint].servo.p_gain = 0.175;
        robot.joints[joint].control = error * robot.joints[joint].servo.p_gain; // just P controller
    }
}


