/*

     KinEval
     Implementation of robot kinematics, control, decision making, and dynamics 
     in HTML5/JavaScript and threejs
     
     @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

*/


kineval.initRobotJoints = function initRobotJoints() {
    // build kinematic hierarchy by looping over each joint in the robot
    //   (object fields can be index through array-style indices, object[field] = property)
    //   and insert threejs scene graph (each joint and link are directly connect to scene root)
    // NOTE: kinematic hierarchy is maintained independently by this code, not threejs

    var x,tempmat;

    for (x in robot.joints) {

        // give the joint its name as an id
        robot.joints[x].name = x;

        // initialize joint angle value and control input value
        robot.joints[x].angle = 0;
        robot.joints[x].control = 0;
        robot.joints[x].servo = {};
        //set appropriate servo gains for arm setpoint control
        robot.joints[x].servo.p_gain = 0.1; 
        robot.joints[x].servo.p_desired = 0;
        robot.joints[x].servo.d_gain = 0.01; 
        
/* STENCIL START */ 
    // STENCIL: complete kinematic hierarchy of robot for convenience.
    //   robot description only specifies parent and child links for joints.
    //   additionally specify parent and child joints for each link
    
        var parent_link = robot.joints[x].parent;
        var child_link = robot.joints[x].child;

        // Set child link dependency
        robot.links[child_link].parent = x;

        // Set parent link dependency
        if (typeof robot.links[parent_link].children === "undefined") {
            robot.links[parent_link].children = [x];
        }
        else {
            robot.links[parent_link].children.push(x);
        }
/* STENCIL END */ 
    }

}



