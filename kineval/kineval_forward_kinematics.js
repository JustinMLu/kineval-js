
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics () { 

    if (typeof kineval.buildFKTransforms === 'undefined') {
        textbar.innerHTML = "forward kinematics not implemented";
        return;
    }

    // STENCIL: call kineval.buildFKTransforms();
    kineval.buildFKTransforms();
}

    // STENCIL: implement buildFKTransforms, which kicks off
    //   a recursive traversal over links and 
    //   joints starting from base, using following functions: 
    //     traverseFKBase
    //     traverseFKLink
    //     traverseFKJoint
    //
    // To use the keyboard interface, assign the global variables 
    //   "robot_heading" and "robot_lateral", 
    //   which represent the z-axis (heading) and x-axis (lateral) 
    //   of the robot's base in its own reference frame, 
    //   transformed into the world coordinates.
    // The axes should be represented in unit vector form 
    //   as 4x1 homogenous matrices

    //
    // if geometries are imported and using ROS coordinates (e.g., fetch),
    //   coordinate conversion is needed for kineval/threejs coordinates:
    //

    // FKBase --> FKLink --> FKJoint --> FKLink...
    kineval.buildFKTransforms = function buildFKTransforms() {
        // Initialize matrix stack
        var mStack = [];

        mStack.push(generate_identity());

        // Start recursive traversal
        kineval.traverseFKBase(mStack);
    }
    
    kineval.traverseFKBase = function traverseFKBase(mStack) {  
        // Helpful vars!
        var x_base = robot.origin.xyz[0],
            y_base = robot.origin.xyz[1],
            z_base = robot.origin.xyz[2],
            roll_base = robot.origin.rpy[0],
            pitch_base = robot.origin.rpy[1],
            yaw_base = robot.origin.rpy[2];
            

         // Create base transform --> compose rotational and translational effects
        var transform = generate_translation_matrix(x_base, y_base, z_base);
        transform = matrix_multiply(transform, 
            generate_net_rotation_matrix(roll_base, pitch_base, yaw_base));
        
        // ROS coordinate conversion
        if (robot.links_geom_imported) {
            var ros_conversion = matrix_multiply(generate_rotation_matrix_Y(-Math.PI/2), generate_rotation_matrix_X(-Math.PI/2));

            transform = matrix_multiply(transform, ros_conversion);
        }
        
        
        // Update base, push to matrix stack
        robot.origin.xform = transform; 
        mStack.push(transform);
        
        // X and Z coordinates let it move!
        if (robot.links_geom_imported) {
            robot_heading = matrix_multiply(robot.origin.xform, [[1], [0], [1], [1]]); // z
            robot_lateral = matrix_multiply(robot.origin.xform, [[0], [1], [0], [1]]); // x
        }
        else {
            robot_heading = matrix_multiply(robot.origin.xform, [[0], [0], [1], [1]]); // z
            robot_lateral = matrix_multiply(robot.origin.xform, [[1], [0], [0], [1]]); // x
        }
    
        // Begin recursive traversal starting at base link
        kineval.traverseFKLink(mStack, robot.base);
    }
    
    
    kineval.traverseFKLink = function traverseFKLink(mStack, link) {
        // Update actual link with transform
        robot.links[link].xform = mStack[mStack.length - 1];

        // If child joints exist, recurse over all child joints
        if (robot.links[link].children) {
            for (let i = 0; i < robot.links[link].children.length; i++) {
                kineval.traverseFKJoint(mStack, robot.links[link].children[i]);
            }
        }
        // Pop transform matrix
        mStack.pop();
    }
    
    kineval.traverseFKJoint = function traverseFKJoint(mStack, joint) {
        // Create clone of top-of-stack transform matrix
        var topClone = mStack[mStack.length - 1];

        // Helpful vars!
        var x_t = robot.joints[joint].origin.xyz[0];
            y_t = robot.joints[joint].origin.xyz[1],
            z_t = robot.joints[joint].origin.xyz[2],
            roll_t = robot.joints[joint].origin.rpy[0],
            pitch_t = robot.joints[joint].origin.rpy[1],
            yaw_t = robot.joints[joint].origin.rpy[2];

        // Create transform --> compose rotational and translational effects
        var transform = generate_translation_matrix(x_t, y_t, z_t);
        transform = matrix_multiply(transform, 
            generate_net_rotation_matrix(roll_t, pitch_t, yaw_t));

        // Generate joint translations that vary by type
        if (robot.joints[joint].type == "prismatic") {

            var axisAngle0 = robot.joints[joint].angle * robot.joints[joint].axis[0];
            var axisAngle1 = robot.joints[joint].angle * robot.joints[joint].axis[1];
            var axisAngle2 = robot.joints[joint].angle * robot.joints[joint].axis[2];
            
            var axisAngle0 = robot.joints[joint].angle * robot.joints[joint].axis[0];
            var axisAngle1 = robot.joints[joint].angle * robot.joints[joint].axis[1];
            var axisAngle2 = robot.joints[joint].angle * robot.joints[joint].axis[2];

            var prismatic_trans = generate_translation_matrix(axisAngle0, axisAngle1, axisAngle2);
            transform = matrix_multiply(transform, prismatic_trans);
        }

        else if (robot.joints[joint].type == "revolute" || robot.joints[joint].type == "continuous" || robot.joints[joint].type == undefined) {
            var quat = kineval.quaternionToRotationMatrix(
                kineval.unitQuaternionFromAxisAngle(robot.joints[joint].axis, robot.joints[joint].angle)  // Use unit quaternion
            );
            transform = matrix_multiply(transform, quat);
        }

        // Apply transform to top-of-stack transform matrix
        var result = matrix_multiply(topClone, transform);

        // Update actual joint, push to stack
        robot.joints[joint].xform = result;
        mStack.push(result);

        // If child link exists, recursively traverse links
        if(robot.joints[joint].child !== null) {
            kineval.traverseFKLink(mStack, robot.joints[joint].child);
        }
    }