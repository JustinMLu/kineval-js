
/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | RRT motion planning

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

//////////////////////////////////////////////////
/////     RRT MOTION PLANNER
//////////////////////////////////////////////////

// STUDENT: 
// compute motion plan and output into robot_path array 
// elements of robot_path are vertices based on tree structure in tree_init() 
// motion planner assumes collision checking by kineval.poseIsCollision()

/* KE 2 : Notes:
   - Distance computation needs to consider modulo for joint angles
   - robot_path[] should be used as desireds for controls
   - Add visualization of configuration for current sample
   - Add cubic spline interpolation
   - Add hook for random configuration
   - Display planning iteration number in UI
*/

/*
STUDENT: reference code has functions for:

*/

kineval.planMotionRRTConnect = function motionPlanningRRTConnect() {

    // exit function if RRT is not implemented
    //   start by uncommenting kineval.robotRRTPlannerInit 
    if (typeof kineval.robotRRTPlannerInit === 'undefined') return;

    if ((kineval.params.update_motion_plan) && (!kineval.params.generating_motion_plan)) {
        kineval.robotRRTPlannerInit();
        kineval.params.generating_motion_plan = true;
        kineval.params.update_motion_plan = false;
        kineval.params.planner_state = "initializing";
    }

    if (kineval.params.generating_motion_plan) {
        rrt_result = robot_rrt_planner_iterate();
        if (rrt_result === "reached") {
            kineval.params.update_motion_plan = false; // KE T needed due to slight timing issue
            kineval.params.generating_motion_plan = false;
            textbar.innerHTML = "planner execution complete";
            kineval.params.planner_state = "complete";
        }
        else kineval.params.planner_state = "searching";
    }

    else if (kineval.params.update_motion_plan_traversal||kineval.params.persist_motion_plan_traversal) {

        if (kineval.params.persist_motion_plan_traversal) {
            kineval.motion_plan_traversal_index = (kineval.motion_plan_traversal_index+1)%kineval.motion_plan.length;
            textbar.innerHTML = "traversing planned motion trajectory";
        }
        else
            kineval.params.update_motion_plan_traversal = false;

        // set robot pose from entry in planned robot path
        robot.origin.xyz = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[0],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[1],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[2]
        ];

        robot.origin.rpy = [
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[3],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[4],
            kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[5]
        ];

        // KE 2 : need to move q_names into a global parameter
        for (x in robot.joints) {
            robot.joints[x].angle = kineval.motion_plan[kineval.motion_plan_traversal_index].vertex[q_names[x]];
        }

    }
}

var is_T_a_start = true;
    // STENCIL: uncomment and complete initialization function
kineval.robotRRTPlannerInit = function robot_rrt_planner_init() {

    // form configuration from base location and joint angles
    q_start_config = [
        robot.origin.xyz[0],
        robot.origin.xyz[1],
        robot.origin.xyz[2],
        robot.origin.rpy[0],
        robot.origin.rpy[1],
        robot.origin.rpy[2]
    ];

    q_names = {};  // store mapping between joint names and q DOFs
    q_index = [];  // store mapping between joint names and q DOFs

    for (x in robot.joints) {
        q_names[x] = q_start_config.length;
        q_index[q_start_config.length] = x;
        q_start_config = q_start_config.concat(robot.joints[x].angle);
    }

    // set goal configuration as the zero configuration
    var i; 
    q_goal_config = new Array(q_start_config.length);
    for (i=0;i<q_goal_config.length;i++) q_goal_config[i] = 0;

    T_a = tree_init(q_start_config);
    T_b = tree_init(q_goal_config);

    // flag to continue rrt iterations
    rrt_iterate = true;
    rrt_iter_count = 0;

    // make sure the rrt iterations are not running faster than animation update
    cur_time = Date.now();
}



function robot_rrt_planner_iterate() {
 
    rrt_alg = 1;  // 0: basic rrt (OPTIONAL), 1: rrt_connect (REQUIRED)

    if (rrt_iterate && (Date.now()-cur_time > 10)) {
        cur_time = Date.now();

    // STENCIL: implement single rrt iteration here. an asynch timing mechanism 
    //   is used instead of a for loop to avoid blocking and non-responsiveness 
    //   in the browser.
    //
    //   once plan is found, highlight vertices of found path by:
    //     tree.vertices[i].vertex[j].geom.material.color = {r:1,g:0,b:0};
    //
    //   provided support functions:
    //
    //   kineval.poseIsCollision - returns if a configuration is in collision
    //   tree_init - creates a tree of configurations
    //   tree_add_vertex - adds and displays new configuration vertex for a tree
    //   tree_add_edge - adds and displays new tree edge between configurations


        if (!(rrt_extend(T_a, random_config()) === "failed")) {
            var q_temp = T_a.vertices[T_a.newest].vertex;

            if (rrt_connect(T_b, q_temp) === "succeeded") {

                var path_Ta = find_path(T_a, T_a.vertices[0], T_a.vertices[T_a.newest]);

                var path_Tb = find_path(T_b, T_b.vertices[0], T_b.vertices[T_b.newest]);

                handlePath(path_Ta, path_Tb);
                rrt_iterate = false;
                return "reached"; // yikes
            }
        }
        // Swap trees (1)
        var temp = T_a;
        T_a = T_b;
        T_b = temp;

        // Swap trees (2)
        is_T_a_start = !is_T_a_start;
        rrt_iter_count++; 
    }
    return false;
}


function handlePath(path_Ta, path_Tb) {
    kineval.motion_plan = [];
    var idx = 0;
    
    if (is_T_a_start) {

        for (let i = 0; i < path_Ta.length; i++) {
            kineval.motion_plan[idx] = path_Ta[i];
            idx++;
        }

        kineval.motion_plan.reverse();

        for (let i = 0; i < path_Tb.length; i++) {
            kineval.motion_plan[idx] = path_Tb[i];
            idx++;
        
        }
    }
    else {
        for (let i = 0; i < path_Tb.length; i++) {
            kineval.motion_plan[idx] = path_Tb[i];
            idx++;
        }

        kineval.motion_plan.reverse();

        for (let i = 0; i < path_Ta.length; i++) {
            kineval.motion_plan[idx] = path_Ta[i];
            idx++;
        }
    }
}

//////////////////////////////////////////////////
/////     STENCIL SUPPORT FUNCTIONS
//////////////////////////////////////////////////

function tree_init(q) {

    // create tree object
    var tree = {};

    // initialize with vertex for given configuration
    tree.vertices = [];
    tree.vertices[0] = {};
    tree.vertices[0].vertex = q;
    tree.vertices[0].edges = [];

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(tree.vertices[0]);

    // maintain index of newest vertex added to tree
    tree.newest = 0;

    return tree;
}

function tree_add_vertex(tree,q) {


    // create new vertex object for tree with given configuration and no edges
    var new_vertex = {};
    new_vertex.edges = [];
    new_vertex.vertex = q;

    // create rendering geometry for base location of vertex configuration
    add_config_origin_indicator_geom(new_vertex);

    // maintain index of newest vertex added to tree
    tree.vertices.push(new_vertex);
    tree.newest = tree.vertices.length - 1;
}

function add_config_origin_indicator_geom(vertex) {

    // create a threejs rendering geometry for the base location of a configuration
    // assumes base origin location for configuration is first 3 elements 
    // assumes vertex is from tree and includes vertex field with configuration

    temp_geom = new THREE.CubeGeometry(0.1,0.1,0.1);
    temp_material = new THREE.MeshLambertMaterial( { color: 0xffff00, transparent: true, opacity: 0.7 } );
    temp_mesh = new THREE.Mesh(temp_geom, temp_material);
    temp_mesh.position.x = vertex.vertex[0];
    temp_mesh.position.y = vertex.vertex[1];
    temp_mesh.position.z = vertex.vertex[2];
    scene.add(temp_mesh);
    vertex.geom = temp_mesh;
}


function tree_add_edge(tree,q1_idx,q2_idx) {

    // add edge to first vertex as pointer to second vertex
    tree.vertices[q1_idx].edges.push(tree.vertices[q2_idx]);

    // add edge to second vertex as pointer to first vertex
    tree.vertices[q2_idx].edges.push(tree.vertices[q1_idx]);

    // can draw edge here, but not doing so to save rendering computation
}


/////     RRT IMPLEMENTATION FUNCTIONS

    // STENCIL: implement RRT-Connect functions here, such as:
    //   rrt_extend
    //   rrt_connect
    //   random_config
    //   new_config
    //   nearest_neighbor
    //   normalize_joint_state
    //   find_path
    //   path_dfs

var eps = 0.75;

// DONE
function getDistance(v1, v2) {
    let totalDistance = 0;
    const limitIndex = 2;
    v1.forEach((item, index) => {
        let difference = item - v2[index];
        totalDistance += difference * difference;
    });
    return Math.sqrt(totalDistance);
}

// DONE
function rrt_extend(T, q) {
    
    var q_nearest = nearest_neighbor(T, q);

    var q_new_cfg = new_config(q,q_nearest.vertex); // object w/ config & pose collision bool
    if (!q_new_cfg[1]) {
        var q_new = q_new_cfg[0];

        tree_add_vertex(T, q_new);
        tree_add_edge(T, T.vertices.length-1, q_nearest.index);

        if (getDistance(q_new, q) < eps/2) {
            return "succeeded";
        }
        else {
            return "extended";
        }
    }
    return "failed";
}

// DONE
function rrt_connect(T, q) {

    var result = "extended";
    while (result === "extended") {
        result = rrt_extend(T, q);
    }
    return result;
}

function randomInRange(num1, num2) {
    return num1 + Math.random() * (num2 - num1); // I love you copilot
}

function random_config_bad() {
    // console.log("q_start_config length: ", q_start_config.length);
    // only generate for x, z, and p
                        // x, y, z, r, p, y
    var random_vals = [randomInRange(-10, 10),0,randomInRange(-10,10),0,randomInRange(0,2*Math.PI),0];

    for (joint in robot.joints) {
        random_vals.push(randomInRange(0, 2*Math.PI));
    }
    return random_vals;
}


// MUCH better with joint limits
function random_config() {
    var randomConfig = []; // Should be 6 + robot.joints.length

    for (let i = 0; i < q_start_config.length; i++) {
        randomConfig[i] = 0;
    }

    var upper = 0;
    var lower = 0;

    for (i = 0; i < randomConfig.length; i++) {
        var idx = q_index[i]; // Use for robot.joints

        // IF A JOINT AND NOT X,Y,Z,R,P,Y:
        if (i > 5) {
            if (robot.joints[idx].type == 'revolute' || robot.joints[idx].type =='prismatic') {
                upper = robot.joints[idx].limit.upper;
                lower = robot.joints[idx].limit.lower;
            }
            // Set continuous joints to pi
            else if (robot.joints[q_index[i]].type == 'continuous') {
                upper = 2*Math.PI;
                lower = 0;
            }
            // Set undefined type to 0
            else {
                upper = 0;
                lower = 0;
            }
        }
        else {
            switch (i) {
                case 0:
                case 2:
                    upper = robot_boundary[1][i];
                    lower = robot_boundary[0][i];
                    break;
                case 4:
                    lower = 0;
                    upper = 2*Math.PI; 
                    break;
                
                    default:
                    lower = 0;
                    upper = 0;
                    break;
            }   
        }
        randomConfig[i] = lower+Math.random()*(upper-lower);
    } 
    
    return randomConfig;
}

// DONE
function new_config(q, q_nearest) {
    var q_new = [];
    var l2_norm = 0;
    var i;

    for (let i = 0; i < q.length; i++) {
        var delta = q[i] - q_nearest[i];
        l2_norm += delta * delta;
    }
    l2_norm = Math.sqrt(l2_norm); // Euclidean dist.

    
    for (let i = 0; i < q.length; i++) {
        var delta = q[i] - q_nearest[i];
        q_new[i] = q_nearest[i] + (delta / l2_norm) * eps;
    }
    
    result = [q_new, kineval.poseIsCollision(q_new)]; 
    // result.poseIsCollision = kineval.poseIsCollision(q_new); 
    return result;
}

// DONE
function nearest_neighbor(T, q) {
    var nearestNeighbour = {}; 
    var minDist = 9007199254740991;

    for (let i = 0; i < T.vertices.length; i++) {

        var curDist = getDistance(q, T.vertices[i].vertex);

        if (curDist < minDist) {
            minDist = curDist;

            nearestNeighbour.vertex = T.vertices[i].vertex;
            nearestNeighbour.index = i;
        }
    }
    return nearestNeighbour;
}

// DONE
function find_path(T, q_init, q_goal) {
    var dfsPath = path_dfs(T, q_init, q_goal);
    // Make the path red
    for (let i = 0; i < dfsPath.length; i++) {
        dfsPath[i].geom.material.color = {r:1,g:0,b:0};
    }
    return dfsPath;
}

// DONE(?)
function path_dfs(T, v, q_goal) {
    // Perform recursive DFS on T

    if (v === q_goal) return [v]; // Return path if goal found
    v.visited = true; // Mark as visited

    // Traverse adjacent 
    for (let i = 0; i < v.edges.length; i++) {
        if (!v.edges[i].visited) {
            let result = path_dfs(T, v.edges[i], q_goal);
            if (result) return result.concat(v); // Return path + cur v if found
        }
    }

    return false; // Return false if no path found thru vertex v
}








