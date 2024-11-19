/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | RRT Methods

    Stencil methods for student implementation of RRT-based search.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

var q_new;

function swapTrees() {
    // Global access to T_a and T_b
    let swap = T_a;
    T_a = T_b;
    T_b = swap;
}

function iterateRRT() {


    // STENCIL: implement a single iteration of an RRT algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree
}

function iterateRRTConnect() {


    // STENCIL: implement a single iteration of an RRT-Connect algorithm.
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.

    // RETURN VALUES:
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "extended" otherwise.

    //   PROVIDED SUPPORT FUNCTIONS:
    //   testCollision - returns whether a given configuration is in collision
    //   insertTreeVertex - adds and displays new configuration vertex for a tree
    //   insertTreeEdge - adds and displays new tree edge between configurations
    //   drawHighlightedPath - renders a highlighted path in a tree

    if (!(extendRRT(T_a, randomConfig()) == "failed")) {
        if (connectRRT(T_b, q_new) == "succeeded") {
            
            var path_Ta = dfsPath(T_a);
            var path_Tb = dfsPath(T_b);
            
            drawHighlightedPath(path_Ta);
            drawHighlightedPath(path_Tb);
            
            search_iterate = false;
            // NOt "reached" i guess :/
            return "succeeded";
        }
    }

    // Swap trees
    swapTrees();
    return "extended";
}

function iterateRRTStar() {

}

//////////////////////////////////////////////////
/////     RRT IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement RRT-Connect functions here, such as:
    //   extendRRT
    //   connectRRT
    //   randomConfig
    //   newConfig
    //   findNearestNeighbor
    //   dfsPath

function extendRRT(T, q) {
    
    var q_nearest = findNearestNeighbour(q, T);
    q_new = newConfig(q, q_nearest.vertex);

    if (!testCollision(q_new)) {

        insertTreeVertex(T, q_new);
        insertTreeEdge(T, q_nearest.index, T.newest); // tree, q1_idx, q2_idx
        T.vertices[T.newest].parent = q_nearest.index;

        // Now use 'distance' to check if q_new == q
        var dx = Math.abs(q[0] - q_new[0]);
        var dy = Math.abs(q[1] - q_new[1]);

        if (dx < eps/2 && dy < eps/2) {
            return "succeeded";
        }
        else {
            return "extended";
        }
    }
    else {
        return "failed";
    }

}

function connectRRT(T, q) {
    var s = extendRRT(T, q);
    while (s === "extended") { // until not "advanced"
        s = extendRRT(T, q);
    }
    return s;
}

function randomConfig() {
    return [(Math.random() * 9) - 2, (Math.random() * 9) - 2];
}

function newConfig(q, q_nearest) {
    var dx = q[0] - q_nearest[0];
    var dy = q[1] - q_nearest[1];

    // Euclidean distance between q and q_nearest
    var dist = Math.sqrt(Math.pow(dx, 2) + Math.pow(dy, 2));

    // Unit vectors so we can take an EPSILON-sized step
    var dx_unit = dx / dist;
    var dy_unit = dy / dist;
    
    // Return that new point
    return [q_nearest[0]+dx_unit*eps, q_nearest[1]+dy_unit*eps];
    
}

function findNearestNeighbour(q, T) {
    // Chad!
    var minDist = 9007199254740991

    var nearestNeighbour = {};
    for (let i = 0; i < T.vertices.length; i++) {
        var dx = Math.pow(T.vertices[i].vertex[0] - q[0], 2);
        var dy = Math.pow(T.vertices[i].vertex[1] - q[1], 2);
        
        var curDist = Math.sqrt(dx + dy);
        if (curDist < minDist) {
            minDist = curDist;
            nearestNeighbour.vertex = T.vertices[i].vertex;
            nearestNeighbour.index = i;
        }
    }
    return nearestNeighbour;
}

function dfsPath(T) {
    // Initialize the result list
    var result = [];

    // Start with the newest vertex in T
    var cur_node = T.vertices[T.newest];

    // Continue looping until the flag is set to false
    while (true) {
        // Add the current node to the result list
        result.push(cur_node);
        // Move to the next node in the graph by taking the first edge
        cur_node = cur_node.edges[0];

        // Check if the current node's first vertex component matches the initial OR goal states
        if ((cur_node.vertex[0] == q_init[0] || cur_node.vertex[0] == q_goal[0]) && (cur_node.vertex[1] == q_init[1] || cur_node.vertex[1] == q_goal[1])) {
           
            // If both components match, add the final node to the result and exit the loop
            result.push(cur_node);
            break;
            
        }
    }

    // Return the path as the result list
    return result;
}
