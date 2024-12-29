/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | Graph Search Methods

    Stencil methods for student implementation of graph search algorithms.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Justin Lu, Chad Jenkins, and the 
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

function euclideanDist(x1, x2, y1, y2) {
    return Math.sqrt(Math.pow(Math.abs(x1 - x2), 2) + Math.pow(Math.abs(y1 - y2), 2))
}

function initSearchGraph() {

    // create the search queue
    visit_queue = [];

    // initialize search graph as 2D array over configuration space
    //   of 2D locations with specified spatial resolution
    G = [];
    for (iind=0, xpos=-2; xpos<7; iind++, xpos += eps) {
        G[iind] = [];
        for (jind=0, ypos=-2; ypos<7; jind++ ,ypos += eps) {
            G[iind][jind] = {
                i:iind, j:jind, // mapping to graph array
                x:xpos, y:ypos, // mapping to map coordinates
                parent:null, // pointer to parent in graph along motion path
                distance:10000, // distance to start via path through parent
                visited:false, // flag for whether the node has been visited
                priority:null, // visit priority based on fscore
                queued:false // flag for whether the node has been queued for visiting
            };
            // set priority regardless, priority inf
            // STENCIL: determine whether this graph node should be the start point for the search
            if (Math.abs(xpos - q_init[0]) <= eps/2.0 && Math.abs(ypos - q_init[1]) <= eps/2.0) {
                G[iind][jind].distance = 0;
                G[iind][jind].priority = euclideanDist(q_init[0], q_goal[0], q_init[1], q_goal[1]);
                G[iind][jind].visited = true;
                heap_insert(visit_queue, G[iind][jind]);
            }
            // abs >= eps/2
            // priority, distance, priority == f score, queued=true
        }
    }
}

function iterateGraphSearch() {


    // STENCIL: implement a single iteration of a graph search algorithm
    //   for A-star (or DFS, BFS, Greedy Best-First)
    //   An asynch timing mechanism is used instead of a for loop to avoid
    //   blocking and non-responsiveness in the browser.
    //
    //   Return "failed" if the search fails on this iteration.
    //   Return "succeeded" if the search succeeds on this iteration.
    //   Return "iterating" otherwise.
    //
    //   When search is complete ("failed" or "succeeded") set the global variable 
    //   search_iterate to false. 
    //
    //   Provided support functions:
    //
    //   testCollision - returns whether a given configuration is in collision
    //   drawHighlightedPathGraph - draws a path back to the start location
    //   draw_2D_configuration - draws a square at a given location

    // Pop cur_node from visited PQ, set visited to true, and draw2d
    var cur_node = heap_extract(visit_queue);
    cur_node.visited = true;
    // cur_node.queued = false;
    search_visited++; // Increment global visited counter
    draw_2D_configuration([cur_node.x, cur_node.y], "visited");

    // // Check null
    // if (cur_node === null) {
    //     search_iterate = false;
    //     drawHighlightedPathGraph(cur_node);
    //     return "failed";
    // }

    // // Check goal
    // if (Math.abs(cur_node.x - q_goal[0]) <= eps/2.0 && Math.abs(cur_node.y - q_goal[1]) <= eps/2.0) {
    //     search_iterate = false;
    //     drawHighlightedPathGraph(cur_node);
    //     return "succeeded";
    // }

    // Check all four adjacent neighboursÇ
    var neighbours = [
            [cur_node.i+1, cur_node.j],
            [cur_node.i-1, cur_node.j],
            [cur_node.i, cur_node.j+1], 
            [cur_node.i, cur_node.j-1]];

    for (let i = 0; i < neighbours.length; i++) {    
        var neighbour_idx = neighbours[i]; // Neighbour i,j coordinates
        var neighbour_node = G[neighbour_idx[0]][neighbour_idx[1]] // Node object

        // Check if i and j coordinates are within array bounds
        if (neighbour_idx[0] > 0 
                && neighbour_idx[0] < G.length 
                && neighbour_idx[1] > 0 
                && neighbour_idx[1] < G[0].length) {

            // Check if unvisited and not an obstacle
            if (!neighbour_node.queued && neighbour_node.visited == false && !testCollision([neighbour_node.x, neighbour_node.y])) {            
                
                // If valid, enqueue
                // neighbour_node.queued = true;
                // neighbour_node.priority = neighbour_node.distance + euclideanDist(neighbour_node.x, q_goal[0], neighbour_node.y, q_goal[1]);
                // heap_insert(visit_queue, neighbour_node);
                // draw_2D_configuration([neighbour_node.x, neighbour_node.y], "queued");
                neighbour_node.visited = true;
                // If a smaller distance is found:
                if (neighbour_node.distance > cur_node.distance + eps) {
                    // Update parent of neighbour to be current node
                    neighbour_node.parent = cur_node;
                    // Update distance to new reduced distance
                    neighbour_node.distance = cur_node.distance + eps;
                    // Set queued to true (new)
                    neighbour_node.queued = true; 
                    // Update F-score
                    neighbour_node.priority = neighbour_node.distance 
                            + euclideanDist(neighbour_node.x, q_goal[0], neighbour_node.y, q_goal[1]);
                    
                    draw_2D_configuration([neighbour_node.x, neighbour_node.y], "queued");
                    heap_insert(visit_queue, neighbour_node);
                }
            }
        }
    }

    // Check null
    if (cur_node === null) {
        search_iterate = false;
        drawHighlightedPathGraph(cur_node);
        return "failed";
    }

    // Check goal
    let distance_x = Math.pow(q_goal[0] - cur_node.x, 2);
    let distance_y = Math.pow(q_goal[1] - cur_node.y, 2);

    if (Math.sqrt(distance_x + distance_y) < eps/2.0) {
        search_iterate = false;
        drawHighlightedPathGraph(cur_node);
        return "succeeded";
    }

    return "iterating";
}

//////////////////////////////////////////////////
/////     MIN HEAP IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

    // STENCIL: implement min heap functions for graph search priority queue.
    //   These functions work use the 'priority' field for elements in graph.


    function heap_insert(heap, new_element) {
        var elementIdx = heap.length;
        var parentIdx = Math.floor((elementIdx - 1) / 2);
        heap.push(new_element);

        var sorted = (elementIdx <= 0) || (heap[parentIdx].priority <= heap[elementIdx].priority);
    
        while (!sorted) {
            // swap parent with cur element
            var temp = heap[parentIdx];
            heap[parentIdx] = heap[elementIdx];
            heap[elementIdx] = temp;
    
            // Update element and parent idx
            elementIdx = parentIdx;
            parentIdx = Math.floor((elementIdx - 1) / 2);
    
            // Re-evaluate heap condition
            sorted = (elementIdx <= 0) || (heap[parentIdx].priority <= heap[elementIdx].priority);
        }
    }
    
    // define extract function for min binary heap
    function heap_extract(heap) {
        if (heap.length === 0) return null; 
        
        var oldRoot = heap[0]; // old root for returning it
        var lastLeaf = heap.pop(); // last leaf
    
        if (heap.length > 0) {
            heap[0] = lastLeaf; // copy lastleaf to root
    
            var idx = 0;
            while (true) {
                var smallest = idx; // index of smallest (i.e parent of left and right)
                var left = (2 * idx + 1);
                var right = (2 * idx + 2);
            
                if (left < heap.length && heap[left].priority < heap[smallest].priority) {
                    smallest = left;
                }
    
                if (right < heap.length && heap[right].priority < heap[smallest].priority) {
                    smallest = right;
                }
    
                if (smallest !== idx) {
                    // Swap
                    var temp = heap[idx];
                    heap[idx] = heap[smallest];
                    heap[smallest] = temp;
                    idx = smallest; // c o n t i n u e (no heapify function)
                } 
    
                else {
                    break;
                }
            }
        }
    
        return oldRoot; // Return removed element
    }