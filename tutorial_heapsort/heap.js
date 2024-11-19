/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
  
    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/


// create empty object 
minheaper = {}; 

// define insert function for min binary heap
function minheap_insert(heap, new_element) {

    // STENCIL: implement your min binary heap insert operation
    // Get indices
    var elementIdx = heap.length;
    var parentIdx = Math.floor((elementIdx - 1) / 2);
    heap.push(new_element);

    // Heap condition = true if new element added as root, or if new element is <= parent elements
    var sorted = (elementIdx <= 0) || (heap[parentIdx] <= heap[elementIdx]);

    while (!sorted) {
        // swap parent with cur element
        var temp = heap[parentIdx];
        heap[parentIdx] = heap[elementIdx];
        heap[elementIdx] = temp;

        // Update element and parent idx
        elementIdx = parentIdx;
        parentIdx = Math.floor((elementIdx - 1) / 2);

        // Re-evaluate heap condition
        sorted = (elementIdx <= 0) || (heap[parentIdx] <= heap[elementIdx]);
    }
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/


// define extract function for min binary heap
function minheap_extract(heap) {
    if (heap.length === 0) return null; 

    var oldRoot = heap[0]; // old root for returning it
    var lastLeaf = heap.pop(); // last leaf

    if (heap.length > 0) {
        heap[0] = lastLeaf; // copy lastleaf to root

        var idx = 0;
        while (true) {
            var smallest = idx;
            var left = (2 * idx + 1);
            var right = (2 * idx + 2);
        
            if (left < heap.length && heap[left] < heap[smallest]) {
                smallest = left;
            }

            if (right < heap.length && heap[right] < heap[smallest]) {
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

// assign extract function within minheaper object

    // STENCIL: ensure extract method is within minheaper object
minheaper.extract = minheap_extract;





