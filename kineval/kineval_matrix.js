//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS 
//////////////////////////////////////////////////

function matrix_copy(m1) {
    // returns 2D array that is a copy of m1

    var mat = [];
    var i, j;

    for (i = 0; i < m1.length; i++) { // for each row of m1
        mat[i] = [];
        for (j = 0; j < m1[0].length; j++) { // for each column of m1
            mat[i][j] = m1[i][j];
        }
    }
    return mat;
}


// STENCIL: reference matrix code has the following functions:
//   matrix_multiply
//   matrix_transpose
//   matrix_pseudoinverse
//   matrix_invert_affine
//   vector_normalize
//   vector_cross
//   generate_identity
//   generate_translation_matrix
//   generate_rotation_matrix_X
//   generate_rotation_matrix_Y
//   generate_rotation_matrix_Z



// **** Function stencils are provided below, please uncomment and implement them ****//


function matrix_multiply(m1, m2) {
    // returns 2D array that is the result of m1*m2
    // let result = new Array(m1.length);
    var result = [];

    for (let row = 0; row < m1.length; row++) {
        result[row] = [];

        for (let col = 0; col < m2[0].length; col++) {
            result[row][col] = 0;

            for (let elt = 0; elt < m1[0].length; elt++) {
                result[row][col] += m1[row][elt] * m2[elt][col];
            }
        }
    }
    return result;
}

function matrix_transpose(m) {
    var result = [];
    var numRows = m.length;
    var numCols = m[0].length;

    for (var column = 0; column < numCols; column++) {
        result[column] = [];
        for (var row = 0; row < numRows; row++) {
            result[column][row] = 0;
        }
    }
    for (var row = 0; row < numRows; row++) {
        for (var column = 0; column < numCols; column++) {
            result[column][row] = m[row][column];
        }
    }
    return result;
}


function matrix_pseudoinverse(m) {
    // returns pseudoinverse of matrix m
    var m_transpose = matrix_transpose(m);
    var mT_m_square = matrix_multiply(m_transpose, m);
    var mT_m_inverse = numeric.inv(mT_m_square);
    return matrix_multiply(mT_m_inverse, m_transpose); // pseudoinverse

}

function matrix_invert_affine(m) {
}

function vector_normalize(v) {
    let a = v[0];
    let b = v[1];
    let c = v[2];
    let mag = Math.sqrt(a*a + b*b + c*c);
    return [a/mag, b/mag, c/mag];
}

function vector_cross(a,b) {
    return [
        a[1] * b[2] - a[2] * b[1],
        -a[0] * b[2] + a[2] * b[0],
        a[0] * b[1] - a[1] * b[0]
        ];
}

function generate_identity() { // 4x4 only
    return [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
        ];
}

function generate_translation_matrix(tx, ty, tz) {
    // returns 4-by-4 matrix as a 2D array
    return [
        [1, 0, 0, tx],
        [0, 1, 0, ty],
        [0, 0, 1, tz],
        [0, 0, 0, 1]
    ];

}

function generate_rotation_matrix_X(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    return [
        [1, 0, 0, 0],
        [0, Math.cos(angle), -Math.sin(angle), 0],
        [0, Math.sin(angle), Math.cos(angle), 0],
        [0, 0, 0, 1],
      ];

}

function generate_rotation_matrix_Y(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    return [
        [Math.cos(angle), 0, Math.sin(angle), 0],
        [0, 1, 0, 0],
        [-Math.sin(angle), 0, Math.cos(angle), 0],
        [0, 0, 0, 1],
      ];

}

function generate_rotation_matrix_Z(angle) {
    // returns 4-by-4 matrix as a 2D array, angle is in radians
    return [
        [Math.cos(angle), -Math.sin(angle), 0, 0],
        [Math.sin(angle), Math.cos(angle), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
      ];

}

function generate_net_rotation_matrix(roll, pitch, yaw) {
    var mat = generate_rotation_matrix_Z(yaw);
    mat = matrix_multiply(mat, generate_rotation_matrix_Y(pitch));
    mat = matrix_multiply(mat, generate_rotation_matrix_X(roll));
    return mat;
}