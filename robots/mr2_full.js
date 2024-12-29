//   CREATE ROBOT STRUCTURE

links_geom_imported = false;

//////////////////////////////////////////////////
/////     DEFINE ROBOT AND LINKS
//////////////////////////////////////////////////

// create robot data object
robot = {}; 

// give the robot a name
robot.name = "mr2";

// initialize start pose of robot in the world
robot.origin = {xyz: [0,0,0], rpy:[0,0,0]};

// specify base link of the robot; robot.origin is transform of world to the robot base
robot.base = "base";

// specify and create data objects for the links of the robot
robot.links = {
    "base": {},
    "clavicle_right": {},
    "clavicle_left": {},
    "shoulder_right": {},
    "shoulder_left": {},
    "upperarm_right": {},
    "upperarm_left": {},
    "forearm_right": {},
    "forearm_left": {}
};

//////////////////////////////////////////////////
/////     DEFINE JOINTS AND KINEMATIC HIERARCHY
//////////////////////////////////////////////////

// specify and create data objects for the joints of the robot
robot.joints = {};

// CLAVICLE JOINTS
robot.joints.clavicle_right = {parent:"base", child:"clavicle_right"};
robot.joints.clavicle_right.origin = {xyz: [0.3,0.4,0.0], rpy:[-Math.PI/2,0,0]};
robot.joints.clavicle_right.axis = [0.0,0.0,-1.0];

robot.joints.clavicle_left = {parent:"base", child:"clavicle_left"};
robot.joints.clavicle_left.origin = {xyz: [-0.3,0.4,0.0], rpy:[-Math.PI/2,0,0]};
robot.joints.clavicle_left.axis = [0.0,0.0,1.0];

// SHOULDER JOINTS
robot.joints.shoulder_right = {parent:"clavicle_right", child:"shoulder_right"};
robot.joints.shoulder_right.origin = {xyz: [0.0,-0.15,0.85], rpy:[Math.PI/2,0,0]};
robot.joints.shoulder_right.axis = [0.0,0.707,0.707];

robot.joints.shoulder_left = {parent:"clavicle_left", child:"shoulder_left"};
robot.joints.shoulder_left.origin = {xyz: [0.0,-0.15,0.85], rpy:[Math.PI/2,0,0]};
robot.joints.shoulder_left.axis = [0.0,0.707,0.707];

// UPPER ARM JOINTS
robot.joints.upperarm_right = {parent:"shoulder_right", child:"upperarm_right"};
robot.joints.upperarm_right.origin = {xyz: [0.0,0.0,0.7], rpy:[0,0,0]};
robot.joints.upperarm_right.axis = [0.0,1.0,0.0];

robot.joints.upperarm_left = {parent:"shoulder_left", child:"upperarm_left"};
robot.joints.upperarm_left.origin = {xyz: [0.0,0.0,0.7], rpy:[0,0,0]};
robot.joints.upperarm_left.axis = [0.0,1.0,0.0];

// FOREARM JOINTS
robot.joints.forearm_right = {parent:"upperarm_right", child:"forearm_right"};
robot.joints.forearm_right.origin = {xyz: [0.0,0.0,0.7], rpy:[0,0,0]};
robot.joints.forearm_right.axis = [1.0,0.0,0.0];

robot.joints.forearm_left = {parent:"upperarm_left", child:"forearm_left"};
robot.joints.forearm_left.origin = {xyz: [0.0,0.0,0.7], rpy:[0,0,0]};
robot.joints.forearm_left.axis = [1.0,0.0,0.0];

//////////////////////////////////////////////////
/////     DEFINE LINK threejs GEOMETRIES
//////////////////////////////////////////////////

links_geom = {};

// base
links_geom["base"] = new THREE.CubeGeometry( 1, 0.4, 1 );
links_geom["base"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0.2, 0) );

// CLAVICLE OR COLLARBONES
links_geom["clavicle_right"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["clavicle_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

links_geom["clavicle_left"] = new THREE.CubeGeometry( 0.3, 0.3, 1 );
links_geom["clavicle_left"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.5) );

// SHOULDER JOINTS
links_geom["shoulder_right"] = new THREE.CubeGeometry( 0.3, 0.3, 0.7 );
links_geom["shoulder_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.35) );

links_geom["shoulder_left"] = new THREE.CubeGeometry( 0.3, 0.3, 0.7 );
links_geom["shoulder_left"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.35) );

// UPPER ARMS
links_geom["upperarm_right"] = new THREE.CubeGeometry( 0.3, 0.3, 0.7 );
links_geom["upperarm_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.35) );

links_geom["upperarm_left"] = new THREE.CubeGeometry( 0.3, 0.3, 0.7 );
links_geom["upperarm_left"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.35) );

// LOWER ARMS
links_geom["forearm_right"] = new THREE.CubeGeometry( 0.3, 0.3, 0.5 );
links_geom["forearm_right"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.25) );

links_geom["forearm_left"] = new THREE.CubeGeometry( 0.3, 0.3, 0.5 );
links_geom["forearm_left"].applyMatrix( new THREE.Matrix4().makeTranslation(0, 0, 0.25) );

//////////////////////////////////////////////////
/////     AUTOMATICALLY SET ENDEFFECTOR OFFSET
//////////////////////////////////////////////////

// We want to measure the bounding box of forearm_right, then set the end-effector offset 
// so it’s exactly at the farthest end of that geometry (in local z direction). 
// We'll do the same for forearm_left if you like.

const tmpMeshRight = new THREE.Mesh(links_geom["forearm_right"]);
const bboxRight = new THREE.Box3().setFromObject(tmpMeshRight);
// measure how far along z
const lengthRight = bboxRight.max.z - bboxRight.min.z;

// set end effector data for the right side
robot.endeffector = {
    frame: "forearm_right",
    position: [
        [0], 
        [0], 
        [lengthRight], // place end effector at the far edge of the forearm
        [1]
    ]
};

// same logic for left if you want a second end effector
const tmpMeshLeft = new THREE.Mesh(links_geom["forearm_left"]);
const bboxLeft = new THREE.Box3().setFromObject(tmpMeshLeft);
const lengthLeft = bboxLeft.max.z - bboxLeft.min.z;

// create a second end effector if desired
robot.endeffector_2 = {
    frame: "forearm_left",
    position: [
        [0],
        [0],
        [lengthLeft],
        [1]
    ]
};
