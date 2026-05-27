boxDim = 0.2;
maxNumOfBoxes = 20;
boxPosition = [-0.65 0.65 0.6];
% Setup the rigidBodyTree
rbt = rigidBodyTree(MaxNumBodies=14,DataFormat="row");
rbt.BaseName = 'world';

% Define the base of the robot as a raised platform 
extBase = rigidBody('externalBase','MaxNumCollisions',1);
if(coder.target('MATLAB'))
    addVisual(extBase,"box",[0.25,0.25,0.5],trvec2tform([0,0,-0.25]));
end
addCollision(extBase,collisionBox(0.25,0.25,0.5),trvec2tform([0,0,-0.25]));
extBaseJnt = rigidBodyJoint('externalBaseJoint','fixed');
setFixedTransform(extBaseJnt,trvec2tform([0 0 0.5]));
extBase.Joint = extBaseJnt;

% Add base to the tree
addBody(rbt,extBase,'world')

% Load the robot rigidBodyTree
urrobot = loadrobot("universalUR10e",DataFormat="row");

% Add the robot tree to the base 
addSubtree(rbt,'externalBase',urrobot)

% Load the gripper rigidBodyTree
gripper = loadrobot("robotiqEPick4CupVacuumAssembly", Dataformat="row");

% Add the gripper to the tree
addSubtree(rbt,'tool0',gripper);
open_system("PalletizeBoxesUsingCobot.slx");