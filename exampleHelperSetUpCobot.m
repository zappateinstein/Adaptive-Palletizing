function rbt = exampleHelperSetUpCobot(baseHeight)
%EXAMPLEHELPERSETUPCOBOT helper function to build the robot rigid body tree

%   baseHeight (optional): height of the robot base above the floor [m].
%   Default 0.71 (0.70 m pedestal of PalletizeBoxesUsingCobot); the original
%   example scene (0.5 m pedestal) uses 0.51.


baseHeight = 0.71;
% Setup the rigidBodyTree
rbt = rigidBodyTree(MaxNumBodies=14,DataFormat="row");
rbt.BaseName = 'world';

% Define the base of the robot as a raised platform 
extBase = rigidBody('externalBase','MaxNumCollisions',1);
extBaseJnt = rigidBodyJoint('externalBaseJoint','fixed');
setFixedTransform(extBaseJnt,trvec2tform([0 0 baseHeight]));
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
addSubtree(rbt,'tool0',gripper)
end
