function collisionArraySubset = exampleHelperGetCollisionEnvironment(ID,graspState,boxDim,boxPosition,maxNumOfBoxes,baseLoc,baseDim,benchDim,benchLoc,palletDim,palletLoc)
%exampleHelperGetCollisionEnvironment Set up the collision environment for the planner

%   Copyright 2024 The MathWorks, Inc.

persistent collisionArray
coder.varsize('collisionArray',[1,50]);

% Initialize the collision environment with the existing actors
if isempty(collisionArray)
    
    collisionArray = cell(1,0);
    
    % Initialize a cell array of collisionBoxes placed away from the robot
    for i=coder.unroll(1:50)
        if i<=maxNumOfBoxes+3
        collisionArray{end+1} = collisionBox(boxDim(1),boxDim(2),boxDim(3),Pose=trvec2tform([3,3,0]));
        end
    end

    % Update the first three collision boxes with information from the
    % existing collision environment
    
    % Update the collisionBox for the bench
    collisionArray{1}.X = baseDim(1);
    collisionArray{1}.Y = baseDim(2);
    collisionArray{1}.Z = baseDim(3);
    collisionArray{1}.Pose = trvec2tform(baseLoc);

    collisionArray{2}.X = benchDim(1);
    collisionArray{2}.Y = benchDim(2);
    collisionArray{2}.Z = benchDim(3);
    collisionArray{2}.Pose = trvec2tform(benchLoc);

    % Update the collisionBox for the pallet
    collisionArray{3}.X = palletDim(1);
    collisionArray{3}.Y = palletDim(2);
    collisionArray{3}.Z = palletDim(3);
    collisionArray{3}.Pose = trvec2tform(palletLoc);
    
    % Update the collisionBox for box0
    collisionArray{4}.X = boxDim(1);
    collisionArray{4}.Y = boxDim(2);
    collisionArray{4}.Z = boxDim(3);
    collisionArray{4}.Pose = trvec2tform(boxPosition);
end

% After the box has been dropped off update the position of the dropped
% box and the spawned box

% As boxes are spawned, update the properties of the next collisionBox
if graspState == false
    % New box is spawned when the old box is dropped off i.e. when the graspState is false
    if ID < maxNumOfBoxes
        collisionArray{ID+4}.Pose = trvec2tform(boxPosition);
    end
    % Simultaneously update the position of the dropped box 
    % (There is no dropped box at the first time step)
    if ID>=1 
        droppedBoxPoseUpdate = exampleHelperPalletArrangement(ID-1,palletDim,palletLoc,boxDim);
        collisionArray{ID+3}.Pose = trvec2tform(droppedBoxPoseUpdate);
    end
end

% If the robot is holding the current box, the box is attached to the robot
% tree and temporarily removed from the environment
if graspState == true
    if ID < maxNumOfBoxes
        collisionArray{ID+4}.Pose = trvec2tform([3,3,0]);
    end
end

% Trim the unused boxes
collisionArraySubset = cell(1,0);
for i = coder.unroll(1:50)
    if i <= ID+4
        collisionArraySubset{end+1} = collisionArray{i};
    end
end

end