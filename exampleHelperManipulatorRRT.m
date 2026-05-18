function [fixedDimInterpPlan,numComputedSamples] = exampleHelperManipulatorRRT(graspState, startConfig,goalConfig,ID,boxPosition, boxDim, maxNumOfBoxes)
%EXAMPLEHELPERMANIPULATORRRT This helper sets up the collision environment 
% and uses manipulatorRRT for collision-free planning
%
%
coder.extrinsic("exampleHelperGetInitActorProp")
persistent baseLoc baseDim benchLoc benchDim palletDim palletLoc 
if isempty(palletDim)
    baseLoc = zeros(1,3);
    baseDim = zeros(1,3);
    benchLoc = zeros(1,3);
    benchDim = zeros(1,3);
    palletLoc = zeros(1,3);
    palletDim = zeros(1,3);
    [baseLoc,baseDim,benchLoc,benchDim,palletLoc,palletDim] = exampleHelperGetInitActorProp();
end

NUMDOF = 6; 
NUMPATHSAMPLES = 200;
robot = exampleHelperSetUpCobot();

% Attach the Box to the end-effector if the grasp is active
if graspState == true
    boxCenter = rigidBody('boxCenter','MaxNumCollisions',1);
    boxCenterJnt = rigidBodyJoint('boxCenterJoint','fixed');
    setFixedTransform(boxCenterJnt,trvec2tform([0 0 0])); % Overlap with current end-effector
    boxCenter.Joint = boxCenterJnt;
    cBox = collisionBox(1.1*boxDim,boxDim,1.1*boxDim);
    cBox.Pose = trvec2tform([0 -boxDim/2-0.1 0]);

    addCollision(boxCenter,cBox);
    addBody(robot,boxCenter,'4cup_assembly')
end

% Extract the updated collision environment for the planner
collisionEnv = exampleHelperGetCollisionEnvironment(ID,graspState,boxDim,boxPosition,maxNumOfBoxes,baseLoc,baseDim,benchDim,benchLoc,palletDim,palletLoc);

% Setup the planner
planner = manipulatorRRT(robot,collisionEnv);

planner.IgnoreSelfCollision = false;
planner.SkippedSelfCollisions = "Parent";
planner.MaxConnectionDistance = 0.3;
planner.ValidationDistance = 0.05;

% For repeatable results, seed the random number generator and store
% the current seed value.
prevseed = rng(0);

% Plan and interpolate.

[planOut, ~] = planner.plan(startConfig(:)',goalConfig(:)');
shortenedPlan = planner.shorten(planOut, 5);
numOfInterp = floor((NUMPATHSAMPLES - size(shortenedPlan,1))/(size(shortenedPlan,1)-1));
interpolatedPlan = planner.interpolate(shortenedPlan,numOfInterp);
numComputedSamples = size(interpolatedPlan,1);

% Move the samples into one of fixed dimension
fixedDimInterpPlan = zeros(NUMDOF, NUMPATHSAMPLES);
fixedDimInterpPlan(:,1:numComputedSamples) = interpolatedPlan(1:numComputedSamples,:)';

% Pad the end of the output matrix with the goal state if the
% interpolated plan doesn't have enough samples
if numComputedSamples < NUMPATHSAMPLES
    fixedDimInterpPlan(:,(numComputedSamples+1):NUMPATHSAMPLES) = repmat(goalConfig(:), 1, NUMPATHSAMPLES-numComputedSamples);
else
    fixedDimInterpPlan = fixedDimInterpPlan(:,1:NUMPATHSAMPLES);
end

% Restore the random number generator to the previously stored seed.
rng(prevseed);

end



