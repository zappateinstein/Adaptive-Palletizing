function [fixedDimInterpPlan,numComputedSamples] = exampleHelperManipulatorRRT(graspState, startConfig,goalConfig,ID,boxPosition, boxDim, boxWeight, maxNumOfBoxes)
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
% Robot base sits on top of the scene's pedestal (collBase) + 1 cm, so this
% works for both the 0.5 m (original) and 0.7 m pedestal scenes.
robot = exampleHelperSetUpCobot(baseLoc(3) + baseDim(3)/2 + 0.01);
% Attach the Box to the end-effector if the grasp is active
if graspState == true
    boxCenter = rigidBody('boxCenter','MaxNumCollisions',1);
    boxCenterJnt = rigidBodyJoint('boxCenterJoint','fixed');
    setFixedTransform(boxCenterJnt,trvec2tform([0 0 0])); % Overlap with current end-effector
    boxCenter.Joint = boxCenterJnt;
    L = boxDim(1);
    W = boxDim(2);
    H = boxDim(3);
    dGripperGap = 0.11;
    d = dGripperGap + (H/2);
    clearance = 0.01;
    cBox = collisionBox(1.1*L, H, 1.1*W);
    %maxLateralOffset = 0.15;  % meters — cap so offset can't grow unbounded with width
    %lateralOffset = min(W/2 + 0.1, maxLateralOffset);
    cBox.Pose = trvec2tform([0 -(d - clearance) 0]);
    addCollision(boxCenter,cBox);
    m = double(boxWeight);
    boxCenter.Mass = m;
    boxCenter.CenterOfMass = [0 -d 0];        % matches the collision box offset
    Ic = [m/12*(W^2+H^2), m/12*(L^2+W^2), m/12*(L^2+H^2)]; % solid-box inertia
    boxCenter.Inertia = [Ic(1) + m * d^2, Ic(2), Ic(3) + m * d^2, 0, 0, 0];
 
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

% DIAGNOSTIC: if start or goal is in self- or world-collision, record what
% collides (saved to base workspace as rrtFailure) before the planner errors out.
% checkCollision orders sepDist rows/cols as robot.BodyNames first, base LAST,
% then (world check) one column per collisionEnv object.
names = [robot.BodyNames {robot.BaseName}];
N = robot.NumBodies;
envNames = cell(1,numel(collisionEnv));
envNames(1:min(3,end)) = {'pedestal(collBase)','bench(collBench)','pallet(collPallet)'};
for e = 4:numel(collisionEnv), envNames{e} = sprintf('box%d', e-4); end
cfgs = {startConfig(:)', goalConfig(:)'}; tags = {'start','goal'};
for c = 1:2
    if any(~isfinite(cfgs{c}))
        info = struct('which',tags{c},'ID',double(ID),'graspState',logical(graspState), ...
            'kind','NaN/Inf config from IK', ...
            'startConfigDeg',rad2deg(startConfig(:)'),'goalConfigDeg',rad2deg(goalConfig(:)'), ...
            'boxPosition',double(boxPosition(:)'),'boxDim',double(boxDim(:)'),'boxWeight',double(boxWeight));
        assignin('base','rrtFailure',info);
        error('PalletDiag:nanConfig', ...
            'Box ID %d, graspState=%d: %s config from the IK block contains NaN/Inf ([%s] deg). Details in base variable rrtFailure.', ...
            info.ID, info.graspState, tags{c}, num2str(rad2deg(cfgs{c}),4));
    end
end
for c = 1:2
    [isColl, sepDist] = checkCollision(robot, cfgs{c}, collisionEnv, ...
        SkippedSelfCollisions="parent", Exhaustive="on");
    if any(isColl)
        pairs = {};
        [r,k] = find(isnan(sepDist(:,1:N+1)));          % robot <-> robot
        for p = 1:numel(r)
            if r(p) < k(p), pairs{end+1} = sprintf('%s <-> %s', names{r(p)}, names{k(p)}); end %#ok<AGROW>
        end
        [r,k] = find(isnan(sepDist(:,N+2:end)));        % robot <-> world
        for p = 1:numel(r)
            pairs{end+1} = sprintf('%s <-> %s @[%s]', names{r(p)}, envNames{k(p)}, ...
                num2str(tform2trvec(collisionEnv{k(p)}.Pose),3)); %#ok<AGROW>
        end
        kinds = {'self','world'};
        info = struct('which',tags{c},'ID',double(ID),'graspState',logical(graspState), ...
            'kind',strjoin(kinds(logical(isColl)),'+'), ...
            'startConfigDeg',rad2deg(startConfig(:)'),'goalConfigDeg',rad2deg(goalConfig(:)'), ...
            'eePos',tform2trvec(getTransform(robot,cfgs{c},'4cup_assembly')),'collidingPairs',{pairs});
        assignin('base','rrtFailure',info);
        error('PalletDiag:collision', ...
            'Box ID %d, graspState=%d: %s config in %s collision (%s). Details in base variable rrtFailure.', ...
            info.ID, info.graspState, tags{c}, info.kind, strjoin(pairs, '; '));
    end
end

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