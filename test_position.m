% reproPlannerCall.m
% Assumes you ran the Simulink model and created these workspace variables:
% graspState_ws, startConfig_ws, goalConfig_ws, ID_ws, boxPosition_ws, boxDim_ws, maxNumOfBoxes_ws

% --- Helper to extract last sample or value ---
getLast = @(x) (isstruct(x) && isfield(x,'signals') && isfield(x.signals,'values')) ...
    * 0; % placeholder to keep MATLAB editor happy

% Robust extractor for common To Workspace outputs (Array or timeseries or struct)
function val = lastValue(wsVar)
    if istable(wsVar)
        val = wsVarend ;
    elseif isstruct(wsVar) && isfield(wsVar,'signals') && isfield(wsVar.signals,'values')
        val = wsVar.signals.values(end, :);
    elseif isa(wsVar, 'timeseries')
        val = wsVar.Data(end, :);
    elseif isnumeric(wsVar)
        if isvector(wsVar)
            val = wsVar(end);
        else
            val = wsVar(:,end);
        end
    else
        % fallback: try last element
        try
            val = wsVar(end);
        catch
            error('Cannot extract last value from variable of type %s', class(wsVar));
        end
    end
end

% --- Extract values (modify variable names if you used different ones) ---
graspState = lastValue(out.graspState_ws);
startConfig = lastValue(out.startConfig_ws);
goalConfig = lastValue(out.goalConfig_ws);
ID = lastValue(out.ID_ws);
boxPosition = lastValue(out.boxPosition_ws);

% Display what we will send to the planner
disp('Inputs about to be passed to exampleHelperManipulatorRRT:');
disp('graspState ='); disp(graspState);
disp('startConfig ='); disp(startConfig);
disp('goalConfig ='); disp(goalConfig);
disp('ID ='); disp(ID);
disp('boxPosition ='); disp(boxPosition);
disp('boxDim ='); disp(boxDim);
disp('maxNumOfBoxes ='); disp(maxNumOfBoxes);

% --- Call the helper planner and catch the error for diagnostics ---
try
    [plan, numSamples] = exampleHelperManipulatorRRT(graspState, startConfig, goalConfig, ID, boxPosition, boxDim, maxNumOfBoxes);
    disp('Planner returned a plan (no error).');
    disp(['numSamples: ' num2str(numSamples)]);
catch ME
    disp('Planner threw an error. Full error info:');
    disp(ME.message);
    disp('Stack trace:');
    for k = 1:numel(ME.stack)
        fprintf('  %s (line %d) -> %s\n', ME.stack(k).file, ME.stack(k).line, ME.stack(k).name);
    end

    % If the cause is the manipulatorRRT collision error, show collision detail if available
    % manipulatorRRT often reports via robotics.manip.internal.error with a message.
    % You can also try calling manipulatorRRT directly if available (internal signature may differ).
end

% Optional: visualize the Cartesian goal pose in the 3D world by spawning a helper actor
% (This depends on how the example helper API exposes spawning; you can also manually inspect Add1.Out1 in the model.)
