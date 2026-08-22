function diagnoseSelfCollision(robot, goalConfig, collisionEnv, boxDim, graspState)
%DIAGNOSESELFCOLLISION Diagnostique une erreur "goal configuration is in self collision"
%
%   Usage :
%       load('debug_collision.mat')
%       diagnoseSelfCollision(robot, goalConfig, collisionEnv, boxDim, graspState)

    fprintf('=== Diagnostic auto-collision ===\n');
    fprintf('graspState = %d\n', graspState);
    fprintf('boxDim     = [%.4f  %.4f  %.4f]\n', boxDim(1), boxDim(2), boxDim(3));

    % Verification rapide des unites (UR10e ~ 1.3 m de portee)
    if any(boxDim > 0.6)
        fprintf(2, ['ATTENTION : boxDim semble trop grand pour des metres ' ...
                     '(UR10e ~1.3 m de portee). Verifiez mm vs m !\n']);
    end

    % Verification exhaustive de collision a goalConfig
    [isColliding, sepDist] = checkCollision(robot, goalConfig(:)', collisionEnv, ...
        'IgnoreSelfCollision','off', 'SkippedSelfCollisions','Parent', 'Exhaustive','on');

    fprintf('\nAuto-collision (robot vs robot)       : %d\n', isColliding(1));
    fprintf('Collision avec environnement (vs env) : %d\n', isColliding(2));

    % Identification des paires de corps en collision
    if isColliding(1)
        [i, j] = find(isnan(sepDist));
        pairs = unique(sort([i j], 2), 'rows');
        pairs = pairs(pairs(:,1) ~= pairs(:,2), :);

        names = [{'world/base'}, robot.BodyNames];
        fprintf('\nPaires de corps en collision :\n');
        fprintf('\nPaires de corps en collision :\n');
        maxNameIdx = length(names);
        for k = 1:size(pairs,1)
            id1 = pairs(k,1);
            id2 = pairs(k,2);
            
            % Protection contre les objets de l'environnement
            if id1 <= maxNameIdx, name1 = names{id1}; else, name1 = 'Objet_Environnement'; end
            if id2 <= maxNameIdx, name2 = names{id2}; else, name2 = 'Objet_Environnement'; end
            
            fprintf('  - %s  <-->  %s\n', name1, name2);
        end
    end

    % Verification visuelle
    figure('Name','Diagnostic collision');
    show(robot, goalConfig(:)', 'Collisions','on');
    title(sprintf('goalConfig | graspState = %d', graspState));
end