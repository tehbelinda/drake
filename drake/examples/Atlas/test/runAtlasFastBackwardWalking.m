function runAtlasFastBackwardWalking()
path_handle = addpathTemporary(fullfile(getDrakePath,'examples','Atlas'));
runAtlasWalking([], struct('navgoal', [-2;0;0;0;0;0],...
                                'max_num_steps', 10,...
                                'max_backward_step', 0.3,...
                                'nom_forward_step', 0.3));
end

% TIMEOUT 1500
