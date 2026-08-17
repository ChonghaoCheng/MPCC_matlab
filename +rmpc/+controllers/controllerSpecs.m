function specs = controllerSpecs(keys)
if ischar(keys) || isstring(keys)
    keys = cellstr(keys);
end

specs = repmat(struct('key', '', 'name', '', 'stateType', '', 'kind', '', 'solver', ''), 1, numel(keys));
for i = 1:numel(keys)
    key = char(keys{i});
    specs(i).key = key;
    switch key
        case 'euclidean_pid'
            specs(i).name = 'Euclidean PID';
            specs(i).stateType = 'quat';
            specs(i).kind = 'pid';
            specs(i).solver = 'euclideanPidTrackingControl';
        case {'liegroup_pid', 'pid'}
            specs(i).name = 'Lie Group PID';
            specs(i).stateType = 'quat';
            specs(i).kind = 'pid';
            specs(i).solver = 'pidTrackingControl';
        case 'euclidean_mpc'
            specs(i).name = 'Euclidean MPC';
            specs(i).stateType = 'quat';
            specs(i).kind = 'mpc';
            specs(i).solver = 'solveEuclideanMPCControl';
        case 'quaternion_mpc'
            specs(i).name = 'Quaternion MPC';
            specs(i).stateType = 'quat';
            specs(i).kind = 'mpc';
            specs(i).solver = 'solveQuaternionMPCControl';
        case {'se3_mpc', 'riemannian_mpc'}
            specs(i).name = 'SE(3) MPC';
            specs(i).stateType = 'lie';
            specs(i).kind = 'mpc';
            specs(i).solver = 'solveRiemannianMPCControl';
        case 'euclidean_mpcc'
            specs(i).name = 'Euclidean MPCC';
            specs(i).stateType = 'quat';
            specs(i).kind = 'mpcc';
            specs(i).solver = 'solveEuclideanMPCCTrackingQP';
        case 'riemannian_mpcc'
            specs(i).name = 'Riemannian MPCC';
            specs(i).stateType = 'lie';
            specs(i).kind = 'mpcc';
            specs(i).solver = 'solveRiemannianMPCCTrackingQP';
        otherwise
            error('Unknown controller key: %s', key);
    end
end
end
