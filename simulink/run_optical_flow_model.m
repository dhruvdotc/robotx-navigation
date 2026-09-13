% Standalone test for optical_flow_sim.slx -- mirrors run_navtoolbox_model.m.
% Opens the model, subscribes to /optical_flow independently to verify
% output, runs the sim. Confirm this works before wiring in
% optical_flow_bridge.py.
%
% Optionally set `stopTime` (a string, e.g. '60') in the workspace before
% running this script to override the model's default StopTime.

cd(fileparts(mfilename('fullpath')));
load_system('optical_flow_sim.slx');
mdl = 'optical_flow_sim';

logfile = 'optical_flow_capture.log';
fid = fopen(logfile, 'w'); fclose(fid);

verifyNode = ros2node('/optical_flow_verifier');
flowSub = ros2subscriber(verifyNode, '/optical_flow', 'geometry_msgs/TwistStamped', @(msg) logFlowMsg(msg, logfile));

if ~exist('stopTime', 'var')
    stopTime = '30';
end

try
    simOut = sim(mdl, 'StopTime', stopTime);
    fprintf('SIM_COMPLETED_OK\n');
catch e
    fprintf('SIM_ERROR: %s\n', e.message);
    for i = 1:numel(e.cause)
        fprintf('CAUSE %d: %s\n', i, e.cause{i}.message);
    end
end

pause(2);
clear flowSub verifyNode;
fprintf('DONE\n');

function logFlowMsg(msg, logfile)
    fid = fopen(logfile, 'a');
    % angular.z carries quality (see build_optical_flow_model.m header) --
    % not a real angular velocity.
    fprintf(fid, 'flow_x=%.4f flow_y=%.4f quality=%.0f frame_id=%s\n', ...
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z, char(msg.header.frame_id));
    fclose(fid);
end
