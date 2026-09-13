% Standalone test for rangefinder_sim.slx -- mirrors run_navtoolbox_model.m.
% Opens the model, subscribes to /rangefinder independently to verify output,
% runs the sim. Confirm this works before wiring in distance_sensor_bridge.py.
%
% Optionally set `stopTime` (a string, e.g. '60') in the workspace before
% running this script to override the model's default StopTime.

cd(fileparts(mfilename('fullpath')));
load_system('rangefinder_sim.slx');
mdl = 'rangefinder_sim';

logfile = 'range_capture.log';
fid = fopen(logfile, 'w'); fclose(fid);

verifyNode = ros2node('/range_verifier');
rangeSub = ros2subscriber(verifyNode, '/rangefinder', 'sensor_msgs/Range', @(msg) logRangeMsg(msg, logfile));

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
clear rangeSub verifyNode;
fprintf('DONE\n');

function logRangeMsg(msg, logfile)
    fid = fopen(logfile, 'a');
    fprintf(fid, 'range=%.3f min=%.3f max=%.3f frame_id=%s\n', ...
        msg.range, msg.min_range, msg.max_range, char(msg.header.frame_id));
    fclose(fid);
end
