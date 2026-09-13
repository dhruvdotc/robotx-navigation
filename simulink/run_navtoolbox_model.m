cd(fileparts(mfilename('fullpath')));
load_system('gps_sim_navtoolbox.slx');
mdl = 'gps_sim_navtoolbox';

% Relative to this file's directory (set by the cd() above) so this works
% identically whether MATLAB is running on Linux or natively on Windows --
% an absolute Linux path like /root/... silently fails to open on Windows.
logfile = 'fix_capture_navtoolbox.log';
fid = fopen(logfile, 'w'); fclose(fid);

verifyNode = ros2node('/fix_verifier_navtoolbox');
fixSub = ros2subscriber(verifyNode, '/fix', 'sensor_msgs/NavSatFix', @(msg) logFixMsg(msg, logfile));

% stopTime can be preset in the workspace before running this script (e.g.
% from a -batch invocation: `stopTime='240'; run('run_navtoolbox_model.m')`)
% to cover a full course flight instead of the 25s smoke-test default.
if ~exist('stopTime', 'var')
    stopTime = '25';
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
clear fixSub verifyNode;
fprintf('DONE\n');

function logFixMsg(msg, logfile)
    fid = fopen(logfile, 'a');
    fprintf(fid, 'lat=%.8f lon=%.8f alt=%.3f status=%d frame_id=%s\n', ...
        msg.latitude, msg.longitude, msg.altitude, msg.status.status, char(msg.header.frame_id));
    fclose(fid);
end
