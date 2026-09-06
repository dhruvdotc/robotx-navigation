cd(fileparts(mfilename('fullpath')));
mdl = 'gps_sim_navtoolbox';
close_system(mdl, 0);
new_system(mdl);
open_system(mdl);

DATUM = [-35.363262, 149.165237, 0.0];

%% Subscribe to real Gazebo odometry
add_block('ros2lib/Subscribe', [mdl '/OdomSub'], 'Position', [40 40 200 130]);
set_param([mdl '/OdomSub'], 'topicSource', 'Specify your own');
set_param([mdl '/OdomSub'], 'topic', '/model/iris_uav/odometry');
set_param([mdl '/OdomSub'], 'messageType', 'nav_msgs/Odometry');
set_param([mdl '/OdomSub'], 'sampleTime', '0.2');

%% Extract position (N,E) and velocity (N,E) from odometry
add_block('simulink/Signal Routing/Bus Selector', [mdl '/PosVelSel'], 'Position', [280 20 400 160]);
set_param([mdl '/PosVelSel'], 'OutputSignals', ...
    'pose.pose.position.x,pose.pose.position.y,twist.twist.linear.x,twist.twist.linear.y');

add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/MakeVectors'], 'Position', [440 20 560 160]);
mkVecCode = [ ...
"function [pos, vel] = make_vectors(north, east, vnorth, veast)", ...
"    pos = [north, east, 0];", ...
"    vel = [vnorth, veast, 0];", ...
"end" ...
];
rt3 = sfroot;
mkVecChart = rt3.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/MakeVectors']);
mkVecChart.Script = strjoin(mkVecCode, newline);

%% Digital Clock for the fix-mode state machine
add_block('simulink/Sources/Digital Clock', [mdl '/Clk'], 'Position', [40 260 100 290]);
set_param([mdl '/Clk'], 'SampleTime', '0.2');

%% MATLAB Function: fix-mode selector (1=RTK-Int,2=RTK-Float,3=DGPS,4=Standard)
%
% Realism improvement (2026-09-05, UNTESTED -- no MATLAB reachable during this
% investigation; ported from the live-verified equivalent in
% simulation/mock_fix_publisher.py). Real GPS multipath/partial-sky-occlusion
% off a ~0.5m buoy/gate structure at antenna height is a real effect, and the
% RobotX courses are dense with exactly this kind of structure.
%
% First cut of this (just biasing the Markov-chain weights near a buoy,
% still gated by the ~8s mean dwell timer) was live-tested in
% mock_fix_publisher.py and found EMPIRICALLY BROKEN: on a short (~67s),
% buoy-dense course there are only ~8-9 state transitions total, so a couple
% of unlucky/lucky probabilistic draws dominate the average and can make
% near-buoy accuracy come out *better* than open-water by pure chance, even
% though the bias fires correctly at the right moments. See
% simulink/README.md's "Realism improvements" section for the live data.
%
% Fix ported here: bypass the probabilistic dwell entirely and force mode=4
% (Standard, the worst of the four gpsSensor configs) deterministically
% whenever within 8m of a real buoy/gate -- immediate, distance-driven, no
% dependence on transition timing. The Markov dwell still runs unmodified for
% ambient open-water behaviour. (mock_fix_publisher.py's fix is a continuous
% sigma multiplier instead of a hard override, because its noise is
% synthesized per-tick; that route isn't available here since each
% gpsSensor's accuracy is fixed at model-build time, not a runtime input --
% a deterministic override to the worst-configured instance is the closest
% equivalent without adding a 5th gpsSensor block.) Anyone regenerating the
% model with this script should re-verify against mock_fix_publisher.py's
% live-tested behaviour before trusting it (or better: add a --diag-style
% scripted check to run_navtoolbox_model.m first).
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/ModeSel'], 'Position', [280 280 480 380]);
modeFcnCode = [ ...
"function [mode, status] = mode_selector(clk, pos)", ...
"    persistent state_idx state_until", ...
"    if isempty(state_idx)", ...
"        state_idx = int32(2);", ...
"        state_until = clk + (-8.0 * log(rand()));", ...
"    end", ...
"    % Course 1 buoy/gate (north_m, east_m) positions -- gates 1-3 + light buoy.", ...
"    buoys = [1.25 10.0; -1.25 10.0; 1.25 25.0; -1.25 25.0; 1.25 40.0; -1.25 40.0; 0.0 50.0];", ...
"    d = min(hypot(buoys(:,1) - pos(1), buoys(:,2) - pos(2)));", ...
"    near_buoy = d <= 8.0;", ...
"    weights = [0.05, 0.70, 0.20, 0.05];", ...
"    statuses = int8([2, 2, 1, 0]);", ...
"    if clk >= state_until", ...
"        idx_others = int32(setdiff([1,2,3,4], double(state_idx)));", ...
"        w_others = weights(idx_others);", ...
"        total = sum(w_others);", ...
"        rr = rand()*total;", ...
"        acc = 0; chosen = idx_others(end);", ...
"        for k = 1:numel(idx_others)", ...
"            acc = acc + w_others(k);", ...
"            if rr <= acc", ...
"                chosen = idx_others(k);", ...
"                break;", ...
"            end", ...
"        end", ...
"        state_idx = chosen;", ...
"        state_until = clk + (-8.0 * log(rand()));", ...
"    end", ...
"    if near_buoy", ...
"        mode = int32(4);", ...
"        status = int8(0);", ...
"    else", ...
"        mode = state_idx;", ...
"        status = statuses(state_idx);", ...
"    end", ...
"end" ...
];
rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/ModeSel']);
chart.Script = strjoin(modeFcnCode, newline);

%% Four gpsSensor instances: RTK-Int, RTK-Float, DGPS, Standard
gpsCfg = { ...
    'GPS_RTKInt',  0.01, 0.02, 0.02; ...
    'GPS_RTKFloat', 0.05, 0.08, 0.05; ...
    'GPS_DGPS',    0.40, 0.60, 0.20; ...
    'GPS_Std',     1.50, 3.00, 0.50 ...
};
yBase = 20;
for i = 1:4
    name = [mdl '/' gpsCfg{i,1}];
    add_block('sensorgpslib/GPS', name, 'Position', [560 yBase 700 yBase+90]);
    set_param(name, 'PositionInputFormat', 'Local');
    set_param(name, 'ReferenceLocation', mat2str(DATUM));
    set_param(name, 'HorizontalPositionAccuracy', num2str(gpsCfg{i,2}));
    set_param(name, 'VerticalPositionAccuracy', num2str(gpsCfg{i,3}));
    set_param(name, 'VelocityAccuracy', num2str(gpsCfg{i,4}));
    yBase = yBase + 130;
end

%% MATLAB Function to select active LLA output based on mode
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/LLASwitch'], 'Position', [780 100 900 260]);
llaSwitchCode = [ ...
"function lla = select_lla(mode, lla1, lla2, lla3, lla4)", ...
"    switch mode", ...
"        case 1", ...
"            lla = lla1;", ...
"        case 2", ...
"            lla = lla2;", ...
"        case 3", ...
"            lla = lla3;", ...
"        otherwise", ...
"            lla = lla4;", ...
"    end", ...
"end" ...
];
rt2 = sfroot;
llaChart = rt2.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/LLASwitch']);
llaChart.Script = strjoin(llaSwitchCode, newline);

%% NavSatStatus sub-message
add_block('ros2lib/Blank Message', [mdl '/BlankStatus'], 'Position', [900 40 1000 70]);
set_param([mdl '/BlankStatus'], 'entityType', 'sensor_msgs/NavSatStatus');
set_param([mdl '/BlankStatus'], 'messageType', 'sensor_msgs/NavSatStatus');
add_block('simulink/Sources/Constant', [mdl '/ServiceGPS'], 'Position', [900 100 950 130]);
set_param([mdl '/ServiceGPS'], 'Value', 'uint16(1)');
add_block('simulink/Signal Routing/Bus Assignment', [mdl '/AssignStatus'], 'Position', [1040 40 1140 110]);
set_param([mdl '/AssignStatus'], 'AssignedSignals', 'status,service');

%% Split LLA vector back into lat/lon/alt scalars
add_block('simulink/Signal Routing/Demux', [mdl '/LLADemux'], 'Position', [860 200 880 280]);
set_param([mdl '/LLADemux'], 'Outputs', '3');

%% NavSatFix outer message
add_block('ros2lib/Blank Message', [mdl '/BlankFix'], 'Position', [900 300 1000 330]);
set_param([mdl '/BlankFix'], 'entityType', 'sensor_msgs/NavSatFix');
set_param([mdl '/BlankFix'], 'messageType', 'sensor_msgs/NavSatFix');

add_block('simulink/Signal Routing/Bus Assignment', [mdl '/AssignFix'], 'Position', [1200 250 1320 420]);
set_param([mdl '/AssignFix'], 'AssignedSignals', 'latitude,longitude,altitude,status');

%% Header Assignment + Publish
add_block('ros2lib/Header Assignment', [mdl '/HdrAssign'], 'Position', [1380 300 1500 350]);
set_param([mdl '/HdrAssign'], 'SetFrameID', 'on');
set_param([mdl '/HdrAssign'], 'FrameID', 'gps');
set_param([mdl '/HdrAssign'], 'InsertTimeStamp', 'on');

add_block('ros2lib/Publish', [mdl '/FixPub'], 'Position', [1560 300 1660 350]);
set_param([mdl '/FixPub'], 'topic', '/fix');
set_param([mdl '/FixPub'], 'messageType', 'sensor_msgs/NavSatFix');

%% Wiring
odomPh = get_param([mdl '/OdomSub'], 'PortHandles');
selPh  = get_param([mdl '/PosVelSel'], 'PortHandles');
add_line(mdl, odomPh.Outport(2), selPh.Inport(1), 'autorouting', 'on');

mkVecPh = get_param([mdl '/MakeVectors'], 'PortHandles');
add_line(mdl, selPh.Outport(1), mkVecPh.Inport(1), 'autorouting', 'on');
add_line(mdl, selPh.Outport(2), mkVecPh.Inport(2), 'autorouting', 'on');
add_line(mdl, selPh.Outport(3), mkVecPh.Inport(3), 'autorouting', 'on');
add_line(mdl, selPh.Outport(4), mkVecPh.Inport(4), 'autorouting', 'on');

clkPh = get_param([mdl '/Clk'], 'PortHandles');
modePh = get_param([mdl '/ModeSel'], 'PortHandles');
add_line(mdl, clkPh.Outport(1), modePh.Inport(1), 'autorouting', 'on');
% pos (from MakeVectors, already computed for the gpsSensor blocks below) feeds
% ModeSel's buoy-proximity distance check -- see the ModeSel script comment.
add_line(mdl, mkVecPh.Outport(1), modePh.Inport(2), 'autorouting', 'on');

swPh = get_param([mdl '/LLASwitch'], 'PortHandles');
gpsNames = {'GPS_RTKInt','GPS_RTKFloat','GPS_DGPS','GPS_Std'};
for i = 1:4
    gpsPh = get_param([mdl '/' gpsNames{i}], 'PortHandles');
    add_line(mdl, mkVecPh.Outport(1), gpsPh.Inport(1), 'autorouting', 'on');
    add_line(mdl, mkVecPh.Outport(2), gpsPh.Inport(2), 'autorouting', 'on');
    add_line(mdl, gpsPh.Outport(1), swPh.Inport(i+1), 'autorouting', 'on');
end
add_line(mdl, modePh.Outport(1), swPh.Inport(1), 'autorouting', 'on');

blankStatusPh = get_param([mdl '/BlankStatus'], 'PortHandles');
svcPh = get_param([mdl '/ServiceGPS'], 'PortHandles');
asgStatusPh = get_param([mdl '/AssignStatus'], 'PortHandles');
add_line(mdl, blankStatusPh.Outport(1), asgStatusPh.Inport(1), 'autorouting', 'on');
add_line(mdl, modePh.Outport(2), asgStatusPh.Inport(2), 'autorouting', 'on');
add_line(mdl, svcPh.Outport(1), asgStatusPh.Inport(3), 'autorouting', 'on');

demuxPh = get_param([mdl '/LLADemux'], 'PortHandles');
add_line(mdl, swPh.Outport(1), demuxPh.Inport(1), 'autorouting', 'on');

blankFixPh = get_param([mdl '/BlankFix'], 'PortHandles');
asgFixPh = get_param([mdl '/AssignFix'], 'PortHandles');
add_line(mdl, blankFixPh.Outport(1), asgFixPh.Inport(1), 'autorouting', 'on');
add_line(mdl, demuxPh.Outport(1), asgFixPh.Inport(2), 'autorouting', 'on');
add_line(mdl, demuxPh.Outport(2), asgFixPh.Inport(3), 'autorouting', 'on');
add_line(mdl, demuxPh.Outport(3), asgFixPh.Inport(4), 'autorouting', 'on');
add_line(mdl, asgStatusPh.Outport(1), asgFixPh.Inport(5), 'autorouting', 'on');

hdrPh = get_param([mdl '/HdrAssign'], 'PortHandles');
add_line(mdl, asgFixPh.Outport(1), hdrPh.Inport(1), 'autorouting', 'on');
pubPh = get_param([mdl '/FixPub'], 'PortHandles');
add_line(mdl, hdrPh.Outport(1), pubPh.Inport(1), 'autorouting', 'on');

set_param(mdl, 'StopTime', '25');
set_param(mdl, 'EnablePacing', 'on');
set_param(mdl, 'PacingRate', '1');
save_system(mdl, 'gps_sim_navtoolbox.slx');
fprintf('MODEL_BUILT_OK\n');
