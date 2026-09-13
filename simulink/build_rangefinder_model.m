% Builds rangefinder_sim.slx: a downward-facing laser-rangefinder noise model
% against real Gazebo odometry, following the exact same architecture as
% build_gps_model.m (Subscribe real state -> add realistic sensor noise ->
% Publish on its own ROS 2 topic -> a Python bridge injects it into ArduPilot
% as a real MAVLink message).
%
% STATUS: UNTESTED. Built the same way gps_sim_navtoolbox.slx originally was
% (no MATLAB available to run it at authoring time) -- verify with
% run_rangefinder_model.m before trusting it, the same way gps_sim_navtoolbox
% needed a live run before its /fix output was confirmed correct.
%
% ArduPilot side (see rangefinder_mav_params.parm):
%   RNGFND1_TYPE  = 10  (MAVLink backend, confirmed against
%                        ~/ardupilot/libraries/AP_RangeFinder/AP_RangeFinder.h)
%   RNGFND1_ORIENT = 25 (ROTATION_PITCH_270 = facing straight down, confirmed
%                        against ~/ardupilot/libraries/AP_Math/rotations.h)
% The DISTANCE_SENSOR message's `orientation` field must match RNGFND1_ORIENT
% exactly or AP_RangeFinder_MAVLink::handle_msg silently drops every reading
% (see that file's handle_msg -- it checks `packet.orientation == orientation()`
% before accepting anything, no error either way).
%
% Realism modeled here (not just Gaussian noise on a perfect number):
%   - Range limiting: real laser rangefinders (e.g. Lidar-Lite/Garmin-class)
%     lose lock past ~25-40m; this model reports "no signal" (quality=0,
%     range clamped to max) beyond RANGE_MAX_M.
%   - Water-surface unreliability: this is a nadir sensor over open water for
%     RobotX. Real laser rangefinders perform poorly over calm water --
%     specular reflection scatters the beam away from the receiver rather
%     than back at it, unlike a diffuse solid surface. Modeled as a per-sample
%     dropout probability (DROPOUT_PROB) independent of distance-to-buoy --
%     unlike the GPS model's buoy-proximity degradation, this effect is
%     ambient over water generally, not something that gets better near
%     structures (if anything, a solid buoy hull directly underneath would
%     return a genuinely GOOD reading, the opposite of the GPS multipath
%     story -- not modeled here as it would require knowing the vehicle is
%     directly over a buoy, not just near one).

cd(fileparts(mfilename('fullpath')));
mdl = 'rangefinder_sim';
close_system(mdl, 0);
new_system(mdl);
open_system(mdl);

RANGE_MAX_M = 40.0;
RANGE_MIN_M = 0.06;
NOISE_STDDEV_FRAC = 0.01;   % 1% of true range
NOISE_STDDEV_BASE_M = 0.02; % + 2cm base noise floor
DROPOUT_PROB = 0.15;        % ambient over-water dropout chance per sample

%% Subscribe to real Gazebo odometry (same topic the GPS model uses)
add_block('ros2lib/Subscribe', [mdl '/OdomSub'], 'Position', [40 40 200 130]);
set_param([mdl '/OdomSub'], 'topicSource', 'Specify your own');
set_param([mdl '/OdomSub'], 'topic', '/model/iris_uav/odometry');
set_param([mdl '/OdomSub'], 'messageType', 'nav_msgs/Odometry');
set_param([mdl '/OdomSub'], 'sampleTime', '0.1');

%% Extract true height (Z) -- approximates AGL over a flat water plane at Z=0
add_block('simulink/Signal Routing/Bus Selector', [mdl '/HeightSel'], 'Position', [280 60 400 120]);
set_param([mdl '/HeightSel'], 'OutputSignals', 'pose.pose.position.z');

%% Digital Clock feeds the dropout RNG so each sample gets an independent draw
add_block('simulink/Sources/Digital Clock', [mdl '/Clk'], 'Position', [40 260 100 290]);
set_param([mdl '/Clk'], 'SampleTime', '0.1');

%% Sensor model: range limiting + noise + water-dropout quality
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/RangeModel'], 'Position', [280 220 480 340]);
rangeModelCode = [ ...
"function [range_m, quality] = range_model(clk, true_height_m)", ...
sprintf("    RANGE_MAX_M = %.4f;", RANGE_MAX_M), ...
sprintf("    RANGE_MIN_M = %.4f;", RANGE_MIN_M), ...
sprintf("    NOISE_STDDEV_FRAC = %.4f;", NOISE_STDDEV_FRAC), ...
sprintf("    NOISE_STDDEV_BASE_M = %.4f;", NOISE_STDDEV_BASE_M), ...
sprintf("    DROPOUT_PROB = %.4f;", DROPOUT_PROB), ...
"    true_height_m = max(true_height_m, 0);", ...
"    noise_sigma = NOISE_STDDEV_BASE_M + NOISE_STDDEV_FRAC * true_height_m;", ...
"    noisy = true_height_m + noise_sigma * randn();", ...
"    if rand() < DROPOUT_PROB || true_height_m > RANGE_MAX_M || true_height_m < RANGE_MIN_M", ...
"        % Dropout / out-of-range: sensor reports max range with zero quality,", ...
"        % matching how real rangefinders signal 'no valid return'.", ...
"        range_m = single(RANGE_MAX_M);", ...
"        quality = uint8(0);", ...
"    else", ...
"        range_m = single(min(max(noisy, RANGE_MIN_M), RANGE_MAX_M));", ...
"        quality = uint8(min(255, max(0, 200 + round(randn() * 20))));", ...
"    end", ...
"end" ...
];
rt1 = sfroot;
rangeChart = rt1.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/RangeModel']);
rangeChart.Script = strjoin(rangeModelCode, newline);

%% sensor_msgs/Range message assembly
add_block('ros2lib/Blank Message', [mdl '/BlankRange'], 'Position', [560 40 660 70]);
set_param([mdl '/BlankRange'], 'entityType', 'sensor_msgs/Range');
set_param([mdl '/BlankRange'], 'messageType', 'sensor_msgs/Range');

add_block('simulink/Sources/Constant', [mdl '/RadiationType'], 'Position', [560 120 620 150]);
set_param([mdl '/RadiationType'], 'Value', 'uint8(1)'); % 1 = INFRARED (laser-class)
add_block('simulink/Sources/Constant', [mdl '/FieldOfView'], 'Position', [560 170 620 200]);
set_param([mdl '/FieldOfView'], 'Value', 'single(0.05)'); % ~3 degree beam, radians
add_block('simulink/Sources/Constant', [mdl '/MinRangeConst'], 'Position', [560 220 620 250]);
set_param([mdl '/MinRangeConst'], 'Value', sprintf('single(%.4f)', RANGE_MIN_M));
add_block('simulink/Sources/Constant', [mdl '/MaxRangeConst'], 'Position', [560 270 620 300]);
set_param([mdl '/MaxRangeConst'], 'Value', sprintf('single(%.4f)', RANGE_MAX_M));

add_block('simulink/Signal Routing/Bus Assignment', [mdl '/AssignRange'], 'Position', [720 60 840 320]);
set_param([mdl '/AssignRange'], 'AssignedSignals', 'radiation_type,field_of_view,min_range,max_range,range');

%% Header Assignment + Publish
add_block('ros2lib/Header Assignment', [mdl '/HdrAssign'], 'Position', [900 140 1020 190]);
set_param([mdl '/HdrAssign'], 'SetFrameID', 'on');
set_param([mdl '/HdrAssign'], 'FrameID', 'rangefinder');
set_param([mdl '/HdrAssign'], 'InsertTimeStamp', 'on');

add_block('ros2lib/Publish', [mdl '/RangePub'], 'Position', [1080 140 1180 190]);
set_param([mdl '/RangePub'], 'topicSource', 'Specify your own');
set_param([mdl '/RangePub'], 'topic', '/rangefinder');
set_param([mdl '/RangePub'], 'messageType', 'sensor_msgs/Range');

%% Wiring
odomPh = get_param([mdl '/OdomSub'], 'PortHandles');
selPh  = get_param([mdl '/HeightSel'], 'PortHandles');
add_line(mdl, odomPh.Outport(2), selPh.Inport(1), 'autorouting', 'on'); % port 2 = Msg (confirmed 2026-09-13)

clkPh = get_param([mdl '/Clk'], 'PortHandles');
rmPh  = get_param([mdl '/RangeModel'], 'PortHandles');
add_line(mdl, clkPh.Outport(1), rmPh.Inport(1), 'autorouting', 'on');
add_line(mdl, selPh.Outport(1), rmPh.Inport(2), 'autorouting', 'on');

blankPh = get_param([mdl '/BlankRange'], 'PortHandles');
asgPh   = get_param([mdl '/AssignRange'], 'PortHandles');
radPh   = get_param([mdl '/RadiationType'], 'PortHandles');
fovPh   = get_param([mdl '/FieldOfView'], 'PortHandles');
minPh   = get_param([mdl '/MinRangeConst'], 'PortHandles');
maxPh   = get_param([mdl '/MaxRangeConst'], 'PortHandles');
add_line(mdl, blankPh.Outport(1), asgPh.Inport(1), 'autorouting', 'on');
add_line(mdl, radPh.Outport(1), asgPh.Inport(2), 'autorouting', 'on');
add_line(mdl, fovPh.Outport(1), asgPh.Inport(3), 'autorouting', 'on');
add_line(mdl, minPh.Outport(1), asgPh.Inport(4), 'autorouting', 'on');
add_line(mdl, maxPh.Outport(1), asgPh.Inport(5), 'autorouting', 'on');
add_line(mdl, rmPh.Outport(1), asgPh.Inport(6), 'autorouting', 'on');

hdrPh = get_param([mdl '/HdrAssign'], 'PortHandles');
add_line(mdl, asgPh.Outport(1), hdrPh.Inport(1), 'autorouting', 'on');
pubPh = get_param([mdl '/RangePub'], 'PortHandles');
add_line(mdl, hdrPh.Outport(1), pubPh.Inport(1), 'autorouting', 'on');

% Note: `quality` from RangeModel (rmPh.Outport(2)) is intentionally not
% wired into the sensor_msgs/Range message -- that message has no quality
% field. It exists so a future consumer (or a logged/scoped signal during
% verification) can see when the model is simulating a dropout vs. a real
% reading, matching the pattern used for debugging gps_sim_navtoolbox.

set_param(mdl, 'StopTime', '30');
set_param(mdl, 'EnablePacing', 'on');
set_param(mdl, 'PacingRate', '1');
save_system(mdl, 'rangefinder_sim.slx');
fprintf('MODEL_BUILT_OK\n');
