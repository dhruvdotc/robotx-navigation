% Builds optical_flow_sim.slx: a downward-facing optical-flow sensor noise
% model against real Gazebo odometry. Same architecture as build_gps_model.m
% and build_rangefinder_model.m.
%
% STATUS: UNTESTED -- see build_rangefinder_model.m's header for what that
% means and how to verify (run_optical_flow_model.m is the equivalent here).
%
% ArduPilot side (see optical_flow_mav_params.parm):
%   FLOW_TYPE = 5  (MAVLINK backend, confirmed against
%                   ~/ardupilot/libraries/AP_OpticalFlow/AP_OpticalFlow.h)
% AP_OpticalFlow_MAV::handle_msg (confirmed by reading that file) prefers the
% OPTICAL_FLOW message's flow_rate_x/flow_rate_y fields (rad/s) over the
% legacy flow_x/flow_y integer fields whenever flow_rate is nonzero -- this
% model and its bridge (optical_flow_bridge.py) always populate flow_rate,
% never the legacy fields.
%
% There is no standard ROS message for optical flow. This model publishes
% geometry_msgs/TwistStamped as a practical stand-in (no custom .msg build
% needed): linear.x/linear.y carry flow_rate_x/flow_rate_y in rad/s,
% angular.z carries quality (0-255, stuffed into a float field) --
% angular.z is otherwise unused and chosen over linear.z specifically so it
% doesn't look like a real flow-rate component. This is a deliberate
% non-standard repurposing, not a real Twist -- documented here and in
% simulink/README.md so nobody mistakes it for actual angular velocity.
%
% Realism modeled here:
%   - True flow rate is derived from real body-frame-ish velocity divided by
%     height (small-angle optical-flow approximation: angular_rate ~
%     velocity / height), so it genuinely varies with both how fast the
%     drone moves AND how high it is -- not just noise on a constant.
%   - Altitude-dependent quality: real optical flow degrades both very close
%     to the ground (motion blur, out of focus) and very high up (surface
%     features become too small/sparse to track) -- modeled as a U-shaped
%     quality curve peaking around FLOW_OPTIMAL_HEIGHT_M.
%   - Ambient over-water quality penalty: unlike a textured solid surface,
%     open water has few stable trackable visual features. RobotX operates
%     entirely over water, so this is modeled as a constant quality
%     multiplier (WATER_QUALITY_FACTOR) rather than a proximity effect (no
%     buoy makes open water suddenly more textured for a downward camera).

cd(fileparts(mfilename('fullpath')));
mdl = 'optical_flow_sim';
close_system(mdl, 0);
new_system(mdl);
open_system(mdl);

FLOW_OPTIMAL_HEIGHT_M = 3.0;
FLOW_MIN_USABLE_HEIGHT_M = 0.3;
FLOW_MAX_USABLE_HEIGHT_M = 15.0;
WATER_QUALITY_FACTOR = 0.55;   % open water vs. a textured solid surface
NOISE_STDDEV_FRAC = 0.05;      % 5% of true flow rate

%% Subscribe to real Gazebo odometry
add_block('ros2lib/Subscribe', [mdl '/OdomSub'], 'Position', [40 40 200 130]);
set_param([mdl '/OdomSub'], 'topicSource', 'Specify your own');
set_param([mdl '/OdomSub'], 'topic', '/model/iris_uav/odometry');
set_param([mdl '/OdomSub'], 'messageType', 'nav_msgs/Odometry');
set_param([mdl '/OdomSub'], 'sampleTime', '0.05'); % 20Hz, typical flow sensor rate

%% Extract velocity (N,E) and height (Z)
add_block('simulink/Signal Routing/Bus Selector', [mdl '/VelHeightSel'], 'Position', [280 30 420 150]);
set_param([mdl '/VelHeightSel'], 'OutputSignals', ...
    'twist.twist.linear.x,twist.twist.linear.y,pose.pose.position.z');

%% Sensor model: velocity/height -> flow rate, with altitude + water quality
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/FlowModel'], 'Position', [480 30 700 170]);
flowModelCode = [ ...
"function [flow_rate_x, flow_rate_y, quality] = flow_model(veast, vnorth, height_m)", ...
sprintf("    FLOW_OPTIMAL_HEIGHT_M = %.4f;", FLOW_OPTIMAL_HEIGHT_M), ...
sprintf("    FLOW_MIN_USABLE_HEIGHT_M = %.4f;", FLOW_MIN_USABLE_HEIGHT_M), ...
sprintf("    FLOW_MAX_USABLE_HEIGHT_M = %.4f;", FLOW_MAX_USABLE_HEIGHT_M), ...
sprintf("    WATER_QUALITY_FACTOR = %.4f;", WATER_QUALITY_FACTOR), ...
sprintf("    NOISE_STDDEV_FRAC = %.4f;", NOISE_STDDEV_FRAC), ...
"    h = max(height_m, 0.05); % avoid divide-by-zero right at the surface", ...
"    true_rate_x = -veast / h;  % small-angle optical-flow approximation", ...
"    true_rate_y =  vnorth / h;", ...
"    flow_rate_x = true_rate_x + NOISE_STDDEV_FRAC * abs(true_rate_x) * randn();", ...
"    flow_rate_y = true_rate_y + NOISE_STDDEV_FRAC * abs(true_rate_y) * randn();", ...
"    if h < FLOW_MIN_USABLE_HEIGHT_M || h > FLOW_MAX_USABLE_HEIGHT_M", ...
"        quality = uint8(0);", ...
"    else", ...
"        % U-shaped altitude quality curve, peak at FLOW_OPTIMAL_HEIGHT_M,", ...
"        % scaled down by the ambient open-water penalty.", ...
"        dist_from_optimal = abs(h - FLOW_OPTIMAL_HEIGHT_M) / FLOW_OPTIMAL_HEIGHT_M;", ...
"        base_q = max(0, 1 - 0.5 * dist_from_optimal);", ...
"        q = base_q * WATER_QUALITY_FACTOR * 255;", ...
"        quality = uint8(max(0, min(255, round(q + randn() * 10))));", ...
"    end", ...
"end" ...
];
rt1 = sfroot;
flowChart = rt1.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/FlowModel']);
flowChart.Script = strjoin(flowModelCode, newline);

%% geometry_msgs/TwistStamped message assembly (see header note on repurposing)
add_block('ros2lib/Blank Message', [mdl '/BlankTwist'], 'Position', [780 40 880 70]);
set_param([mdl '/BlankTwist'], 'entityType', 'geometry_msgs/TwistStamped');
set_param([mdl '/BlankTwist'], 'messageType', 'geometry_msgs/TwistStamped');

add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/QualityToDouble'], 'Position', [780 120 900 160]);
q2dCode = [ ...
"function q_double = quality_to_double(quality)", ...
"    q_double = double(quality);", ...
"end" ...
];
rt2 = sfroot;
q2dChart = rt2.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/QualityToDouble']);
q2dChart.Script = strjoin(q2dCode, newline);

add_block('simulink/Signal Routing/Bus Assignment', [mdl '/AssignTwist'], 'Position', [960 40 1080 260]);
set_param([mdl '/AssignTwist'], 'AssignedSignals', 'twist.linear.x,twist.linear.y,twist.angular.z');

%% Header Assignment + Publish
add_block('ros2lib/Header Assignment', [mdl '/HdrAssign'], 'Position', [1140 120 1260 170]);
set_param([mdl '/HdrAssign'], 'SetFrameID', 'on');
set_param([mdl '/HdrAssign'], 'FrameID', 'optical_flow');
set_param([mdl '/HdrAssign'], 'InsertTimeStamp', 'on');

add_block('ros2lib/Publish', [mdl '/FlowPub'], 'Position', [1320 120 1420 170]);
set_param([mdl '/FlowPub'], 'topicSource', 'Specify your own');
set_param([mdl '/FlowPub'], 'topic', '/optical_flow');
set_param([mdl '/FlowPub'], 'messageType', 'geometry_msgs/TwistStamped');

%% Wiring
odomPh = get_param([mdl '/OdomSub'], 'PortHandles');
selPh  = get_param([mdl '/VelHeightSel'], 'PortHandles');
add_line(mdl, odomPh.Outport(2), selPh.Inport(1), 'autorouting', 'on'); % port 2 = Msg (confirmed 2026-09-13)

flowPh = get_param([mdl '/FlowModel'], 'PortHandles');
add_line(mdl, selPh.Outport(1), flowPh.Inport(1), 'autorouting', 'on');
add_line(mdl, selPh.Outport(2), flowPh.Inport(2), 'autorouting', 'on');
add_line(mdl, selPh.Outport(3), flowPh.Inport(3), 'autorouting', 'on');

q2dPh = get_param([mdl '/QualityToDouble'], 'PortHandles');
add_line(mdl, flowPh.Outport(3), q2dPh.Inport(1), 'autorouting', 'on');

blankPh = get_param([mdl '/BlankTwist'], 'PortHandles');
asgPh   = get_param([mdl '/AssignTwist'], 'PortHandles');
add_line(mdl, blankPh.Outport(1), asgPh.Inport(1), 'autorouting', 'on');
add_line(mdl, flowPh.Outport(1), asgPh.Inport(2), 'autorouting', 'on');
add_line(mdl, flowPh.Outport(2), asgPh.Inport(3), 'autorouting', 'on');
add_line(mdl, q2dPh.Outport(1), asgPh.Inport(4), 'autorouting', 'on');

hdrPh = get_param([mdl '/HdrAssign'], 'PortHandles');
add_line(mdl, asgPh.Outport(1), hdrPh.Inport(1), 'autorouting', 'on');
pubPh = get_param([mdl '/FlowPub'], 'PortHandles');
add_line(mdl, hdrPh.Outport(1), pubPh.Inport(1), 'autorouting', 'on');

set_param(mdl, 'StopTime', '30');
set_param(mdl, 'EnablePacing', 'on');
set_param(mdl, 'PacingRate', '1');
save_system(mdl, 'optical_flow_sim.slx');
fprintf('MODEL_BUILT_OK\n');
