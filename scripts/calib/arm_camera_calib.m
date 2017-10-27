clear all;

% parse marker data
markers = load('marker_data.csv');
marker_pos = [(markers(:, 5) - markers(1, 5))/1e9, markers(:, 10), markers(:, 11), markers(:, 12)];

% parse forward kinematics data
bag = rosbag('arm_calib.bag');
fwd_kin_bag = select(bag, 'Topic', '/arm_node/end_effector_position');
fwd_kin_msg = readMessages(fwd_kin_bag);
fwd_kin_times = table2array(fwd_kin_bag.MessageList( :, 1));

fwd_kin = [];
for i = 1 : size(fwd_kin_times, 1)
    fwd_kin = [fwd_kin; (fwd_kin_times(i) - markers(1, 5) / 1e9), fwd_kin_msg{i}.X, fwd_kin_msg{i}.Y, fwd_kin_msg{i}.Z];
end

% make sure we do not extrapolate times
marker_pos = marker_pos(marker_pos(:, 1) > fwd_kin(1, 1), :);
marker_pos = marker_pos(marker_pos(:, 1) < fwd_kin(end, 1), :);

% interpolate times
fwd_kin_ts = timeseries(fwd_kin(:, 2:4), fwd_kin(:, 1));
fwd_kin_interp = resample(fwd_kin_ts, marker_pos( :, 1));

% find transform of camera in frame of arm
marker_pos_hom = [marker_pos(:, 2:4), ones(size(marker_pos, 1), 1)];
A_t = marker_pos_hom\fwd_kin_interp.Data;
lsq_error = mean(sum((marker_pos_hom * A_t - fwd_kin_interp.Data).^ 2, 2))
A = A_t'

% extract rotation and translation
[U, S, V] = svd(A(1 : 3, 1 : 3));
t = A(:, 4)
R = U *V'
rpy = mat2rpy(R) * 180 / pi

T = [ R, t ];
reproj_error = mean(sum((marker_pos_hom*T' - fwd_kin_interp.Data).^2, 2));

% Nonlinear opt
fun = @(x) marker_pos_hom*[eul2rotm([x(3),x(2),x(1)]), x(4:6)']' - fwd_kin_interp.Data;
x = lsqnonlin(fun, [0, 0, 0, 0, 0, 0]);
rpy_nonlin = x(1:3)*180/pi
R_nonlin = eul2rotm([x(3),x(2),x(1)])
t_nonlin = x(4:6)

cam_arm = [[R_nonlin, t_nonlin'];
    [0, 0, 0, 1]];

res = fun(x);
reproj_error_nonlin = sqrt(mean(sum(res.^2,2)));

arm_quad = [[eul2rotm([pi, -pi/2, 0]), [.17; 0; -.109]];
    [ 0, 0, 0, 1]];

cam_quad =  arm_quad * cam_arm;
t_cam_quad = cam_quad(1:3, 4)
rpy_cam_quad = mat2rpy(cam_quad(1:3, 1:3))
