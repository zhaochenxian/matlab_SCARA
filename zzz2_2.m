%% 轨迹规划实验 - CoppeliaSim仿真版
%% 主程序（轨迹规划实验 + CoppeliaSim仿真）
%目前最完美的实验2方案
clc; clear;

%% ===== 1. 选择运行模式 =====
fprintf('=============== 轨迹规划实验 ===============\n');
fprintf('请选择运行模式:\n');
fprintf('1. 仅MATLAB分析（无仿真）\n');
fprintf('2. CoppeliaSim仿真验证\n');
fprintf('3. 完整实验（分析+仿真）\n');

choice = input('请输入选择 (1-3): ');

try
    %% ===== 2. 机器人模型创建 =====
    my_robot = create_robot_model();
    
    % 设置起始点和目标点
    P1 = [0.4, 0, 0.10];     % 起始点
    P2 = [0, -0.4, 0.10];    % 目标点
    
    fprintf('起始点 P1: [%.3f, %.3f, %.3f] m\n', P1);
    fprintf('目标点 P2: [%.3f, %.3f, %.3f] m\n', P2);
    
    % 验证工作空间
    check_workspace_validity(P1, P2, my_robot);
    
    % 时间参数设置（移到这里，所有模式都可以使用）
    t_total = 5.0;
    dt = 0.05;
    t = 0:dt:t_total;
    
    if choice == 1 || choice == 3
        %% ===== 3. 关节空间轨迹规划分析 =====
        fprintf('\n=============== 关节空间轨迹规划 ===============\n');
        
        % 逆运动学求解
        q_start = improved_inverse_kinematics(P1(1), P1(2), P1(3), my_robot);
        q_end = improved_inverse_kinematics(P2(1), P2(2), P2(3), my_robot);
        
        % 三种轨迹规划方法
        [q_linear, qd_linear, qdd_linear] = linear_trajectory_planning(q_start, q_end, t);
        [q_parabolic, qd_parabolic, qdd_parabolic] = parabolic_trajectory_planning(q_start, q_end, t);
        [q_cubic, qd_cubic, qdd_cubic] = cubic_trajectory_planning(q_start, q_end, t);
        
        % 绘制分析图
        plot_single_trajectory_method(t, q_linear, qd_linear, qdd_linear, my_robot, '关节空间一次线性轨迹规划', 'linear');
        plot_single_trajectory_method(t, q_parabolic, qd_parabolic, qdd_parabolic, my_robot, '关节空间二次抛物线轨迹规划', 'parabolic');
        plot_single_trajectory_method(t, q_cubic, qd_cubic, qdd_cubic, my_robot, '关节空间三次轨迹规划', 'cubic');
        plot_trajectory_comparison(t, q_linear, qd_linear, qdd_linear, q_parabolic, qd_parabolic, qdd_parabolic, q_cubic, qd_cubic, qdd_cubic, my_robot);
        
        %% ===== 4. 笛卡尔空间轨迹规划分析 =====
        fprintf('\n=============== 笛卡尔空间轨迹规划 ===============\n');
        
        % 直线轨迹
        [line_traj, line_t] = cartesian_line_trajectory(P1, P2, t_total, dt);
        execute_cartesian_trajectory_analysis(line_traj, line_t, my_robot, '笛卡尔空间直线轨迹');
        
        % 圆形轨迹
        circle_center = [0.3, -0.2, 0.08];
        circle_radius = 0.08;
        [circle_traj, circle_t] = cartesian_circle_trajectory(circle_center, circle_radius, t_total, dt);
        execute_cartesian_trajectory_analysis(circle_traj, circle_t, my_robot, '笛卡尔空间圆形轨迹');
    end
    
    if choice == 2 || choice == 3
        %% ===== 5. CoppeliaSim仿真验证 =====
        fprintf('\n=============== CoppeliaSim仿真验证 ===============\n');
        
        % 初始化仿真
        [sim, clientID] = init_simulation();
        jointHandles = get_joint_handles(sim, clientID);
        check_joint_limits(sim, clientID, jointHandles, my_robot);
        
        % 选择要仿真的轨迹（修正：添加完整的选择菜单）
        fprintf('选择要仿真的轨迹:\n');
        fprintf('1. 关节空间线性轨迹\n');
        fprintf('2. 关节空间二次抛物线轨迹\n');
        fprintf('3. 关节空间三次轨迹\n');
        fprintf('4. 笛卡尔空间直线轨迹\n');
        fprintf('5. 笛卡尔空间圆形轨迹\n');
        fprintf('6. 绘制数字"24"轨迹\n');
        
        sim_choice = input('请选择仿真轨迹 (1-6): ');
        
        switch sim_choice
            case 1
                % 关节空间线性轨迹仿真
                q_start = improved_inverse_kinematics(P1(1), P1(2), P1(3), my_robot);
                q_end = improved_inverse_kinematics(P2(1), P2(2), P2(3), my_robot);
                [q_sim, ~, ~] = linear_trajectory_planning(q_start, q_end, t);
                [xyTrajectory, ~] = forward_kinematics(q_sim, my_robot, 'SampleTime', dt, 'Plot', true);
                execute_trajectory(sim, clientID, jointHandles, q_sim, t, [], xyTrajectory);
                
            case 2
                % 关节空间二次抛物线轨迹仿真
                q_start = improved_inverse_kinematics(P1(1), P1(2), P1(3), my_robot);
                q_end = improved_inverse_kinematics(P2(1), P2(2), P2(3), my_robot);
                [q_sim, ~, ~] = parabolic_trajectory_planning(q_start, q_end, t);
                [xyTrajectory, ~] = forward_kinematics(q_sim, my_robot, 'SampleTime', dt, 'Plot', true);
                execute_trajectory(sim, clientID, jointHandles, q_sim, t, [], xyTrajectory);
                
            case 3
                % 关节空间三次轨迹仿真
                q_start = improved_inverse_kinematics(P1(1), P1(2), P1(3), my_robot);
                q_end = improved_inverse_kinematics(P2(1), P2(2), P2(3), my_robot);
                [q_sim, ~, ~] = cubic_trajectory_planning(q_start, q_end, t);
                [xyTrajectory, ~] = forward_kinematics(q_sim, my_robot, 'SampleTime', dt, 'Plot', true);
                execute_trajectory(sim, clientID, jointHandles, q_sim, t, [], xyTrajectory);
                
            case 4
                % 笛卡尔空间直线轨迹仿真
                [line_traj, line_t] = cartesian_line_trajectory(P1, P2, t_total, dt);
                execute_cartesian_trajectory_simulation(sim, clientID, jointHandles, line_traj, line_t, my_robot, '笛卡尔空间直线轨迹');
                
            case 5
                % 笛卡尔空间圆形轨迹仿真
                circle_center = [0.3, -0.2, 0.08];
                circle_radius = 0.08;
                [circle_traj, circle_t] = cartesian_circle_trajectory(circle_center, circle_radius, t_total, dt);
                execute_cartesian_trajectory_simulation(sim, clientID, jointHandles, circle_traj, circle_t, my_robot, '笛卡尔空间圆形轨迹');
                
            case 6
                % 绘制数字"24"轨迹
                execute_digit_24_trajectory(sim, clientID, jointHandles, my_robot);
                
            otherwise
                fprintf('无效选择，请重新运行程序\n');
                return;
        end
        
        % 停止仿真
        stop_simulation(sim, clientID);
    end
    
    fprintf('\n=============== 轨迹规划实验完成 ===============\n');
    
catch ME
    fprintf('[ERROR] 执行失败: %s\n', ME.message);
    if exist('clientID', 'var') && clientID > -1
        stop_simulation(sim, clientID);
    end
    rethrow(ME);
end

%% ===== 数字"24"轨迹仿真函数 =====
function execute_digit_24_trajectory(sim, clientID, jointHandles, robot)
    fprintf('开始执行数字"24"轨迹仿真...\n');
    
    % 定义数字"24"的轨迹点
    trajectoryPoints = [
        0.15,    0.25,  0.10;    % 数字"2"开始
        0.35,    0.25,  0.10;
        0.35,    0,     0.10;
        0.15,    0,     0.10;
        0.15,   -0.25,  0.10;
        0.35,   -0.25,  0.10;
        0.4,     0,     0.20;    % 移动到数字"4"
        0.45,    0.25,  0.10;    % 数字"4"开始
        0.45,    0,     0.10;
        0.7,     0,     0.10;
        0.675,   0.125, 0.20;    % 抬笔
        0.65,    0.25,  0.10;
        0.65,   -0.25,  0.10
    ]';
    
    % 使用改进的插值方法生成平滑轨迹
    pathLength = calculate_path_length(trajectoryPoints);
    totalTime = max(3, 4.2 * pathLength);
    
    interpolatedPath = interpolate_cartesian_path_smooth(trajectoryPoints, ...
        'TotalTime', totalTime, ...
        'SampleRate', 200, ...
        'MaxVelocity', 0.3, ...
        'MaxAcceleration', 0.8, ...
        'InterpolationMethod', 'pchip', ...
        'SmoothingWindow', 7, ...
        'Plot', true);
    
    % 逆运动学求解
    jointSolutions = calculate_inverse_kinematics_for_path(interpolatedPath, robot);
    
    % 时间向量
    nSamples = size(interpolatedPath, 2);
    t = linspace(0, totalTime, nSamples);
    
    % 关节轨迹平滑
    [smoothedJointTrajectories, pp] = smooth_joint_trajectories_basic(jointSolutions, t, 'SmoothMethod', 'linear');
    
    % 正向运动学验证
    [xyTrajectory, ~] = forward_kinematics(smoothedJointTrajectories, robot, 'SampleTime', t(2)-t(1), 'Plot', true);
    
    % 执行轨迹
    execute_trajectory(sim, clientID, jointHandles, smoothedJointTrajectories, t, pp, xyTrajectory);
end

%% ===== 笛卡尔空间轨迹仿真执行函数 =====
function execute_cartesian_trajectory_simulation(sim, clientID, jointHandles, trajectory, t, robot, title_str)
    fprintf('开始执行%s仿真...\n', title_str);
    
    % 逆运动学求解
    n_points = size(trajectory, 2);
    joint_trajectory = zeros(4, n_points);
    valid_points = 0;
    
    for i = 1:n_points
        try
            q = improved_inverse_kinematics(trajectory(1,i), trajectory(2,i), trajectory(3,i), robot);
            joint_trajectory(:,i) = q;
            valid_points = valid_points + 1;
        catch ME
            if i > 1
                joint_trajectory(:,i) = joint_trajectory(:,i-1);
            else
                joint_trajectory(:,i) = [0; 0; 0.1; 0];
            end
        end
    end
    
    fprintf('有效轨迹点: %d/%d\n', valid_points, n_points);
    
    % 正向运动学验证
    [xyTrajectory, ~] = forward_kinematics(joint_trajectory, robot, 'SampleTime', t(2)-t(1), 'Plot', true);
    
    % 执行轨迹
    execute_trajectory(sim, clientID, jointHandles, joint_trajectory, t, [], xyTrajectory);
    
    fprintf('%s仿真执行完成\n', title_str);
end

%% ===== 分析版笛卡尔空间轨迹执行函数 =====
function execute_cartesian_trajectory_analysis(trajectory, t, robot, title_str)
    fprintf('开始分析%s...\n', title_str);
    
    % 逆运动学求解
    n_points = size(trajectory, 2);
    joint_trajectory = zeros(4, n_points);
    valid_points = 0;
    
    for i = 1:n_points
        try
            q = improved_inverse_kinematics(trajectory(1,i), trajectory(2,i), trajectory(3,i), robot);
            joint_trajectory(:,i) = q;
            valid_points = valid_points + 1;
        catch ME
            if i > 1
                joint_trajectory(:,i) = joint_trajectory(:,i-1);
            else
                joint_trajectory(:,i) = [0; 0; 0.1; 0];
            end
        end
    end
    
    fprintf('有效轨迹点: %d/%d\n', valid_points, n_points);
    
    % 可视化
    visualize_cartesian_trajectory(trajectory, joint_trajectory, t, title_str, robot);
    
    fprintf('%s分析完成\n', title_str);
end

%% ===== 改进的逆运动学函数 =====
function q = improved_inverse_kinematics(x, y, z, robot)
    try
        a2 = robot.links(2).a;
        a3 = robot.links(3).a;
        base_height = robot.links(1).d;
        offset_y = robot.links(2).d;
    catch
        a2 = 0.467;
        a3 = 0.4005;
        base_height = 0.152;
        offset_y = 0.05;
    end
    
    % 考虑第二关节的偏移
    z_adj = z - offset_y;
    
    % 计算水平距离
    r = sqrt(x^2 + y^2);
    
    % 工作空间检查
    r_max = a2 + a3;
    r_min = abs(a2 - a3);
    
    if r > r_max
        scale = r_max * 0.95 / r;
        x = x * scale;
        y = y * scale;
        r = sqrt(x^2 + y^2);
    elseif r < r_min && r > 1e-6
        scale = r_min / r;
        x = x * scale;
        y = y * scale;
        r = sqrt(x^2 + y^2);
    end
    
    % 计算关节角
    cos_theta2 = (r^2 - a2^2 - a3^2) / (2*a2*a3);
    cos_theta2 = max(-1, min(1, cos_theta2));
    
    sin_theta2 = sqrt(1 - cos_theta2^2);
    theta2 = atan2(sin_theta2, cos_theta2);
    
    k1 = a2 + a3*cos_theta2;
    k2 = a3*sin_theta2;
    theta1 = atan2(y, x) - atan2(k2, k1);
    
    d3 = base_height - z_adj;
    theta4 = 0;
    
    q = [theta1; theta2; d3; theta4];
    
    % 关节限位检查
    for j = 1:length(q)
        if q(j) < robot.qlim(j,1) || q(j) > robot.qlim(j,2)
            q(j) = max(robot.qlim(j,1), min(q(j), robot.qlim(j,2)));
        end
    end
end

%% ===== 工作空间验证函数 =====
function check_workspace_validity(P1, P2, robot)
    fprintf('验证工作空间...\n');
    
    try
        q1 = improved_inverse_kinematics(P1(1), P1(2), P1(3), robot);
        fprintf('P1可达，关节角: [%.3f, %.3f, %.3f, %.3f]\n', q1);
        
        T1 = robot.fkine(q1');
        if isa(T1, 'SE3')
            pos1 = T1.t;
        else
            pos1 = T1(1:3, 4);
        end
        error1 = norm(pos1 - P1');
        fprintf('P1正向运动学误差: %.6f m\n', error1);
        
    catch ME
        warning('P1不可达: %s', ME.message);
    end
    
    try
        q2 = improved_inverse_kinematics(P2(1), P2(2), P2(3), robot);
        fprintf('P2可达，关节角: [%.3f, %.3f, %.3f, %.3f]\n', q2);
        
        T2 = robot.fkine(q2');
        if isa(T2, 'SE3')
            pos2 = T2.t;
        else
            pos2 = T2(1:3, 4);
        end
        error2 = norm(pos2 - P2');
        fprintf('P2正向运动学误差: %.6f m\n', error2);
        
    catch ME
        warning('P2不可达: %s', ME.message);
    end
end

%% ===== 其他核心函数（从之前的代码复制） =====

% 轨迹规划函数
function [q, qd, qdd] = linear_trajectory_planning(q_start, q_end, t)
    n_joints = length(q_start);
    n_points = length(t);
    
    q = zeros(n_joints, n_points);
    qd = zeros(n_joints, n_points);
    qdd = zeros(n_joints, n_points);
    
    t_total = t(end);
    
    for i = 1:n_joints
        q(i,:) = q_start(i) + (q_end(i) - q_start(i)) * t / t_total;
        qd(i,:) = (q_end(i) - q_start(i)) / t_total * ones(size(t));
        qdd(i,:) = zeros(size(t));
    end
    
    fprintf('一次线性轨迹规划完成\n');
end

function [q, qd, qdd] = parabolic_trajectory_planning(q_start, q_end, t)
    n_joints = length(q_start);
    n_points = length(t);
    
    q = zeros(n_joints, n_points);
    qd = zeros(n_joints, n_points);
    qdd = zeros(n_joints, n_points);
    
    t_total = t(end);
    t_mid = t_total / 2;
    
    for i = 1:n_joints
        for j = 1:n_points
            if t(j) <= t_mid
                q(i,j) = q_start(i) + 2*(q_end(i) - q_start(i))*(t(j)/t_total)^2;
                qd(i,j) = 4*(q_end(i) - q_start(i))*t(j)/(t_total^2);
                qdd(i,j) = 4*(q_end(i) - q_start(i))/(t_total^2);
            else
                q(i,j) = q_end(i) - 2*(q_end(i) - q_start(i))*((t_total - t(j))/t_total)^2;
                qd(i,j) = 4*(q_end(i) - q_start(i))*(t_total - t(j))/(t_total^2);
                qdd(i,j) = -4*(q_end(i) - q_start(i))/(t_total^2);
            end
        end
    end
    
    fprintf('二次抛物线轨迹规划完成\n');
end

function [q, qd, qdd] = cubic_trajectory_planning(q_start, q_end, t)
    n_joints = length(q_start);
    n_points = length(t);
    
    q = zeros(n_joints, n_points);
    qd = zeros(n_joints, n_points);
    qdd = zeros(n_joints, n_points);
    
    t_total = t(end);
    
    for i = 1:n_joints
        T = t_total;
        A = [1, 0, 0, 0, 0, 0;
             0, 1, 0, 0, 0, 0;
             0, 0, 2, 0, 0, 0;
             1, T, T^2, T^3, T^4, T^5;
             0, 1, 2*T, 3*T^2, 4*T^3, 5*T^4;
             0, 0, 2, 6*T, 12*T^2, 20*T^3];
        
        b = [q_start(i); 0; 0; q_end(i); 0; 0];
        coeffs = A \ b;
        
        for j = 1:n_points
            tau = t(j);
            q(i,j) = coeffs(1) + coeffs(2)*tau + coeffs(3)*tau^2 + ...
                     coeffs(4)*tau^3 + coeffs(5)*tau^4 + coeffs(6)*tau^5;
            qd(i,j) = coeffs(2) + 2*coeffs(3)*tau + 3*coeffs(4)*tau^2 + ...
                      4*coeffs(5)*tau^3 + 5*coeffs(6)*tau^4;
            qdd(i,j) = 2*coeffs(3) + 6*coeffs(4)*tau + 12*coeffs(5)*tau^2 + ...
                       20*coeffs(6)*tau^3;
        end
    end
    
    fprintf('三次轨迹规划完成\n');
end

% 笛卡尔空间轨迹生成函数
function [trajectory, t] = cartesian_line_trajectory(P1, P2, t_total, dt)
    t = 0:dt:t_total;
    n_points = length(t);
    trajectory = zeros(3, n_points);
    
    for i = 1:n_points
        alpha = t(i) / t_total;
        trajectory(:,i) = P1' + alpha * (P2' - P1');
    end
    
    fprintf('空间直线轨迹生成完成，共%d个点\n', n_points);
end

function [trajectory, t] = cartesian_circle_trajectory(center, radius, t_total, dt)
    t = 0:dt:t_total;
    n_points = length(t);
    trajectory = zeros(3, n_points);
    
    for i = 1:n_points
        theta = 2 * pi * t(i) / t_total;
        trajectory(1,i) = center(1) + radius * cos(theta);
        trajectory(2,i) = center(2) + radius * sin(theta);
        trajectory(3,i) = center(3);
    end
    
    fprintf('空间圆形轨迹生成完成，圆心[%.3f,%.3f,%.3f]，半径%.3f\n', ...
        center, radius);
end

% 机器人模型创建
function my_robot = create_robot_model()
    L1 = Link([0, 0.152, 0, 0, 0], 'modified');
    L2 = Link([0, 0.05, 0.467, 0, 0], 'modified');
    L3 = Link([0, -0.01, 0.4005, -pi, 1], 'modified');
    L4 = Link([0, 0, 0, -pi, 0], 'modified');
    
    my_robot = SerialLink([L1 L2 L3 L4], 'name', 'RRPR_Robot');
    my_robot.qlim = [
        deg2rad([-180, 180]);
        deg2rad([-180, 180]);
        [0, 0.202];
        deg2rad([-180, 180])
    ];
    
    fprintf('机器人模型创建成功\n');
end

% CoppeliaSim相关函数
function [sim, clientID] = init_simulation()
    addpath('.\need\');
    sim = remApi('remoteApi');
    sim.simxFinish(-1);
    clientID = sim.simxStart('127.0.0.1', 19991, true, true, 5000, 5);
    
    if clientID < 0
        error('CoppeliaSim连接失败。请检查仿真是否运行');
    end
    fprintf('[STATUS] 成功连接CoppeliaSim (ClientID: %d)\n', clientID);
    
    sim.simxSynchronous(clientID, true);
    sim.simxStartSimulation(clientID, sim.simx_opmode_blocking);
end

function jointHandles = get_joint_handles(sim, clientID)
    jointNames = {'MTB_axis1', 'MTB_axis2', 'MTB_axis3', 'MTB_axis4'};
    jointHandles = zeros(1,4);
    
    for i = 1:4
        [~, jointHandles(i)] = sim.simxGetObjectHandle(...
            clientID, jointNames{i}, sim.simx_opmode_blocking);
    end
end

function check_joint_limits(sim, clientID, handles, robot)
    for j = 1:robot.n
        [~, pos] = sim.simxGetJointPosition(clientID, handles(j), sim.simx_opmode_blocking);
        
        is_prismatic = (j == 3);
        
        if is_prismatic
            if pos < robot.qlim(j,1) || pos > robot.qlim(j,2)
                error('关节%d(移动)超出限位! 当前: %.3fm', j, pos);
            end
        else
            if pos < robot.qlim(j,1) || pos > robot.qlim(j,2)
                error('关节%d(旋转)超出限位! 当前: %.1f°', j, rad2deg(pos));
            end
        end
    end
    fprintf('[STATUS] 关节限位检查通过\n');
end

function stop_simulation(sim, clientID)
    sim.simxStopSimulation(clientID, sim.simx_opmode_blocking);
    sim.simxFinish(clientID);
    sim.delete();
    fprintf('[STATUS] 仿真已安全停止\n');
end


%% ===== 新增：单独绘制每种轨迹规划方法（修复文本警告） =====
function plot_single_trajectory_method(t, q, qd, qdd, robot, method_name, method_type)
    % 为单一轨迹规划方法绘制详细图形
    
    % 设置颜色
    switch method_type
        case 'linear'
            color = 'r';
            line_style = '-';
        case 'parabolic'
            color = 'g';
            line_style = '-';
        case 'cubic'
            color = 'b';
            line_style = '-';
    end
    
    figure('Name', method_name, 'Position', [100, 100, 1600, 1000]);
    
    joint_names = {'关节1', '关节2', '关节3', '关节4'};
    joint_units = {'°', '°', 'm', '°'};
    velocity_units = {'°/s', '°/s', 'm/s', '°/s'};
    accel_units = {'°/s²', '°/s²', 'm/s²', '°/s²'};
    
    for j = 1:4
        % 位置子图
        subplot(3, 4, j);
        if j == 3  % 移动关节
            plot(t, q(j,:), color, 'LineWidth', 3, 'LineStyle', line_style);
            ylabel(['位置 (' joint_units{j} ')'], 'FontSize', 12);
        else  % 旋转关节
            plot(t, rad2deg(q(j,:)), color, 'LineWidth', 3, 'LineStyle', line_style);
            ylabel(['角度 (' joint_units{j} ')'], 'FontSize', 12);
        end
        xlabel('时间 (s)', 'FontSize', 12);
        title([joint_names{j} ' 位置'], 'FontSize', 14, 'FontWeight', 'bold');
        grid on;
        set(gca, 'FontSize', 11);
        
        % 速度子图
        subplot(3, 4, j+4);
        if j == 3  % 移动关节
            plot(t, qd(j,:), color, 'LineWidth', 3, 'LineStyle', line_style);
        else  % 旋转关节
            plot(t, rad2deg(qd(j,:)), color, 'LineWidth', 3, 'LineStyle', line_style);
        end
        xlabel('时间 (s)', 'FontSize', 12);
        ylabel(['速度 (' velocity_units{j} ')'], 'FontSize', 12);
        title([joint_names{j} ' 速度'], 'FontSize', 14, 'FontWeight', 'bold');
        grid on;
        set(gca, 'FontSize', 11);
        
        % 加速度子图
        subplot(3, 4, j+8);
        if j == 3  % 移动关节
            plot(t, qdd(j,:), color, 'LineWidth', 3, 'LineStyle', line_style);
        else  % 旋转关节
            plot(t, rad2deg(qdd(j,:)), color, 'LineWidth', 3, 'LineStyle', line_style);
        end
        xlabel('时间 (s)', 'FontSize', 12);
        ylabel(['加速度 (' accel_units{j} ')'], 'FontSize', 12);
        title([joint_names{j} ' 加速度'], 'FontSize', 14, 'FontWeight', 'bold');
        grid on;
        set(gca, 'FontSize', 11);
    end
    
    sgtitle([method_name ' - 关节运动特性分析'], 'FontSize', 18, 'FontWeight', 'bold');
    
    % 计算并显示末端执行器轨迹
    end_trajectory = calculate_end_effector_trajectory(q, robot);
    
    figure('Name', [method_name ' - 末端执行器轨迹'], 'Position', [200, 200, 1400, 900]);
    
    % 3D轨迹
    subplot(2, 3, [1, 4]);
    plot3(end_trajectory(1,:), end_trajectory(2,:), end_trajectory(3,:), ...
        color, 'LineWidth', 4, 'LineStyle', line_style);
    hold on;
    plot3(end_trajectory(1,1), end_trajectory(2,1), end_trajectory(3,1), ...
        'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot3(end_trajectory(1,end), end_trajectory(2,end), end_trajectory(3,end), ...
        'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    
    xlabel('X (m)', 'FontSize', 13);
    ylabel('Y (m)', 'FontSize', 13);
    zlabel('Z (m)', 'FontSize', 13);
    title('末端执行器3D轨迹 (关节空间规划)', 'FontSize', 15, 'FontWeight', 'bold');
    legend('轨迹', '起点', '终点', 'Location', 'best', 'FontSize', 12);
    grid on;
    view(45, 30);
    
    % 修复：使用annotation代替text避免警告
    annotation('textbox', [0.02, 0.85, 0.3, 0.1], ...
               'String', {'说明：这是关节空间规划的结果', '末端执行器轨迹为曲线'}, ...
               'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black', ...
               'FitBoxToText', 'on');
    
    % 设置坐标轴范围
    x_range = [min(end_trajectory(1,:)), max(end_trajectory(1,:))];
    y_range = [min(end_trajectory(2,:)), max(end_trajectory(2,:))];
    z_range = [min(end_trajectory(3,:)), max(end_trajectory(3,:))];
    
    x_margin = max((x_range(2) - x_range(1)) * 0.1, 0.01);
    y_margin = max((y_range(2) - y_range(1)) * 0.1, 0.01);
    z_margin = max((z_range(2) - z_range(1)) * 0.1, 0.01);
    
    xlim([x_range(1) - x_margin, x_range(2) + x_margin]);
    ylim([y_range(1) - y_margin, y_range(2) + y_margin]);
    zlim([z_range(1) - z_margin, z_range(2) + z_margin]);
    
    % X-Y平面投影
    subplot(2, 3, 2);
    plot(end_trajectory(1,:), end_trajectory(2,:), color, 'LineWidth', 3, 'LineStyle', line_style);
    hold on;
    plot(end_trajectory(1,1), end_trajectory(2,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(end_trajectory(1,end), end_trajectory(2,end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    xlabel('X (m)', 'FontSize', 12);
    ylabel('Y (m)', 'FontSize', 12);
    title('X-Y平面投影', 'FontSize', 14, 'FontWeight', 'bold');
    legend('轨迹', '起点', '终点', 'Location', 'best');
    grid on;
    axis equal;
    
    % 位置-时间曲线
    subplot(2, 3, 3);
    plot(t, end_trajectory(1,:), 'r-', 'LineWidth', 3, 'DisplayName', 'X');
    hold on;
    plot(t, end_trajectory(2,:), 'g-', 'LineWidth', 3, 'DisplayName', 'Y');
    plot(t, end_trajectory(3,:), 'b-', 'LineWidth', 3, 'DisplayName', 'Z');
    xlabel('时间 (s)', 'FontSize', 12);
    ylabel('位置 (m)', 'FontSize', 12);
    title('笛卡尔坐标-时间', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best');
    grid on;
    
    % 计算速度（数值微分）
    dt = t(2) - t(1);
    end_velocity = zeros(3, length(t));
    for i = 2:length(t)-1
        end_velocity(:,i) = (end_trajectory(:,i+1) - end_trajectory(:,i-1)) / (2*dt);
    end
    % 边界点处理
    end_velocity(:,1) = (end_trajectory(:,2) - end_trajectory(:,1)) / dt;
    end_velocity(:,end) = (end_trajectory(:,end) - end_trajectory(:,end-1)) / dt;
    
    subplot(2, 3, 5);
    plot(t, end_velocity(1,:), 'r-', 'LineWidth', 3, 'DisplayName', 'Vx');
    hold on;
    plot(t, end_velocity(2,:), 'g-', 'LineWidth', 3, 'DisplayName', 'Vy');
    plot(t, end_velocity(3,:), 'b-', 'LineWidth', 3, 'DisplayName', 'Vz');
    xlabel('时间 (s)', 'FontSize', 12);
    ylabel('速度 (m/s)', 'FontSize', 12);
    title('末端执行器速度', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best');
    grid on;
    
    % 速度幅值
    subplot(2, 3, 6);
    velocity_magnitude = sqrt(sum(end_velocity.^2, 1));
    plot(t, velocity_magnitude, color, 'LineWidth', 3, 'LineStyle', line_style);
    xlabel('时间 (s)', 'FontSize', 12);
    ylabel('速度幅值 (m/s)', 'FontSize', 12);
    title('末端执行器速度幅值', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
    
    sgtitle([method_name ' - 末端执行器运动分析'], 'FontSize', 18, 'FontWeight', 'bold');
end

%% ===== 修改：轨迹对比图（修复重叠问题） =====
function plot_trajectory_comparison(t, q_linear, qd_linear, qdd_linear, ...
    q_parabolic, qd_parabolic, qdd_parabolic, q_cubic, qd_cubic, qdd_cubic, robot)
    % 三种轨迹规划方法对比图
    
    figure('Name', '关节空间轨迹规划方法对比分析', 'Position', [150, 150, 1600, 1100]);
    
    joint_names = {'关节1', '关节2', '关节3', '关节4'};
    joint_units = {'°', '°', 'm', '°'};
    
    for j = 1:4
        % 位置对比
        subplot(3, 4, j);
        if j == 3  % 移动关节
            plot(t, q_linear(j,:), 'r-', 'LineWidth', 2.5, 'DisplayName', '线性');
            hold on;
            plot(t, q_parabolic(j,:), 'g--', 'LineWidth', 2.5, 'DisplayName', '抛物线');
            plot(t, q_cubic(j,:), 'b:', 'LineWidth', 2.5, 'DisplayName', '三次');
            ylabel(['位置 (' joint_units{j} ')'], 'FontSize', 11);
        else  % 旋转关节
            plot(t, rad2deg(q_linear(j,:)), 'r-', 'LineWidth', 2.5, 'DisplayName', '线性');
            hold on;
            plot(t, rad2deg(q_parabolic(j,:)), 'g--', 'LineWidth', 2.5, 'DisplayName', '抛物线');
            plot(t, rad2deg(q_cubic(j,:)), 'b:', 'LineWidth', 2.5, 'DisplayName', '三次');
            ylabel(['角度 (' joint_units{j} ')'], 'FontSize', 11);
        end
        xlabel('时间 (s)', 'FontSize', 11);
        title([joint_names{j} ' 位置对比'], 'FontSize', 12, 'FontWeight', 'bold');
        legend('Location', 'best', 'FontSize', 10);
        grid on;
        
        % 速度对比
        subplot(3, 4, j+4);
        if j == 3  % 移动关节
            plot(t, qd_linear(j,:), 'r-', 'LineWidth', 2.5);
            hold on;
            plot(t, qd_parabolic(j,:), 'g--', 'LineWidth', 2.5);
            plot(t, qd_cubic(j,:), 'b:', 'LineWidth', 2.5);
            ylabel('速度 (m/s)', 'FontSize', 11);
        else  % 旋转关节
            plot(t, rad2deg(qd_linear(j,:)), 'r-', 'LineWidth', 2.5);
            hold on;
            plot(t, rad2deg(qd_parabolic(j,:)), 'g--', 'LineWidth', 2.5);
            plot(t, rad2deg(qd_cubic(j,:)), 'b:', 'LineWidth', 2.5);
            ylabel('角速度 (°/s)', 'FontSize', 11);
        end
        xlabel('时间 (s)', 'FontSize', 11);
        title([joint_names{j} ' 速度对比'], 'FontSize', 12, 'FontWeight', 'bold');
        grid on;
        
        % 加速度对比
        subplot(3, 4, j+8);
        if j == 3  % 移动关节
            plot(t, qdd_linear(j,:), 'r-', 'LineWidth', 2.5);
            hold on;
            plot(t, qdd_parabolic(j,:), 'g--', 'LineWidth', 2.5);
            plot(t, qdd_cubic(j,:), 'b:', 'LineWidth', 2.5);
            ylabel('加速度 (m/s²)', 'FontSize', 11);
        else  % 旋转关节
            plot(t, rad2deg(qdd_linear(j,:)), 'r-', 'LineWidth', 2.5);
            hold on;
            plot(t, rad2deg(qdd_parabolic(j,:)), 'g--', 'LineWidth', 2.5);
            plot(t, rad2deg(qdd_cubic(j,:)), 'b:', 'LineWidth', 2.5);
            ylabel('角加速度 (°/s²)', 'FontSize', 11);
        end
        xlabel('时间 (s)', 'FontSize', 11);
        title([joint_names{j} ' 加速度对比'], 'FontSize', 12, 'FontWeight', 'bold');
        grid on;
    end
    
    sgtitle('关节空间三种轨迹规划方法对比分析', 'FontSize', 18, 'FontWeight', 'bold');
    
    % 末端执行器轨迹对比（修复重叠问题）
    figure('Name', '关节空间规划末端执行器轨迹对比分析', 'Position', [250, 250, 1500, 900]);
    
    % 计算三种规划方法的末端轨迹
    end_traj_linear = calculate_end_effector_trajectory(q_linear, robot);
    end_traj_parabolic = calculate_end_effector_trajectory(q_parabolic, robot);
    end_traj_cubic = calculate_end_effector_trajectory(q_cubic, robot);
    
    % 打印调试信息
    fprintf('末端轨迹数据检查:\n');
    fprintf('线性规划 X范围: [%.4f, %.4f]\n', min(end_traj_linear(1,:)), max(end_traj_linear(1,:)));
    fprintf('抛物线规划 X范围: [%.4f, %.4f]\n', min(end_traj_parabolic(1,:)), max(end_traj_parabolic(1,:)));
    fprintf('三次规划 X范围: [%.4f, %.4f]\n', min(end_traj_cubic(1,:)), max(end_traj_cubic(1,:)));
    
    % 3D轨迹对比（使用不同的线型和标记）
    subplot(2, 3, [1, 4]);
    h1 = plot3(end_traj_linear(1,:), end_traj_linear(2,:), end_traj_linear(3,:), ...
        'r-', 'LineWidth', 4, 'DisplayName', '关节空间线性规划');
    hold on;
    h2 = plot3(end_traj_parabolic(1,:), end_traj_parabolic(2,:), end_traj_parabolic(3,:), ...
        'g--', 'LineWidth', 4, 'DisplayName', '关节空间抛物线规划');
    h3 = plot3(end_traj_cubic(1,:), end_traj_cubic(2,:), end_traj_cubic(3,:), ...
        'b:', 'LineWidth', 4, 'DisplayName', '关节空间三次规划');
    
    % 添加起点和终点标记
    plot3(end_traj_linear(1,1), end_traj_linear(2,1), end_traj_linear(3,1), ...
        'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot3(end_traj_linear(1,end), end_traj_linear(2,end), end_traj_linear(3,end), ...
        'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    
    xlabel('X (m)', 'FontSize', 13);
    ylabel('Y (m)', 'FontSize', 13);
    zlabel('Z (m)', 'FontSize', 13);
    title('末端执行器3D轨迹对比 (关节空间规划)', 'FontSize', 15, 'FontWeight', 'bold');
    legend([h1, h2, h3], 'Location', 'best', 'FontSize', 12);
    grid on;
    view(45, 30);
    
    % 修复：使用annotation代替text避免警告
    annotation('textbox', [0.02, 0.02, 0.35, 0.08], ...
               'String', {'说明：这些都是关节空间规划的结果，末端轨迹均为曲线'}, ...
               'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black', ...
               'FitBoxToText', 'on');
    
    % 设置坐标轴范围
    all_x = [end_traj_linear(1,:), end_traj_parabolic(1,:), end_traj_cubic(1,:)];
    all_y = [end_traj_linear(2,:), end_traj_parabolic(2,:), end_traj_cubic(2,:)];
    all_z = [end_traj_linear(3,:), end_traj_parabolic(3,:), end_traj_cubic(3,:)];
    
    x_range = [min(all_x), max(all_x)];
    y_range = [min(all_y), max(all_y)];
    z_range = [min(all_z), max(all_z)];
    
    x_margin = max((x_range(2) - x_range(1)) * 0.1, 0.01);
    y_margin = max((y_range(2) - y_range(1)) * 0.1, 0.01);
    z_margin = max((z_range(2) - z_range(1)) * 0.1, 0.01);
    
    xlim([x_range(1) - x_margin, x_range(2) + x_margin]);
    ylim([y_range(1) - y_margin, y_range(2) + y_margin]);
    zlim([z_range(1) - z_margin, z_range(2) + z_margin]);
    
    % X-Y平面投影对比
    subplot(2, 3, 2);
    plot(end_traj_linear(1,:), end_traj_linear(2,:), 'r-', 'LineWidth', 3, 'DisplayName', '线性');
    hold on;
    plot(end_traj_parabolic(1,:), end_traj_parabolic(2,:), 'g--', 'LineWidth', 3, 'DisplayName', '抛物线');
    plot(end_traj_cubic(1,:), end_traj_cubic(2,:), 'b:', 'LineWidth', 3, 'DisplayName', '三次');
    
    plot(end_traj_linear(1,1), end_traj_linear(2,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(end_traj_linear(1,end), end_traj_linear(2,end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    xlabel('X (m)', 'FontSize', 12);
    ylabel('Y (m)', 'FontSize', 12);
    title('X-Y平面轨迹对比', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 11);
    grid on;
    axis equal;
    
    % X方向位置对比
    subplot(2, 3, 3);
    plot(t, end_traj_linear(1,:), 'r-', 'LineWidth', 3, 'DisplayName', 'X-线性');
    hold on;
    plot(t, end_traj_parabolic(1,:), 'g--', 'LineWidth', 3, 'DisplayName', 'X-抛物线');
    plot(t, end_traj_cubic(1,:), 'b:', 'LineWidth', 3, 'DisplayName', 'X-三次');
    xlabel('时间 (s)', 'FontSize', 12);
    ylabel('X位置 (m)', 'FontSize', 12);
    title('X方向位置-时间对比', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 11);
    grid on;
    
    % Y方向位置对比
    subplot(2, 3, 5);
    plot(t, end_traj_linear(2,:), 'r-', 'LineWidth', 3, 'DisplayName', 'Y-线性');
    hold on;
    plot(t, end_traj_parabolic(2,:), 'g--', 'LineWidth', 3, 'DisplayName', 'Y-抛物线');
    plot(t, end_traj_cubic(2,:), 'b:', 'LineWidth', 3, 'DisplayName', 'Y-三次');
    xlabel('时间 (s)', 'FontSize', 12);
    ylabel('Y位置 (m)', 'FontSize', 12);
    title('Y方向位置-时间对比', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 11);
    grid on;
    
    % Z方向位置对比
    subplot(2, 3, 6);
    plot(t, end_traj_linear(3,:), 'r-', 'LineWidth', 3, 'DisplayName', 'Z-线性');
    hold on;
    plot(t, end_traj_parabolic(3,:), 'g--', 'LineWidth', 3, 'DisplayName', 'Z-抛物线');
    plot(t, end_traj_cubic(3,:), 'b:', 'LineWidth', 3, 'DisplayName', 'Z-三次');
    xlabel('时间 (s)', 'FontSize', 12);
    ylabel('Z位置 (m)', 'FontSize', 12);
    title('Z方向位置-时间对比', 'FontSize', 14, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 11);
    grid on;
    
    sgtitle('关节空间规划末端执行器轨迹对比分析', 'FontSize', 18, 'FontWeight', 'bold');
end

%% ===== 笛卡尔空间轨迹可视化（修复文本警告） =====
function visualize_cartesian_trajectory(trajectory, joint_trajectory, t, title_str, robot)
    % 可视化笛卡尔空间轨迹
    
    figure('Name', title_str, 'Position', [300, 300, 1400, 900]);
    
    % 3D轨迹
    subplot(2, 3, [1, 4]);
    plot3(trajectory(1,:), trajectory(2,:), trajectory(3,:), 'b-', 'LineWidth', 3);
    hold on;
    plot3(trajectory(1,1), trajectory(2,1), trajectory(3,1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
    plot3(trajectory(1,end), trajectory(2,end), trajectory(3,end), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
    
    xlabel('X (m)', 'FontSize', 12); 
    ylabel('Y (m)', 'FontSize', 12); 
    zlabel('Z (m)', 'FontSize', 12);
    title('3D轨迹 (笛卡尔空间规划)', 'FontSize', 14, 'FontWeight', 'bold');
    grid on;
    view(45, 30);
    
    % 修复：使用annotation代替text避免警告
    if contains(title_str, '直线')
        text_str = {'说明：这是笛卡尔空间直线轨迹', '末端执行器轨迹为直线'};
    else
        text_str = {'说明：这是笛卡尔空间圆形轨迹', '末端执行器轨迹为圆形'};
    end
    
    annotation('textbox', [0.02, 0.85, 0.3, 0.1], ...
               'String', text_str, ...
               'FontSize', 10, 'BackgroundColor', 'white', 'EdgeColor', 'black', ...
               'FitBoxToText', 'on');
    
    % 计算合适的坐标轴范围
    x_range = [min(trajectory(1,:)), max(trajectory(1,:))];
    y_range = [min(trajectory(2,:)), max(trajectory(2,:))];
    z_range = [min(trajectory(3,:)), max(trajectory(3,:))];
    
    x_margin = max((x_range(2) - x_range(1)) * 0.1, 0.01);
    y_margin = max((y_range(2) - y_range(1)) * 0.1, 0.01);
    z_margin = max((z_range(2) - z_range(1)) * 0.1, 0.01);
    
    xlim([x_range(1) - x_margin, x_range(2) + x_margin]);
    ylim([y_range(1) - y_margin, y_range(2) + y_margin]);
    zlim([z_range(1) - z_margin, z_range(2) + z_margin]);
    
    legend('轨迹', '起点', '终点', 'Location', 'best');
    
    % X-Y平面轨迹
    subplot(2, 3, 2);
    plot(trajectory(1,:), trajectory(2,:), 'b-', 'LineWidth', 2.5);
    hold on;
    plot(trajectory(1,1), trajectory(2,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot(trajectory(1,end), trajectory(2,end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    xlabel('X (m)', 'FontSize', 11); 
    ylabel('Y (m)', 'FontSize', 11);
    title('X-Y平面轨迹', 'FontSize', 12, 'FontWeight', 'bold');
    grid on; 
    axis equal;
    legend('轨迹', '起点', '终点', 'Location', 'best');
    
    % 笛卡尔坐标时间曲线
    subplot(2, 3, 3);
    plot(t, trajectory(1,:), 'r-', 'LineWidth', 2, 'DisplayName', 'X');
    hold on;
    plot(t, trajectory(2,:), 'g-', 'LineWidth', 2, 'DisplayName', 'Y');
    plot(t, trajectory(3,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Z');
    xlabel('时间 (s)', 'FontSize', 11); 
    ylabel('位置 (m)', 'FontSize', 11);
    title('笛卡尔坐标-时间', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'best'); 
    grid on;
    
    % 关节角度时间曲线
    subplot(2, 3, 5);
    plot(t, rad2deg(joint_trajectory(1,:)), 'r-', 'LineWidth', 2, 'DisplayName', '关节1');
    hold on;
    plot(t, rad2deg(joint_trajectory(2,:)), 'g-', 'LineWidth', 2, 'DisplayName', '关节2');
    plot(t, rad2deg(joint_trajectory(4,:)), 'b-', 'LineWidth', 2, 'DisplayName', '关节4');
    xlabel('时间 (s)', 'FontSize', 11); 
    ylabel('角度 (°)', 'FontSize', 11);
    title('旋转关节角度', 'FontSize', 12, 'FontWeight', 'bold');
    legend('Location', 'best'); 
    grid on;
    
    subplot(2, 3, 6);
    plot(t, joint_trajectory(3,:), 'm-', 'LineWidth', 2.5);
    xlabel('时间 (s)', 'FontSize', 11); 
    ylabel('位移 (m)', 'FontSize', 11);
    title('移动关节位移', 'FontSize', 12, 'FontWeight', 'bold');
    grid on;
    
    sgtitle(title_str, 'FontSize', 16, 'FontWeight', 'bold');
end
%% ===== 补充的支持函数 =====

function end_trajectory = calculate_end_effector_trajectory(joint_trajectory, robot)
    n_points = size(joint_trajectory, 2);
    end_trajectory = zeros(3, n_points);
    
    for i = 1:n_points
        T = robot.fkine(joint_trajectory(:,i)');
        if isa(T, 'SE3')
            end_trajectory(:,i) = T.t;
        else
            end_trajectory(:,i) = T(1:3, 4);
        end
    end
end

%% ===== 改进的插值函数 =====
function interpolatedPath = interpolate_cartesian_path_smooth(waypoints, varargin)
    % 改进的笛卡尔空间路径插值：样条插值 + 速度控制 + 平滑处理
    
    p = inputParser;
    addRequired(p, 'waypoints', @(x) size(x,1)==3);
    addParameter(p, 'TotalTime', 3, @(x)x>0);
    addParameter(p, 'SampleRate', 200, @(x)x>10);
    addParameter(p, 'MaxVelocity', 0.5, @(x)x>0);      % 最大速度 m/s
    addParameter(p, 'MaxAcceleration', 1.0, @(x)x>0);  % 最大加速度 m/s²
    addParameter(p, 'InterpolationMethod', 'spline', @ischar); % 插值方法
    addParameter(p, 'SmoothingWindow', 7, @(x)x>0);    % 平滑窗口大小
    addParameter(p, 'Plot', false, @islogical);
    parse(p, waypoints, varargin{:});
    
    %% 步骤1：计算路径段长度和时间分配
    pathSegments = diff(waypoints, 1, 2);
    segmentLengths = sqrt(sum(pathSegments.^2, 1));
    totalLength = sum(segmentLengths);
    
    % 使用累积弧长参数化
    arcLength = [0, cumsum(segmentLengths)];
    normalizedArcLength = arcLength / totalLength;
    timePoints = normalizedArcLength * p.Results.TotalTime;
    
    fprintf('[INFO] 路径分析: %d个路径点, 总长度%.3fm, 总时间%.2fs\n', ...
        size(waypoints,2), totalLength, p.Results.TotalTime);
    
    %% 步骤2：样条插值（基于弧长参数化）
    dt = 1 / p.Results.SampleRate;
    t_fine = 0:dt:p.Results.TotalTime;
    
    % 对每个维度进行样条插值
    splineInterp = zeros(3, length(t_fine));
    for dim = 1:3
        if strcmp(p.Results.InterpolationMethod, 'spline')
            % 使用MATLAB的spline函数（三次样条）
            splineInterp(dim,:) = spline(timePoints, waypoints(dim,:), t_fine);
        elseif strcmp(p.Results.InterpolationMethod, 'pchip')
            % 使用分段三次Hermite插值（保单调性）
            splineInterp(dim,:) = pchip(timePoints, waypoints(dim,:), t_fine);
        else
            % 回退到线性插值
            splineInterp(dim,:) = interp1(timePoints, waypoints(dim,:), t_fine, 'linear');
        end
    end
    
    %% 步骤3：速度和加速度约束处理
    [constrainedPath, actualVel, actualAcc] = apply_velocity_acceleration_limits_improved(...
        splineInterp, t_fine, p.Results.MaxVelocity, p.Results.MaxAcceleration);
    
    %% 步骤4：最终平滑处理
    interpolatedPath = apply_final_smoothing_improved(constrainedPath, p.Results.SmoothingWindow);
    
    %% 步骤5：确保通过所有关键点
    interpolatedPath = ensure_waypoint_passage(interpolatedPath, waypoints, timePoints, t_fine);
    
    if p.Results.Plot
        visualize_improved_interpolation(waypoints, interpolatedPath, timePoints, t_fine, actualVel, actualAcc);
    end
    
    fprintf('[INFO] 插值完成: %d -> %d点, 最大速度%.3fm/s, 最大加速度%.3fm/s²\n', ...
        size(waypoints,2), size(interpolatedPath,2), actualVel, actualAcc);
end

function [smoothPath, maxVel, maxAcc] = apply_velocity_acceleration_limits_improved(path, t, vMax, aMax)
    % 改进的速度和加速度限制处理
    
    dt = t(2) - t(1);
    smoothPath = path;
    
    % 计算初始速度和加速度
    velocity = diff(path, 1, 2) / dt;
    velMagnitude = sqrt(sum(velocity.^2, 1));
    
    maxVel = max(velMagnitude);
    
    % 速度限制：使用时间重参数化
    if maxVel > vMax
        % 计算需要的时间缩放因子
        scaleFactor = vMax / maxVel;
        fprintf('[INFO] 速度超限(%.3f m/s)，应用时间缩放因子: %.3f\n', maxVel, scaleFactor);
        
        % 重新参数化时间
        newTotalTime = t(end) / scaleFactor;
        t_scaled = linspace(0, newTotalTime, length(t));
        
        % 重新映射路径到新的时间尺度
        for dim = 1:3
            smoothPath(dim,:) = interp1(t, path(dim,:), ...
                t_scaled * scaleFactor, 'pchip', 'extrap');
        end
        
        % 更新时间向量
        t = t_scaled;
        dt = t(2) - t(1);
    end
    
    % 重新计算速度和加速度
    velocity = diff(smoothPath, 1, 2) / dt;
    acceleration = diff(velocity, 1, 2) / dt;
    
    maxVel = max(sqrt(sum(velocity.^2, 1)));
    maxAcc = max(sqrt(sum(acceleration.^2, 1)));
    
    % 加速度限制：使用Savitzky-Golay滤波器平滑
    if maxAcc > aMax
        fprintf('[INFO] 加速度超限(%.3f m/s²)，应用平滑滤波\n', maxAcc);
        
        % 计算滤波器参数
        windowSize = max(5, ceil(0.05 / dt)); % 0.05秒的窗口
        if mod(windowSize, 2) == 0
            windowSize = windowSize + 1; % 确保是奇数
        end
        polynomialOrder = min(3, windowSize-1);
        
        % 应用Savitzky-Golay滤波
        for dim = 1:3
            if exist('sgolayfilt', 'file')
                smoothPath(dim,:) = sgolayfilt(smoothPath(dim,:), polynomialOrder, windowSize);
            else
                % 如果没有Signal Processing Toolbox，使用移动平均
                smoothPath(dim,:) = movmean(smoothPath(dim,:), windowSize);
            end
        end
        
        % 重新计算
        velocity = diff(smoothPath, 1, 2) / dt;
        acceleration = diff(velocity, 1, 2) / dt;
        maxVel = max(sqrt(sum(velocity.^2, 1)));
        maxAcc = max(sqrt(sum(acceleration.^2, 1)));
    end
end

function finalPath = apply_final_smoothing_improved(path, windowSize)
    % 改进的最终平滑处理
    
    finalPath = zeros(size(path));
    
    % 确保窗口大小为奇数
    if mod(windowSize, 2) == 0
        windowSize = windowSize + 1;
    end
    
    for dim = 1:3
        if exist('sgolayfilt', 'file')
            % 使用Savitzky-Golay滤波器（保持数据特征）
            polynomialOrder = min(3, windowSize-1);
            finalPath(dim,:) = sgolayfilt(path(dim,:), polynomialOrder, windowSize);
        else
            % 回退到加权移动平均
            finalPath(dim,:) = movmean(path(dim,:), windowSize);
        end
    end
    
    % 确保起点和终点精确不变
    finalPath(:,1) = path(:,1);
    finalPath(:,end) = path(:,end);
    
    % 在路径点附近进行局部修正（减少偏差）
    tolerance = 3; % 在路径点附近的容差范围
    for i = 1:size(finalPath,2)
        for j = 1:size(path,2)
            if abs(i - j) <= tolerance
                weight = exp(-abs(i-j)/tolerance); % 高斯权重
                finalPath(:,i) = (1-weight) * finalPath(:,i) + weight * path(:,j);
            end
        end
    end
end

function correctedPath = ensure_waypoint_passage(interpolatedPath, waypoints, waypointTimes, t)
    % 确保插值路径尽可能接近原始路径点
    
    correctedPath = interpolatedPath;
    
    % 找到每个路径点在插值时间序列中的最近位置
    for i = 1:size(waypoints, 2)
        [~, closestIdx] = min(abs(t - waypointTimes(i)));
        
        % 在路径点附近应用局部修正
        correctionRadius = 5; % 修正半径（采样点数）
        startIdx = max(1, closestIdx - correctionRadius);
        endIdx = min(length(t), closestIdx + correctionRadius);
        
        % 使用高斯权重进行局部修正
        for k = startIdx:endIdx
            distance = abs(k - closestIdx);
            weight = exp(-distance^2 / (correctionRadius/2)^2); % 高斯权重
            
            correctedPath(:,k) = (1-weight) * correctedPath(:,k) + weight * waypoints(:,i);
        end
    end
end

function visualize_improved_interpolation(waypoints, interpolatedPath, waypointTimes, t, maxVel, maxAcc)
    % 改进的可视化函数
    
    figure('Name', '改进插值结果', 'Position', [100,100,1400,900]);
    
    % 计算导数
    dt = t(2) - t(1);
    velocity = diff(interpolatedPath, 1, 2) / dt;
    acceleration = diff(velocity, 1, 2) / dt;
    velMagnitude = sqrt(sum(velocity.^2, 1));
    accMagnitude = sqrt(sum(acceleration.^2, 1));
    
    % 3D轨迹（更大的子图）
    subplot(2,3,[1,4]);
    plot3(interpolatedPath(1,:), interpolatedPath(2,:), interpolatedPath(3,:), 'b-', 'LineWidth', 2.5);
    hold on;
    plot3(waypoints(1,:), waypoints(2,:), waypoints(3,:), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    % 添加路径点编号
    for i = 1:size(waypoints,2)
        text(waypoints(1,i), waypoints(2,i), waypoints(3,i), sprintf('  P%d', i), ...
            'FontSize', 9, 'Color', 'red', 'FontWeight', 'bold');
    end
    
    xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
    title('3D轨迹（样条插值）', 'FontSize', 12); 
    grid on; axis equal;
    legend('插值轨迹', '路径点', 'Location', 'best');
    
    % 位置-时间
    subplot(2,3,2);
    plot(t, interpolatedPath(1,:), 'r-', 'DisplayName', 'X', 'LineWidth', 1.5);
    hold on;
    plot(t, interpolatedPath(2,:), 'g-', 'DisplayName', 'Y', 'LineWidth', 1.5);
    plot(t, interpolatedPath(3,:), 'b-', 'DisplayName', 'Z', 'LineWidth', 1.5);
    scatter(waypointTimes, waypoints(1,:), 50, 'r', 'filled', '^');
    scatter(waypointTimes, waypoints(2,:), 50, 'g', 'filled', '^');
    scatter(waypointTimes, waypoints(3,:), 50, 'b', 'filled', '^');
    xlabel('时间 (s)'); ylabel('位置 (m)'); title('位置-时间曲线');
    legend('Location', 'best'); grid on;
    
    % 速度大小
    subplot(2,3,3);
    plot(t(1:end-1), velMagnitude, 'k-', 'LineWidth', 2);
    xlabel('时间 (s)'); ylabel('速度 (m/s)'); title('速度大小');
    grid on;
    ylim([0, max(velMagnitude)*1.1]);
    
    % 速度分量
    subplot(2,3,5);
    plot(t(1:end-1), velocity(1,:), 'r-', 'DisplayName', 'Vx', 'LineWidth', 1.5);
    hold on;
    plot(t(1:end-1), velocity(2,:), 'g-', 'DisplayName', 'Vy', 'LineWidth', 1.5);
    plot(t(1:end-1), velocity(3,:), 'b-', 'DisplayName', 'Vz', 'LineWidth', 1.5);
    xlabel('时间 (s)'); ylabel('速度 (m/s)'); title('速度分量');
    legend('Location', 'best'); grid on;
    
    % 加速度大小和统计信息
    subplot(2,3,6);
    plot(t(1:end-2), accMagnitude, 'k-', 'LineWidth', 2);
    hold on;
    xlabel('时间 (s)'); ylabel('加速度 (m/s²)'); title('加速度大小');
    grid on;
    
    % 添加文本统计信息
    stats_text = {
        sprintf('路径长度: %.3f m', sum(sqrt(sum(diff(interpolatedPath,1,2).^2,1))));
        sprintf('总时间: %.2f s', t(end));
        sprintf('最大速度: %.3f m/s', maxVel);
        sprintf('最大加速度: %.3f m/s²', maxAcc);
        sprintf('平均速度: %.3f m/s', sum(sqrt(sum(diff(interpolatedPath,1,2).^2,1)))/t(end));
        sprintf('插值方法: 三次样条');
    };
    
    % 在图形右上角添加统计信息
    text(0.02, 0.98, stats_text, 'Units', 'normalized', 'FontSize', 9, ...
         'VerticalAlignment', 'top', 'BackgroundColor', 'white', ...
         'EdgeColor', 'black', 'Margin', 5);
    
    sgtitle('改进的样条插值轨迹分析', 'FontSize', 16, 'FontWeight', 'bold');
end

%% ===== 关节轨迹平滑函数（无图形） =====
function [smoothedTrajectories, pp] = smooth_joint_trajectories_basic(jointSolutions, t, varargin)
    % 简化的关节轨迹平滑（无可视化）
    
    p = inputParser;
    addRequired(p, 'jointSolutions', @isnumeric);
    addRequired(p, 't', @isnumeric);
    addParameter(p, 'SmoothMethod', 'linear', @ischar);
    parse(p, jointSolutions, t, varargin{:});
    
    nJoints = size(jointSolutions, 1);
    smoothedTrajectories = jointSolutions; % 保持原始解
    pp = struct();
    
    % 对每个关节应用轻微平滑
    for j = 1:nJoints
        % 创建简单的线性插值
        pp.(['j',num2str(j)]) = griddedInterpolant(t, jointSolutions(j,:), 'linear');
        
        % 应用轻微的移动平均平滑
        windowSize = 5;
        smoothedTrajectories(j,:) = movmean(jointSolutions(j,:), windowSize);
        
        % 保持起点和终点
        smoothedTrajectories(j,1) = jointSolutions(j,1);
        smoothedTrajectories(j,end) = jointSolutions(j,end);
    end
    
    fprintf('[INFO] 关节轨迹平滑完成\n');
end

function jointSolutions = calculate_inverse_kinematics_for_path(cartesianPath, robot)
    % SCARA逆运动学解算
    nPoints = size(cartesianPath, 2);
    jointSolutions = zeros(4, nPoints);
    
    fprintf('[STATUS] 开始逆运动学求解，共%d个点...\n', nPoints);
    
    for i = 1:nPoints
        if mod(i, 100) == 0 || i == nPoints
            fprintf('进度: %d/%d\n', i, nPoints);
        end
        
        x = cartesianPath(1,i);
        y = cartesianPath(2,i);
        z = cartesianPath(3,i);
        
        try
            a2 = robot.links(2).a;
            a3 = robot.links(3).a;
        catch
            a2 = 0.467;
            a3 = 0.4005;
            warning('无法读取连杆参数，使用默认值');
        end
        
        % 计算θ2
        c2 = (x^2 + y^2 - a2^2 - a3^2) / (2*a2*a3);
        
        if abs(c2) > 1
            fprintf('警告: 点%d工作空间外 (x=%.2f, y=%.2f), 使用边界值\n', i, x, y);
            c2 = sign(c2) * 1.0;
        end
        
        s2 = sqrt(1 - c2^2);
        theta2 = atan2(s2, c2);
        
        % 计算θ1
        k1 = a2 + a3*c2;
        k2 = a3*s2;
        theta1 = atan2(y, x) - atan2(k2, k1);
        
        % 计算Z轴位移和θ4
        z_base = 0.132;
        d3 = z_base - z;
        theta4 = pi/2;
        
        % 关节限位检查
        q = [theta1; theta2; d3; theta4];
        for j = 1:4
            if q(j) < robot.qlim(j,1) || q(j) > robot.qlim(j,2)
                q(j) = max(robot.qlim(j,1), min(q(j), robot.qlim(j,2)));
            end
        end
        
        jointSolutions(:,i) = q;
    end
    
    fprintf('[STATUS] 逆运动学求解完成\n');
end

function pathLen = calculate_path_length(points)
    diffVec = diff(points, 1, 2);
    pathLen = sum(sqrt(sum(diffVec.^2, 1)));
end

function execute_trajectory(sim, clientID, handles, traj, t, pp, xyTraj)
    fprintf('[STATUS] 开始执行轨迹 (点数: %d)\n', length(t));
    
    % 创建实时显示窗口
    hFig = figure('Name', '实时轨迹监控', 'Position', [100, 100, 800, 600]);
    hAx = axes('Parent', hFig);
    
    % 绘制完整轨迹
    plot3(hAx, xyTraj(1,:), xyTraj(2,:), xyTraj(3,:), 'b-', 'LineWidth', 2);
    hold(hAx, 'on');
    hCurrentPos = plot3(hAx, nan, nan, nan, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    % 添加起点和终点标记
    plot3(hAx, xyTraj(1,1), xyTraj(2,1), xyTraj(3,1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
    plot3(hAx, xyTraj(1,end), xyTraj(2,end), xyTraj(3,end), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
    
    xlabel(hAx, 'X (m)', 'FontSize', 12); 
    ylabel(hAx, 'Y (m)', 'FontSize', 12); 
    zlabel(hAx, 'Z (m)', 'FontSize', 12);
    title(hAx, '实时末端位置跟踪', 'FontSize', 14);
    grid(hAx, 'on');
    
    % ===== 修复：设置合理的坐标轴范围 =====
    % 计算实际轨迹的范围
    x_range = [min(xyTraj(1,:)), max(xyTraj(1,:))];
    y_range = [min(xyTraj(2,:)), max(xyTraj(2,:))];
    z_range = [min(xyTraj(3,:)), max(xyTraj(3,:))];
    
    % 添加适当的边界
    x_margin = max(0.1, (x_range(2) - x_range(1)) * 0.2);
    y_margin = max(0.1, (y_range(2) - y_range(1)) * 0.2);
    z_margin = max(0.02, (z_range(2) - z_range(1)) * 0.2);
    
    % 设置坐标轴限制
    xlim(hAx, [x_range(1) - x_margin, x_range(2) + x_margin]);
    ylim(hAx, [y_range(1) - y_margin, y_range(2) + y_margin]);
    zlim(hAx, [z_range(1) - z_margin, z_range(2) + z_margin]);
    
    % 设置等比例显示（但限制Z轴范围）
    axis(hAx, 'equal');
    
    % 强制Z轴范围不超过合理值
    current_zlim = zlim(hAx);
    if current_zlim(2) - current_zlim(1) > 0.5  % 如果Z轴范围超过0.5m
        z_center = mean(z_range);
        zlim(hAx, [z_center - 0.15, z_center + 0.15]);  % 限制为0.3m范围
    end
    
    % 添加图例
    legend(hAx, '规划轨迹', '当前位置', '起点', '终点', 'Location', 'best');
    
    % 添加实时信息显示
    info_text = text(0.02, 0.98, '', 'Units', 'normalized', 'FontSize', 10, ...
                     'VerticalAlignment', 'top', 'BackgroundColor', 'white', ...
                     'EdgeColor', 'black', 'Parent', hAx);
    
    % 初始位置设置
    for j = 1:4
        sim.simxSetJointPosition(clientID, handles(j), traj(j,1),...
            sim.simx_opmode_oneshot_wait);
    end
    sim.simxSynchronousTrigger(clientID);
    
    % 主循环
    startTime = tic;
    for k = 1:length(t)
        % 设置关节位置
        sim.simxPauseCommunication(clientID, true);
        for j = 1:4
            sim.simxSetJointPosition(clientID, handles(j), traj(j,k),...
                sim.simx_opmode_oneshot);
        end
        sim.simxPauseCommunication(clientID, false);
        
        % 更新实时显示
        set(hCurrentPos, 'XData', xyTraj(1,k), 'YData', xyTraj(2,k), 'ZData', xyTraj(3,k));
        
        % 更新信息文本
        progress = k / length(t) * 100;
        elapsed = toc(startTime);
        pos_str = sprintf('当前位置: [%.3f, %.3f, %.3f] m\n进度: %.1f%%\n时间: %.2fs', ...
                         xyTraj(1,k), xyTraj(2,k), xyTraj(3,k), progress, elapsed);
        set(info_text, 'String', pos_str);
        
        drawnow;
        
        % 精确时间控制
        elapsed = toc(startTime);
        desiredTime = t(k);
        while elapsed < desiredTime
            pause(0.001);
            elapsed = toc(startTime);
        end
        
        % 同步触发
        sim.simxSynchronousTrigger(clientID);
    end
    
    fprintf('[STATUS] 轨迹执行完成 (用时: %.2fs)\n', toc(startTime));
    
    % 显示最终统计
    final_stats = sprintf('轨迹执行完成!\n总点数: %d\n总时间: %.2fs\nZ轴范围: [%.3f, %.3f] m', ...
                         length(t), toc(startTime), min(xyTraj(3,:)), max(xyTraj(3,:)));
    set(info_text, 'String', final_stats);
end

function [xyTrajectory, t] = forward_kinematics(jointTrajectories, robot, varargin)
    p = inputParser;
    addRequired(p, 'jointTrajectories', @isnumeric);
    addRequired(p, 'robot');
    addParameter(p, 'SampleTime', [], @(x)isempty(x) || x>0);
    addParameter(p, 'Plot', false, @islogical);
    parse(p, jointTrajectories, robot, varargin{:});
    
    nSamples = size(jointTrajectories, 2);
    if isempty(p.Results.SampleTime)
        t = linspace(0, 1, nSamples);
    else
        t = 0:p.Results.SampleTime:(nSamples-1)*p.Results.SampleTime;
    end
    
    xyTrajectory = zeros(3, nSamples);
    
    for k = 1:nSamples
        q = jointTrajectories(:,k);
        
        T = robot.fkine(q);
        
        if isa(T, 'SE3')
            pos = T.t;
        elseif isnumeric(T) && size(T,1) == 4 && size(T,2) == 4
            pos = T(1:3,4);
        else
            error('无法解析fkine返回值');
        end
        
        xyTrajectory(:,k) = pos;
    end
    
    if p.Results.Plot
        figure('Name', '末端执行器轨迹', 'Position', [200, 200, 1200, 800]);
        
        % 3D轨迹
        subplot(2,2,1);
        plot3(xyTrajectory(1,:), xyTrajectory(2,:), xyTrajectory(3,:), 'b-', 'LineWidth', 2);
        hold on;
        plot3(xyTrajectory(1,1), xyTrajectory(2,1), xyTrajectory(3,1), 'go', 'MarkerSize', 8);
        plot3(xyTrajectory(1,end), xyTrajectory(2,end), xyTrajectory(3,end), 'ro', 'MarkerSize', 8);
        
        xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
        title('三维轨迹');
        grid on;
        
        % ===== 修复：设置合理的坐标轴范围 =====
        x_range = [min(xyTrajectory(1,:)), max(xyTrajectory(1,:))];
        y_range = [min(xyTrajectory(2,:)), max(xyTrajectory(2,:))];
        z_range = [min(xyTrajectory(3,:)), max(xyTrajectory(3,:))];
        
        % SCARA机械臂的典型工作空间
        x_margin = max(0.05, (x_range(2) - x_range(1)) * 0.1);
        y_margin = max(0.05, (y_range(2) - y_range(1)) * 0.1);
        z_margin = max(0.01, (z_range(2) - z_range(1)) * 0.1);
        
        xlim([x_range(1) - x_margin, x_range(2) + x_margin]);
        ylim([y_range(1) - y_margin, y_range(2) + y_margin]);
        zlim([z_range(1) - z_margin, z_range(2) + z_margin]);
        
        legend('轨迹', '起点', '终点');
        
        % X-Y平面投影
        subplot(2,2,2);
        plot(xyTrajectory(1,:), xyTrajectory(2,:), 'b-', 'LineWidth', 2);
        hold on;
        plot(xyTrajectory(1,1), xyTrajectory(2,1), 'go', 'MarkerSize', 8);
        plot(xyTrajectory(1,end), xyTrajectory(2,end), 'ro', 'MarkerSize', 8);
        xlabel('X (m)'); ylabel('Y (m)');
        title('X-Y平面投影');
        grid on; axis equal;
        
        % 坐标-时间曲线
        subplot(2,2,[3,4]);
        plot(t, xyTrajectory(1,:), 'r-', 'LineWidth', 2, 'DisplayName', 'X');
        hold on;
        plot(t, xyTrajectory(2,:), 'g-', 'LineWidth', 2, 'DisplayName', 'Y');
        plot(t, xyTrajectory(3,:), 'b-', 'LineWidth', 2, 'DisplayName', 'Z');
        xlabel('时间 (s)'); ylabel('位置 (m)');
        title('坐标-时间曲线');
        grid on; legend;
        
        % 显示实际工作范围
        fprintf('轨迹范围统计:\n');
        fprintf('X: [%.3f, %.3f] m (范围: %.3f m)\n', x_range(1), x_range(2), x_range(2)-x_range(1));
        fprintf('Y: [%.3f, %.3f] m (范围: %.3f m)\n', y_range(1), y_range(2), y_range(2)-y_range(1));
        fprintf('Z: [%.3f, %.3f] m (范围: %.3f m)\n', z_range(1), z_range(2), z_range(2)-z_range(1));
    end
end
