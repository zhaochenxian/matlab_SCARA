%% 轨迹规划实验主程序（修复版 - 解决警告和精度问题）
%不带轨迹执行
clc; clear; close all;

%% ===== 1. 机器人模型创建 =====
fprintf('=============== 轨迹规划实验 ===============\n');

% 创建机器人模型（使用实验1的模型）
my_robot = create_robot_model();

% 修正起始点和目标点的Z坐标（确保在工作空间内）
P1 = [0.4, 0, 0.10];     % 起始点（降低Z坐标）
P2 = [0, -0.4, 0.10];    % 目标点（降低Z坐标）

fprintf('起始点 P1: [%.3f, %.3f, %.3f] m\n', P1);
fprintf('目标点 P2: [%.3f, %.3f, %.3f] m\n', P2);

% 验证工作空间
check_workspace_validity(P1, P2, my_robot);

%% ===== 2. 关节空间轨迹规划 =====
fprintf('\n=============== 关节空间轨迹规划 ===============\n');
fprintf('注意：关节空间轨迹规划是在关节空间进行线性插值，末端执行器轨迹会是曲线\n');

% 构造位姿矩阵
T_start = transl(P1) * rpy2tr(0, 0, 0);
T_end = transl(P2) * rpy2tr(0, 0, 0);

% 逆运动学求解（改进版）
q_start = improved_inverse_kinematics(P1(1), P1(2), P1(3), my_robot);
q_end = improved_inverse_kinematics(P2(1), P2(2), P2(3), my_robot);

fprintf('起始关节角: [%.3f, %.3f, %.3f, %.3f]\n', q_start);
fprintf('终止关节角: [%.3f, %.3f, %.3f, %.3f]\n', q_end);

% 时间参数设置
t_total = 5.0;  % 总运动时间
dt = 0.05;      % 时间步长
t = 0:dt:t_total;

% 1) 一次线性轨迹规划
[q_linear, qd_linear, qdd_linear] = linear_trajectory_planning(q_start, q_end, t);

% 2) 二次抛物线轨迹规划（起始和终止速度为0）
[q_parabolic, qd_parabolic, qdd_parabolic] = parabolic_trajectory_planning(q_start, q_end, t);

% 3) 三次轨迹规划（起始和终止加速度为0）
[q_cubic, qd_cubic, qdd_cubic] = cubic_trajectory_planning(q_start, q_end, t);

% 分别绘制每种轨迹规划方法（新增）
plot_single_trajectory_method(t, q_linear, qd_linear, qdd_linear, my_robot, '关节空间一次线性轨迹规划', 'linear');
plot_single_trajectory_method(t, q_parabolic, qd_parabolic, qdd_parabolic, my_robot, '关节空间二次抛物线轨迹规划', 'parabolic');
plot_single_trajectory_method(t, q_cubic, qd_cubic, qdd_cubic, my_robot, '关节空间三次轨迹规划', 'cubic');

% 对比图（修改后的原函数）
plot_trajectory_comparison(t, q_linear, qd_linear, qdd_linear, ...
    q_parabolic, qd_parabolic, qdd_parabolic, q_cubic, qd_cubic, qdd_cubic, my_robot);

%% ===== 3. 笛卡尔空间轨迹规划 =====
fprintf('\n=============== 笛卡尔空间轨迹规划 ===============\n');
fprintf('注意：笛卡尔空间轨迹规划是在任务空间进行，末端执行器轨迹是直线或圆形\n');

% 1) 空间直线运动
fprintf('1. 执行空间直线运动...\n');
[line_traj, line_t] = cartesian_line_trajectory(P1, P2, t_total, dt);
execute_cartesian_trajectory(line_traj, line_t, my_robot, '笛卡尔空间直线轨迹');

% 2) 空间画整圆运动（调整参数确保在工作空间内）
fprintf('2. 执行空间画圆运动...\n');
circle_center = [0.3, -0.2, 0.08];  % 圆心（调整Z坐标）
circle_radius = 0.08;               % 半径（减小半径）
[circle_traj, circle_t] = cartesian_circle_trajectory(circle_center, circle_radius, t_total, dt);
execute_cartesian_trajectory(circle_traj, circle_t, my_robot, '笛卡尔空间圆形轨迹');

fprintf('\n=============== 轨迹规划实验完成 ===============\n');

%% ===== 改进的逆运动学函数（修复精度问题） =====
function q = improved_inverse_kinematics(x, y, z, robot)
    % 改进的逆运动学求解（提高精度）
    try
        a2 = robot.links(2).a;
        a3 = robot.links(3).a;
        base_height = robot.links(1).d;
        offset_y = robot.links(2).d;  % 第二关节的偏移
    catch
        a2 = 0.467;
        a3 = 0.4005;
        base_height = 0.152;
        offset_y = 0.05;
    end
    
    % 考虑第二关节的偏移，调整目标位置
    z_adj = z - offset_y;  % 调整Z坐标
    
    % 计算水平距离
    r = sqrt(x^2 + y^2);
    
    % 检查是否在工作空间内
    r_max = a2 + a3;
    r_min = abs(a2 - a3);
    
    if r > r_max
        fprintf('警告: 目标点距离过远 (r=%.3f > %.3f)，调整到边界\n', r, r_max);
        scale = r_max * 0.95 / r;
        x = x * scale;
        y = y * scale;
        r = sqrt(x^2 + y^2);
    elseif r < r_min
        fprintf('警告: 目标点距离过近 (r=%.3f < %.3f)，调整到边界\n', r, r_min);
        if r < 1e-6
            r = r_min;
            x = r_min;
            y = 0;
        else
            scale = r_min / r;
            x = x * scale;
            y = y * scale;
            r = sqrt(x^2 + y^2);
        end
    end
    
    % 计算第二关节角
    cos_theta2 = (r^2 - a2^2 - a3^2) / (2*a2*a3);
    cos_theta2 = max(-1, min(1, cos_theta2));
    
    sin_theta2 = sqrt(1 - cos_theta2^2);
    theta2 = atan2(sin_theta2, cos_theta2);
    
    % 计算第一关节角
    k1 = a2 + a3*cos_theta2;
    k2 = a3*sin_theta2;
    theta1 = atan2(y, x) - atan2(k2, k1);
    
    % 计算移动关节位移（考虑偏移）
    d3 = base_height - z_adj;
    
    % 第四关节角保持为0
    theta4 = 0;
    
    q = [theta1; theta2; d3; theta4];
    
    % 检查关节限位并调整
    for j = 1:length(q)
        if q(j) < robot.qlim(j,1) || q(j) > robot.qlim(j,2)
            q_old = q(j);
            q(j) = max(robot.qlim(j,1), min(q(j), robot.qlim(j,2)));
            if j == 3
                fprintf('关节%d超限，从%.3fm调整到%.3fm\n', j, q_old, q(j));
            else
                fprintf('关节%d超限，从%.1f°调整到%.1f°\n', j, rad2deg(q_old), rad2deg(q(j)));
            end
        end
    end
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

%% ===== 工作空间验证函数（改进版） =====
function check_workspace_validity(P1, P2, robot)
    % 检查点是否在工作空间内
    fprintf('验证工作空间...\n');
    
    try
        q1 = improved_inverse_kinematics(P1(1), P1(2), P1(3), robot);
        fprintf('P1可达，关节角: [%.3f, %.3f, %.3f, %.3f]\n', q1);
        
        % 验证正向运动学
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
        
        % 验证正向运动学
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

%% ===== 其余函数保持不变 =====
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

function execute_cartesian_trajectory(trajectory, t, robot, title_str)
    fprintf('开始执行%s...\n', title_str);
    
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
    
    visualize_cartesian_trajectory(trajectory, joint_trajectory, t, title_str, robot);
    
    fprintf('%s执行完成\n', title_str);
end

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
    fprintf('关节限位:\n');
    fprintf('  关节1: [%.1f°, %.1f°]\n', rad2deg(my_robot.qlim(1,:)));
    fprintf('  关节2: [%.1f°, %.1f°]\n', rad2deg(my_robot.qlim(2,:)));
    fprintf('  关节3: [%.3f m, %.3f m]\n', my_robot.qlim(3,:));
    fprintf('  关节4: [%.1f°, %.1f°]\n', rad2deg(my_robot.qlim(4,:)));
end

% 保留原来的逆运动学函数作为备用
function q = custom_inverse_kinematics(x, y, z, robot)
    try
        a2 = robot.links(2).a;
        a3 = robot.links(3).a;
        base_height = robot.links(1).d;
    catch
        a2 = 0.467;
        a3 = 0.4005;
        base_height = 0.152;
    end
    
    r = sqrt(x^2 + y^2);
    r_max = a2 + a3;
    r_min = abs(a2 - a3);
    
    if r > r_max
        fprintf('警告: 目标点距离过远 (r=%.3f > %.3f)，调整到边界\n', r, r_max);
        scale = r_max * 0.95 / r;
        x = x * scale;
        y = y * scale;
        r = sqrt(x^2 + y^2);
    elseif r < r_min
        fprintf('警告: 目标点距离过近 (r=%.3f < %.3f)，调整到边界\n', r, r_min);
        if r < 1e-6
            r = r_min;
            x = r_min;
            y = 0;
        else
            scale = r_min / r;
            x = x * scale;
            y = y * scale;
            r = sqrt(x^2 + y^2);
        end
    end
    
    cos_theta2 = (r^2 - a2^2 - a3^2) / (2*a2*a3);
    cos_theta2 = max(-1, min(1, cos_theta2));
    
    sin_theta2 = sqrt(1 - cos_theta2^2);
    theta2 = atan2(sin_theta2, cos_theta2);
    
    k1 = a2 + a3*cos_theta2;
    k2 = a3*sin_theta2;
    theta1 = atan2(y, x) - atan2(k2, k1);
    
    d3 = base_height - z;
    theta4 = 0;
    
    q = [theta1; theta2; d3; theta4];
    
    for j = 1:length(q)
        if q(j) < robot.qlim(j,1) || q(j) > robot.qlim(j,2)
            q_old = q(j);
            q(j) = max(robot.qlim(j,1), min(q(j), robot.qlim(j,2)));
            if j == 3
                fprintf('关节%d超限，从%.3fm调整到%.3fm\n', j, q_old, q(j));
            else
                fprintf('关节%d超限，从%.1f°调整到%.1f°\n', j, rad2deg(q_old), rad2deg(q(j)));
            end
        end
    end
end
