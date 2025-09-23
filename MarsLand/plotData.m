function plotData(optimal_result)
    % 获取最佳解的参数
    N_best = optimal_result.N;
    t_f_best = optimal_result.tf;
    M_best = optimal_result.M;
    Tnet_best = optimal_result.Tnet;
    Tc_best=optimal_result.Tc;
    position_data = optimal_result.pos;  % 获取位置数据
    velocity_data = optimal_result.vel;  % 获取速度数据
    accel_data  = optimal_result.acc;    % 获取加速度数据
    theta_data = optimal_result.theta;

    % 构建时间向量
    t_plot = linspace(0, t_f_best, N_best+1)';
    % 提取推力比
    thrust_ratio = optimal_result.thrust_ratio;
    if optimal_result.id~=-1
        figure;
        % 绘制位置随时间变化图
        subplot(3,2,1);
        plot(t_plot, position_data(3,:), 'r-', 'LineWidth', 1.5);
        hold on;       
        plot(t_plot, position_data(2,:), 'g-', 'LineWidth', 1.5);
        hold on;
        plot(t_plot, position_data(1,:), 'b-', 'LineWidth', 1.5);
        legend('posX', 'posY', 'posZ', 'Location', 'best');  % 添加图例及位置参数
        ylabel('position,m');
        xlabel('t,s');
        grid on;
         % 绘制速度随时间变化图
        subplot(3,2,2);
        plot(t_plot, velocity_data(3,:), 'r-', 'LineWidth', 1.5);
        hold on;
        plot(t_plot, velocity_data(2,:), 'g-', 'LineWidth', 1.5);
        hold on;
        plot(t_plot, velocity_data(1,:), 'b-', 'LineWidth', 1.5);
        legend('velX', 'velY', 'velZ', 'Location', 'best');  % 添加图例及位置参数
        ylabel('velocity,m/s');
        xlabel('t,s');
        grid on;
        %绘制加速度曲线图
        subplot(3,2,3);
        plot(t_plot, accel_data(3,:), 'r-', 'LineWidth', 1.5);
        hold on;
        plot(t_plot, accel_data(2,:), 'g-', 'LineWidth', 1.5);
        hold on;
        plot(t_plot, accel_data(1,:), 'b-', 'LineWidth', 1.5);
        legend('accX', 'accY', 'accZ', 'Location', 'best');  % 添加图例及位置参数
        ylabel('acceleration,m/s');
        xlabel('t,s');
        grid on;
        %绘制推力曲线图
        subplot(3,2,4);
        plot(t_plot, Tc_best(3,:), 'r-', 'LineWidth', 1.5);
        hold on;
        plot(t_plot, Tc_best(2,:), 'g-', 'LineWidth', 1.5);
        hold on;
        plot(t_plot, Tc_best(1,:), 'b-', 'LineWidth', 1.5);
        legend('TX', 'TY', 'TZ', 'Location', 'best');  % 添加图例及位置参数
        ylabel('Thrust,N');
        xlabel('t,s');
        grid on;
        % 绘制推力水平(T/T_max)随时间变化图
        subplot(3,2,5);
        plot(t_plot(1:end-1), thrust_ratio(1:end-1), 'b-', 'LineWidth', 1.5);
        xlabel('时间 t (s)');
        ylabel('T/Tmax');
        title('最佳推力水平随时间变化');
        grid on;
        xlim([0, t_f_best]);
        ylim([0, 1]);       
         % 添加参考线 - 使用转义字符处理特殊符号
        hold on;
        yline(0.3, 'r--', '最小推力 (30% Tmin)');
        yline(0.8, 'r--', '最大推力 (80% Tmax)');
        hold off;
        % 显示统计信息
        fprintf('最优轨迹时间tf:%d\n',t_f_best);
        %滑翔角
        subplot(3,2,6);
        plot(t_plot(1:end-1),theta_data(1:end-1),'g-', 'LineWidth', 1.5);
        ylabel('theta,deg');
        title('滑翔角');
        grid on;
        % 绘制三维轨迹图
        figure;
        plot3(position_data(3,:), position_data(2,:), position_data(1,:), 'b-', 'LineWidth', 1.5);
        hold on;
        scatter3(position_data(3,1), position_data(2,1), position_data(1,1), 8, 'go', 'filled');  % 起点
        scatter3(position_data(3,end), position_data(2,end), position_data(1,end), 8, 'ro', 'filled');  % 终点
        xlabel('X位置 (m)');
        ylabel('Y位置 (m)');
        zlabel('Z位置 (m)');
        title('三维飞行轨迹');
        legend('轨迹', '起点', '终点');
        grid on;
        % 显示统计信息
        fprintf('最优轨迹时间tf:%d\n',t_f_best);
        fprintf('位置统计信息:\n');
        fprintf('起始位置: [%.1f, %.1f, %.1f] m\n', position_data(1,1), position_data(2,1), position_data(3,1));
        fprintf('终止位置: [%.1f, %.1f, %.1f] m\n', position_data(1,end), position_data(2,end), position_data(3,end));
        
        fprintf('速度统计信息:\n');
        fprintf('起始速度: [%.1f, %.1f, %.1f] m/s\n', velocity_data(1,1), velocity_data(2,1), velocity_data(3,1));
        fprintf('终止速度: [%.1f, %.1f, %.1f] m/s\n', velocity_data(1,end), velocity_data(2,end), velocity_data(3,end));
        
        % 绘制求解时间柱状图
        solver_time = evalin('base', 'solvertime');
        if ~isempty(solver_time)
            figure;
            bar(solver_time(:, 1), solver_time(:, 2));
            xlabel('N');
            ylabel('solvertime,s');
            title('求解器运行时间随时间步数 N 的变化');
            grid on;
        end
    else
        fprintf('未找到可行解，无法绘制推力图。\n');
    end
end