function plotData(optimal_result)
    % 获取最佳解的参数
    N_best = optimal_result.N;
    t_f_best = optimal_result.tf;
    M_best = optimal_result.M;
    T_best = optimal_result.T;
    position_data = optimal_result.pos;  % 获取位置数据
    velocity_data = optimal_result.vel;  % 获取速度数据

    % 构建时间向量
    t_plot = linspace(0, t_f_best, N_best+1)';
    % 提取推力比
    thrust_ratio = optimal_result.thrust_ratio;
    % 绘制推力水平(T/T_max)随时间变化图
    if optimal_result.id~=-1
        figure;
        plot(t_plot(1:end-1), thrust_ratio(1:end-1), 'b-', 'LineWidth', 1.5);
        xlabel('时间 t (s)');
        ylabel('T/Tmax');
        title('最佳推力水平随时间变化');
        grid on;
        xlim([0, t_f_best]);
        ylim([0, 1]);
        
        % 添加参考线 - 使用转义字符处理特殊符号
        hold on;
        yline(0.3, 'r--', '最小推力 (30% Tmax)');
        yline(0.8, 'r--', '最大推力 (80% Tmax)');
        hold off;
        
        % 显示统计信息
        fprintf('推力统计信息:\n');
        fprintf('平均推力比: %.3f\n', mean(thrust_ratio));
        fprintf('最大推力比: %.3f\n', max(thrust_ratio));
        fprintf('最小推力比: %.3f\n', min(thrust_ratio));
        
        % 其余绘图代码保持不变...
        % 绘制位置随时间变化图
        figure;
        subplot(3,1,1);
        plot(t_plot, position_data(1,:), 'r-', 'LineWidth', 1.5);
        ylabel('X位置 (m)');
        title('位置和速度随时间变化');
        grid on;
        
        subplot(3,1,2);
        plot(t_plot, position_data(2,:), 'g-', 'LineWidth', 1.5);
        ylabel('Y位置 (m)');
        grid on;
        
        subplot(3,1,3);
        plot(t_plot, position_data(3,:), 'b-', 'LineWidth', 1.5);
        ylabel('Z位置 (m)');
        xlabel('时间 t (s)');
        grid on;
        
        % 绘制速度随时间变化图
        figure;
        subplot(3,1,1);
        plot(t_plot, velocity_data(1,:), 'r-', 'LineWidth', 1.5);
        ylabel('X速度 (m/s)');
        title('速度分量随时间变化');
        grid on;
        
        subplot(3,1,2);
        plot(t_plot, velocity_data(2,:), 'g-', 'LineWidth', 1.5);
        ylabel('Y速度 (m/s)');
        grid on;
        
        subplot(3,1,3);
        plot(t_plot, velocity_data(3,:), 'b-', 'LineWidth', 1.5);
        ylabel('Z速度 (m/s)');
        xlabel('时间 t (s)');
        grid on;
        
        % 绘制三维轨迹图
        figure;
        plot3(position_data(1,:), position_data(2,:), position_data(3,:), 'b-', 'LineWidth', 1.5);
        hold on;
        scatter3(position_data(1,1), position_data(2,1), position_data(3,1), 8, 'go', 'filled');  % 起点
        scatter3(position_data(1,end), position_data(2,end), position_data(3,end), 8, 'ro', 'filled');  % 终点
        xlabel('X位置 (m)');
        ylabel('Y位置 (m)');
        zlabel('Z位置 (m)');
        title('三维飞行轨迹');
        legend('轨迹', '起点', '终点');
        grid on;
        
        % 显示统计信息
        fprintf('位置统计信息:\n');
        fprintf('起始位置: [%.1f, %.1f, %.1f] m\n', position_data(1,1), position_data(2,1), position_data(3,1));
        fprintf('终止位置: [%.1f, %.1f, %.1f] m\n', position_data(1,end), position_data(2,end), position_data(3,end));
        
        fprintf('速度统计信息:\n');
        fprintf('起始速度: [%.1f, %.1f, %.1f] m/s\n', velocity_data(1,1), velocity_data(2,1), velocity_data(3,1));
        fprintf('终止速度: [%.1f, %.1f, %.1f] m/s\n', velocity_data(1,end), velocity_data(2,end), velocity_data(3,end));
        
    else
        fprintf('未找到可行解，无法绘制推力图。\n');
    end
end