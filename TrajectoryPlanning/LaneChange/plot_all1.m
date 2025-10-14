function plot_all1(params,results)
    N=params.N;
    t_plot=linspace(0,params.tf,params.N+1)';
    % 绘制初始轨迹进行验证
    
    init_curve=load('initial_state_profile.mat');
    figure;
    %绘制道路LaneSide
    subplot(2,1,1);
    x_road = [-1, 110];
    y_road_upper = [7, 7];
    y_road_lower = [0, 0];
    plot(x_road, y_road_upper, 'k-', 'LineWidth', 3);
    hold on;
    plot(x_road, y_road_lower, 'k-', 'LineWidth', 3);
    % 绘制障碍物
    plot_obstacles(params.obstacles);

    % 绘制初始轨迹
    x_init=init_curve.x;
    y_init=init_curve.y;
    plot(x_init, y_init, 'k--', 'LineWidth', 2);
    plot(x_init(1), y_init(1), 'bo', 'MarkerSize', 8);
    plot(x_init(end), y_init(end), 'ro', 'MarkerSize', 8);
    grid on;

    % 绘制有障碍物的迭代轨迹
    n=results.act_iterations;
    x_curve1=results.x_history(:,n+1);
    y_curve1=results.y_history(:,n+1);
    h1 = plot(x_curve1, y_curve1, 'b--', 'LineWidth', 2.5,'DisplayName','without obstacle avoidance');

    % 只显示指定的图例项
    legend(h1);
    xlim([-1, 105]);
    ylim([-4, 8]);
    xlabel('x(m)');
    ylabel('y(m)');

    %绘制add_history
    subplot(2,1,2);
    % 绘制add_history散点图
    add_data = results.add(:, n);  % 获取最后一次迭代的add_history数据
    % 绘制蓝色空心圆散点图（1:N的数据点）
    scatter(1:N, add_data(1:N), 'o', 'MarkerFaceColor', 'none', 'MarkerEdgeColor', 'blue', 'LineWidth', 1.5);
    hold on;
    % 绘制y=1的红色实线作为对照
    plot(1:N, ones(1, N), 'r-', 'LineWidth', 1.5);
    % 设置坐标轴范围
    xlim([1, 100]);
    ylim([0, 1.3]);

    xlabel('Time (s)');
    ylabel('u_1^2 + z_3^2');
    legend('= u_1^2 + z_3^2', '= 1', 'Location', 'best');
    grid on;
    
    %绘制数据图表
    % 创建新图形窗口
    figure;
    % 绘制Heading Angle (theta)
    subplot(2,1,1);
    % z_history中第三个状态是sin(theta)
    theta_sin = results.z_history{n}(3,:);
    % 转换为角度制
    theta_deg = asind(theta_sin)';
    plot(t_plot,theta_deg, 'r-', 'LineWidth', 2);
    xlabel('时间步');
    ylabel('航向角 (度)');
    title('航向角变化曲线');
    grid on;

    % 绘制Angular Velocity 
    subplot(2,1,2);
    % w=theta_d/theta_c
    omega = results.u_history{n}(2,:)./results.u_history{n}(1,:);
    omega=(omega.*(180/pi))';
    plot(t_plot,omega, 'b-', 'LineWidth', 2);
    xlabel('时间步');
    ylabel('角速度 (rad/s)');
    title('角速度变化曲线');
    grid on;

    save('data.mat','theta_deg','omega');
end
%障碍物绘图函数
function plot_obstacles(obstacles)
    % 绘制椭圆形障碍物
    % 输入: obstacles - 包含障碍物信息的结构体数组
    %       每个结构体包含: xc, yc (中心坐标), a, b (长轴和短轴半径)
    
    for i = 1:length(obstacles)
        % 创建椭圆参数方程的参数
        theta = linspace(0, 2*pi, 100);
        
        % 计算椭圆上的点
        x = obstacles(i).xc + obstacles(i).a * cos(theta);
        y = obstacles(i).yc + obstacles(i).b * sin(theta);
        
        % 绘制椭圆
        plot(x, y, 'b-', 'LineWidth', 2);
        hold on;
        plot(obstacles(i).xc, obstacles(i).yc,...
            'k+', 'MarkerSize', 8, 'MarkerFaceColor','r',...
            'LineWidth',1);
        hold on;
    end
end
