function plot_all(params,results1,results2)
    N=params.N;
    t_plot=linspace(0,params.tf,params.N+1)';
    % 绘制初始轨迹进行验证
    
    init_curve=load('initial_state_profile.mat');

    figure;
    %绘制道路LaneSide
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
    n1=results1.act_iterations;
    x_curve1=results1.x_history(:,n1+1);
    y_curve1=results1.y_history(:,n1+1);
    h1 = plot(x_curve1, y_curve1, 'r', 'LineWidth', 2.5,'DisplayName','with obstacle avoidance');

    % 绘制无障碍物的迭代轨迹
    n2=results2.act_iterations;
    x_curve2=results2.x_history(:,n2+1);
    y_curve2=results2.y_history(:,n2+1);
    h2 = plot(x_curve2, y_curve2, 'b--', 'LineWidth', 2.5,'DisplayName','without obstacle avoidance');

    % 只显示指定的图例项
    legend([h1, h2]);
    xlim([-1, 105]);
    ylim([-4, 8]);
    xlabel('x(m)');
    ylabel('y(m)');
    
    %绘制heading angle和angular velocity数据
    figure;
    % 绘制Heading Angle (theta)
    subplot(2,1,1);
    % 有障碍物数据
    theta_sin1 = results1.z_history{n1}(3,:);
    theta_deg1 = asind(theta_sin1)';
    plot(t_plot,theta_deg1, 'r-', 'LineWidth', 2.5);
    hold on;
    % 无障碍物数据
    theta_sin2 = results2.z_history{n2}(3,:);
    theta_deg2 = asind(theta_sin2)';
    plot(t_plot,theta_deg2, 'b--', 'LineWidth', 2.5);
    xlabel('Time (s)');
    ylabel('Heading angle (deg)');
    grid on;

    % 绘制Angular Velocity 
    subplot(2,1,2);
    % w=theta_d/theta_c
    %有障碍物数据
    omega1 = results1.u_history{n1}(2,:)./results1.u_history{n1}(1,:);
    omega1=(omega1.*(180/pi))';
    p1=plot(t_plot,omega1, 'r-', 'LineWidth', 2.5,'DisplayName','with obstacle avoidance');
    hold on;
    %无障碍物数据
    omega2 = results2.u_history{n2}(2,:)./results2.u_history{n2}(1,:);
    omega2=(omega2.*(180/pi))';
    p2=plot(t_plot,omega2, 'b--', 'LineWidth', 2.5,'DisplayName','without obstacle avoidance');
    legend([p1,p2]);
    xlabel('Time (s)');
    ylabel('Angular velocity (rad/s)');
    grid on;

    save('data.mat','theta_deg1','omega1','theta_deg2','omega2');
    %绘制add_history
    figure
    % 绘制add_history散点图
    add_data = results1.add(:, n1);  % 获取最后一次迭代的add_history数据
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