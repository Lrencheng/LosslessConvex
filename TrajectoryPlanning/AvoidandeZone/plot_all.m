function plot_all(params,results)
    N=params.N;
    t_plot=linspace(0,params.tf,N+1)';
    % 绘制初始轨迹进行验证
    
    init_curve=load('initial_state_profile.mat');
    figure;
    
    % 绘制障碍物
    plot_obstacles(params.obstacles);
    hold on;
    %绘制approach cone渐进线
    % 计算approach cone的边界线
    Angle1 = params.cone_horizon_deg - params.cone_deg/2;
    Angle2 = params.cone_horizon_deg + params.cone_deg/2;

    % 从目标点开始绘制两条边界线
    xf = params.zf(1);
    yf = params.zf(2);

    % 绘制cone的两条边界线
    line_length = 5; % 线的长度
    x1_end = xf - line_length * cosd(Angle1);
    y1_end = yf - line_length * sind(Angle1);
    x2_end = xf - line_length * cosd(Angle2);
    y2_end = yf - line_length * sind(Angle2);

    plot([xf, x1_end], [yf, y1_end], 'g--', 'LineWidth', 1.5, 'DisplayName', 'Approach Cone');
    hold on;
    plot([xf, x2_end], [yf, y2_end], 'g--', 'LineWidth', 1.5);
    hold on;

    
    % 绘制初始轨迹
    x_init=init_curve.x;
    y_init=init_curve.y;
    plot(x_init, y_init, 'k--', 'LineWidth', 2, 'DisplayName', '初始轨迹');
    plot(x_init(1), y_init(1), 'bo', 'MarkerSize', 8, 'DisplayName', '起点');
    plot(x_init(end), y_init(end), 'ro', 'MarkerSize', 8, 'DisplayName', '终点');
    grid on;
    hold on;

    % 绘制最后的迭代轨迹
    n=results.act_iterations;
    x_curve=results.x_history(:,n+1);
    y_curve=results.y_history(:,n+1);
    plot(x_curve, y_curve, 'r', 'LineWidth', 1.5);

    xlabel('x(m)');
    ylabel('y(m)');
    %绘制x,y数据
    % 创建新图形窗口
    figure;
    %绘制x,数据
    subplot(2,1,1);
    plot(t_plot,x_curve,'r', 'LineWidth', 1.5);
    ylabel('x(m)');
    xlabel('t(s)');
    grid on;
    %绘制x,数据
    subplot(2,1,2);
    plot(t_plot,y_curve,'r', 'LineWidth', 1.5);
    ylabel('y(m)');
    xlabel('t(s)');
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
    xlabel('Time (s)');
    ylabel('航向角 (deg)');
    title('航向角变化曲线');
    grid on;

    % 绘制Angular Velocity 
    subplot(2,1,2);
    % w=theta_d/theta_c
    omega = results.u_history{n}(2,:)./results.u_history{n}(1,:);
    omega=(omega.*(180/pi))';
    plot(t_plot,omega, 'b-', 'LineWidth', 2);
    hold on;
    plot(8.*ones(1, fix(params.tf)+1), 'r--');
    hold on;
    plot(-8.*ones(1, fix(params.tf)+1), 'r--');
    xlabel('Time (s)');
    ylabel('角速度 (rad/s)');
    title('角速度变化曲线');
    grid on;

    save('data.mat','theta_deg','omega');

    %绘制add_history
    figure
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