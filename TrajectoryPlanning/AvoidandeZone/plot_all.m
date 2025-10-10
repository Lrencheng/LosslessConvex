function plot_all(params,results)
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

    %绘制数据图表
    % 获取最后一次迭代的数据
    final_iter = results.act_iterations;
    t_plot=linspace(0,params.tf,params.N+1);
    % 创建新图形窗口
    figure;

    % 绘制Heading Angle (theta)
    subplot(2,1,1);
    % z_history中第三个状态是sin(theta)
    theta_sin = results.z_history{final_iter}(3,:);
    % 转换为角度制
    theta_deg = asind(theta_sin);
    plot(t_plot,theta_deg', 'r-', 'LineWidth', 2);
    xlabel('时间步');
    ylabel('航向角 (度)');
    title('航向角变化曲线');
    grid on;

    % 绘制Angular Velocity (theta_d)
    subplot(2,1,2);
    % u_history中第二个输入是角速度theta_d
    omega = results.u_history{final_iter}(2,:)./results.u_history{final_iter}(1,:);
    omega=omega.*(180/pi);
    plot(t_plot,omega', 'b-', 'LineWidth', 2);
    xlabel('时间步');
    ylabel('角速度 (rad/s)');
    title('角速度变化曲线');
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