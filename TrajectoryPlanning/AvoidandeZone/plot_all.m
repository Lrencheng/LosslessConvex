function plot_all(params,results)
    % 绘制初始轨迹进行验证
    
    init_curve=load('initial_state_profile.mat');
    figure;
    
    % 绘制障碍物
    plot_obstacles(params.obstacles);
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
        plot(x, y, 'r-', 'LineWidth', 2);
        hold on;
    end
end