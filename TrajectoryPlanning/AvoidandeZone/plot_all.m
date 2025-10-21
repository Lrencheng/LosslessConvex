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
    % 验证Assumption 2
    index1=fix(13/params.dt);
    index2=fix(18.6/params.dt);
    t_assump2=t_plot(index1:index2);
    x_assump2=x_curve(index1:index2);
    y_assump2=y_curve(index1:index2);
    isactive=zeros(index2-index1+1,1);
    for i=index1:index2
        isactive(i-index1+1)=isactive_assump2(x_curve(i),y_curve(i),params);
    end
    figure;
    plot(t_assump2, isactive, 'ro', 'MarkerSize', 6);
    title('约束激活验证');
    xlim([12,18]);
    ylim([-1.2,1.2]);
% 获取当前坐标轴范围
xlim_val = get(gca, 'XLim');
ylim_val = get(gca, 'YLim');

% 在图的右上角添加文本
text(xlim_val(2)*0.92, ylim_val(2)*-0.7, '1:点在边界外');
text(xlim_val(2)*0.92, ylim_val(2)*-0.8, '0:点在边界上');
text(xlim_val(2)*0.92, ylim_val(2)*-0.9, '-1:点在边界内');
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

function isactive=isactive_assump2(x,y,params)
    xc=params.obstacles(3).xc;
    yc=params.obstacles(3).yc;
    a=params.obstacles(3).a;
    b=params.obstacles(3).b;
    flag=1-(x/a)^2-(y/b)^2;

    if abs(flag)<=10e-5
        isactive=0;%约束激活
    elseif flag>10e-5
        isactive=-1;%点在边界内
    else 
        isactive=1;%点在边界外
    end

end