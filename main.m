%function main()
    % Add YALMIP to path (if not already added)
    addpath(genpath('yalmip'));
    addpath(genpath('ecos'));
    %parameters:
    g_mars=[-3.7114;0;0];
    m_dry=1505;
    m_wet=1905;
    m_fuel=m_wet-m_dry;
    Isp=225;%比冲
    T_max=3100;
    T1=0.3*T_max;
    T2=0.8*T_max;
    phi_cant=27;
    n_engines=6;%引擎数
    % 计算推力
    rho1 = n_engines * T1 * cosd(phi_cant);  % Lower thrust bound [N]
    rho2 = n_engines * T2 * cosd(phi_cant);  % Upper thrust bound [N]
    % 燃油质量消耗参数
    g_earth = 9.807;    % Earth gravity [m/s^2]
    alpha = 1 / (Isp * g_earth * cosd(phi_cant));
    %% 初始条件
    r0=[1500;0;2000];
    r0_dot=[-75;0;100];
    z0 = log(m_wet);
    x0 = [r0; r0_dot; z0];
    %% 终止条件
    rf=[0;0;0];
    vf=[0;0;0];

    %% 时间参数
    dt=1;%单位[s]
    t_min=(m_wet-m_dry)*norm(r0_dot)/rho2;
    t_max=m_fuel/(alpha*rho1);
    N_min=fix(t_min/dt)+1;
    N_max=fix(t_max/dt);
    %覆写变量
    N_min=70;
    N_max=75;

    %% 预先计算状态空间方程参数
    A_c=[zeros(3,3),eye(3,3),zeros(3,1);
            zeros(3,3),zeros(3,3),zeros(3,1);
            zeros(1,3),zeros(1,3),0;];
    B_c=[zeros(3,3),zeros(3,1);
        eye(3,3),zeros(3,1);
        zeros(1,3),-alpha;];
    A=expm(A_c*dt);
    B=integral(@(tau) expm(A_c * (dt - tau)) * B_c, 0, dt, 'ArrayValued', true);
    %% 迭代计算
    for N=N_min:N_max
        % 定义优化变量
         t_f = N * dt;
         fprintf('尝试 N=%d, t_f=%.1f s\n', N, t_f);
        % 构建时间向量
        t = linspace(0, t_f, N+1)';
        % 计算数值计算矩阵
        Phi=cell(N+1,1);
        Psi=cell(N+1,1);
        Lambda=cell(N+1,1);
        Upsilon=cell(N+1,1);
        xi=cell(N+1,1);
        for k=1:N+1  %N+1个时间点
            %Phi
            Phi{k}=zeros(7,7);
            Phi{k}=A^(k-1);
            %Lamda
            Lambda{k}=zeros(7,4);
            if k>=2
                Lambda{k}=Lambda{k-1}+A^(k-2)*B;
            end
            %Psi
            Psi{k} = zeros(7, 4*N);
            for j=1:min(k-1,N)
                Psi{k}(:,1+4*(j-1):4*j)=A^(k-j-1)*B;
            end
            %Upsilon
            if k<=N
                Upsilon{k}=[zeros(4,4*(k-1)),eye(4),zeros(4,4*(N-k))];
            else
                Upsilon{k}=zeros(4,4*N);
            end
            %xi 重力+初始条件
            xi{k}=zeros(7,1);
            xi{k}=Phi{k}*x0+Lambda{k}*[g_mars;0];
        end
        % ==================== 优化问题求解 ====================
        % 定义决策变量
        p = sdpvar(4*N,1,'full');  % 控制参数，现在有 N 段
        % ==================== 约束构建 ====================
        constraints = [];
        % 终止约束
        x_f = xi{N+1} + Psi{N+1} * p(:);
        constraints = [constraints, norm(x_f(1:3)-rf)<=5];    % Final position
        constraints = [constraints, norm(x_f(4:6)-vf)<=5];    % Final velocity
        for k=1:N
            %推力限幅约束问题1：松弛变量约束
            if k<=N
                sigma_k=Upsilon{k}(4,:)*p(:);
                u_k=Upsilon{k}(1:3,:)*p(:);
            else
                sigma_k=0;
                u_k=zeros(3,1);
            end
            constraints=[constraints,norm(u_k)<=sigma_k];
            %计算状态变量
            state_vec=zeros(7,1);%1:3->r,4:6->v,7->z
            state_vec=xi{k}+Psi{k}*p(:);
            r_k=state_vec(1:3);%位矢
            v_k=state_vec(4:6);%速度
            z_k=state_vec(7);%ln(m)
            %推力限幅约束问题2：松弛变量范围
            z0_k=log(m_wet-alpha*rho2*t(k));
            u1_k=rho1*exp(-z0_k);
            u2_k=rho2*exp(-z0_k);
            sigma_k_min=u1_k*(1-(z_k-z0_k)+(z_k-z0_k)^2/2);
            sigma_k_max=u2_k*(1-(z_k-z0_k));
            %constraints=[constraints,sigma_k<=sigma_k_max];
            %constraints=[constraints,sigma_k>=sigma_k_min];
            constraints=[constraints,sigma_k>=sigma_k_min];
            constraints=[constraints,sigma_k<=sigma_k_max];
            %保证质量在物理规则范围内：
            zk_min=log(m_wet-alpha*rho2*t(k));
            zk_max=log(m_wet-alpha*rho1*t(k));
            constraints=[constraints,z_k>=zk_min];
            constraints=[constraints,z_k<=zk_max];
        end
         % ==================== 计算代价函数 ====================
         % Minimize fuel consumption: J = ∫σ dt ≈ Σσ_k * Δt
        objective = 0;
        for k = 1:N
            sigma_k = Upsilon{k}(4, :) * p(:);
            objective = objective + sigma_k * dt;
        end
        % ==================== 求解优化问题 ====================
        % 设置ECOS求解器
        options = sdpsettings('solver', 'ecos', 'verbose', 0, 'cachesolvers', 1);
        
        % 求解问题
        diagnostics = optimize(constraints,objective, options);
        % 存储结果
        results(N-N_min+1).N = N;
        results(N-N_min+1).t_f = t_f;
        if diagnostics.problem == 0
            results(N-N_min+1).cost = value(objective);
            results(N-N_min+1).p_opt = value(p);
            results(N-N_min+1).feasible = true;
             % 计算数值解并存储
            M_values = cell(N+1,1);
            T_values = cell(N+1,1);
            pos_values=cell(N+1,1);
            vel_values=cell(N+1,1);
            sigma_min=zeros(N+1,1);
            sigma_max=zeros(N+1,1);
            sigma=zeros(N+1,1);
            tx = linspace(0, t_f, N+1)';
            for k=1:N+1
                state_vec_num = double(xi{k} + Psi{k} * value(p));
                pos_values{k}=state_vec_num(1:3);
                vel_values{k}=state_vec_num(4:6);

                z_k_num = state_vec_num(7);
                M_values{k} = exp(z_k_num);
                z0k=log(m_wet-alpha*rho2*tx(k));
                sigma_min(k)=rho1*exp(-z0k)*(1-(z_k_num-z0k)+(z_k_num-z0k)^2/2);
                sigma_max(k)=rho2*exp(-z0k)*(1-(z_k_num-z0k));
                sigma(k)=Upsilon{k}(4,:) * value(p);
                if k <= N
                    u_k_num = double(Upsilon{k}(1:3,:) * value(p));
                    T_values{k} = norm(u_k_num) * M_values{k};
                else
                    T_values{k} = 0;
                end
                results(N-N_min+1).M = M_values;      % 存储质量
                results(N-N_min+1).T = T_values;      % 存储推力
                results(N-N_min+1).pos = pos_values;  % 存储位置
                results(N-N_min+1).vel = vel_values;  % 存储速度
            end
            fprintf('  Feasible solution found. Cost: %.2f\n', value(objective));
            fprintf('  Feasible solution found. use fuel: %.2f\n', m_wet*(1-exp(-alpha*value(objective))));
        else
            results(N-N_min+1).cost = Inf;
            results(N-N_min+1).feasible = false;
            fprintf('  No feasible solution found.\n');
        end
    end
    % ==================== 寻找最优结果 ====================
    best_cost=inf;
    best_idx=-1;
    for i=1:N_max-N_min+1
        if results(i).feasible==true&&results(i).cost<best_cost
            best_cost=results(i).cost;
            best_idx=i;
        end
    end
    if best_idx ~= -1
         % 获取最佳解的参数
        N_best = results(best_idx).N;
        t_f_best = results(best_idx).t_f;
        M_best = results(best_idx).M;
        T_best = results(best_idx).T;
        pos_best = results(best_idx).pos;  % 获取位置数据
        vel_best = results(best_idx).vel;  % 获取速度数据
        
        % 构建时间向量
        t_plot = linspace(0, t_f_best, N_best+1)';
        
        % 提取位置和速度数据
        position_data = zeros(3, N_best+1);
        velocity_data = zeros(3, N_best+1);
        for k = 1:N_best+1
            position_data(:, k) = pos_best{k};  % 从cell数组中提取数据
            velocity_data(:, k) = vel_best{k};  % 从cell数组中提取数据
        end
        
        % 提取推力比
        thrust_ratio = zeros(N_best, 1);
        for k = 1:N_best         
            thrust_ratio(k) = T_best{k} / (n_engines *T_max*cosd(phi_cant));
        end
        
        % 绘制推力水平(T/T_max)随时间变化图
        figure;
        plot(t_plot(1:end-1), thrust_ratio, 'b-', 'LineWidth', 1.5);
        xlabel('时间 t (s)');
        ylabel('T/T_{max}');
        title('最佳推力水平随时间变化');
        grid on;
        xlim([0, t_f_best]);
        ylim([0, 1]);
        
        % 添加参考线
        hold on;
        yline(0.3, 'r--', '最小推力 (30\% T_{max})');
        yline(0.8, 'r--', '最大推力 (80\% T_{max})');
        hold off;
        
        % 显示统计信息
        fprintf('推力统计信息:\n');
        fprintf('平均推力比: %.3f\n', mean(thrust_ratio));
        fprintf('最大推力比: %.3f\n', max(thrust_ratio));
        fprintf('最小推力比: %.3f\n', min(thrust_ratio));
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
        scatter3(position_data(1,1), position_data(2,1), position_data(3,1), 'go', 'filled', 'MarkerSize', 8);  % 起点
        scatter3(position_data(1,end), position_data(2,end), position_data(3,end), 'ro', 'filled', 'MarkerSize', 8);  % 终点
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
%end