function optimal_result=solve_problem4(rocket)
    %%参数读取
    g_mars=rocket.g_mars;
    m_dry=rocket.m_dry;
    m_wet=rocket.m_wet;
    rho1=rocket.rho1;
    rho2=rocket.rho2;
    alpha=rocket.alpha;
    x0=rocket.x0;
    rf=rocket.rf;
    vf=rocket.vf;
    nf=rocket.nf;
    v=rocket.v;
    gamma=rocket.gamma;
    N_min=rocket.N_min;
    N_max=rocket.N_max;
    dt=rocket.dt;
    n_engines=rocket.n_engines;
    T_max=rocket.T_max;
    phi_cant=rocket.phi_cant;
    A=rocket.A;
    B=rocket.B;
    theta_alt=rocket.theta_alt;
    CONSTRAINTS_FINAL_THRUST_DRECT=rocket.CONSTRAINTS_FINAL_THRUST_DRECT;
    CONSTRAINTS_HEIGHT=rocket.CONSTRAINTS_HEIGHT;            
    CONSTRAINTS_ALL_THRUST_DRECT=rocket.CONSTRAINTS_ALL_THRUST_DRECT; 
    CONSTRAINTS_GLIDE=rocket.CONSTRAINTS_GLIDE;             
    %滑翔角约束矩阵
    S=[0 1 0;
        0 0 1;];
    c=[-tand(theta_alt) 0 0];
    %% 定义results返回值
    num_iterations = N_max - N_min + 1;
    results = repmat(struct('N', 0, 't_f', 0, 'cost', 0, 'p_opt', [], ...
                       'feasible', false, 'M', {}, 'T', {}, ...
                       'pos', {}, 'vel', {}), num_iterations, 1);
    solver_time=zeros(num_iterations,2);
    for N=N_min:N_max
        %% A的幂次序列-简化计算
        A_powers = cell(N+1, 1);
        A_powers{1} = eye(size(A,1));
        for i = 2:N+1
            A_powers{i} = A_powers{i-1} * A;
        end
        % 预计算C矩阵用于Psi
        AB = cell(N, 1);
        for i = 1:N
            AB{i} = A_powers{i} * B;
        end
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
            Phi{k}=A_powers{k};
            %Lamda
            Lambda{k}=zeros(7,4);
            if k>=2
                Lambda{k}=Lambda{k-1}+A_powers{k-1}*B;
            end
            if k > 1
                num_blocks = min(k-1, N);
                Psi{k} = [AB{num_blocks:-1:1}, zeros(7, 4*(N - num_blocks))];
            else
                Psi{k} = zeros(7, 4*N);
            end
            if k<=N
                Upsilon{k}=[zeros(4,4*(k-1)),eye(4),zeros(4,4*(N-k))];
            else
                Upsilon{k}=zeros(4,4*N);
            end
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
        constraints = [constraints, norm(x_f(1:3) - rf) <= 1e-5];
        constraints = [constraints, norm(x_f(4:6) - vf) <= 1e-5];
        u_f = Upsilon{N}(1:3, :) * p(:);  % 末端控制加速度
        sigma_f = Upsilon{N}(4, :) * p(:); % 末端松弛变量
        %预计算所有的sigma和u
        all_u=sdpvar(3,N);
        all_sigma=sdpvar(1,N);
        for j=1:N
            all_u(1:3,j)=Upsilon{j}(1:3, :) * p(:);
            all_sigma(1,j)=Upsilon{j}(4, :) * p(:);
        end

        %推力限幅约束问题1：松弛变量约束
        for k = 1:N
            constraints = [constraints, norm(all_u(:,k)) <= all_sigma(1,k)];
        end
        %预计算所有状态变量
        all_states = sdpvar(7, N+1);
        for k = 1:N+1
            all_states(:, k) = xi{k} + Psi{k} * p(:);
        end
        % 提取所有高度和z变量
        all_heights = all_states(1, :);  % 所有时间步的高度
        all_z = all_states(7, :);        % 所有时间步的z变量

        % 预计算所有zk_min和zk_max
        zk_min_vec = arrayfun(@(k) log(m_wet - alpha*rho2*t(k)), 1:N+1);
        zk_max_vec = arrayfun(@(k) log(m_wet - alpha*rho1*t(k)), 1:N+1);
        %质量约束
        constraints =[constraints,all_z >= zk_min_vec];
        constraints =[constraints,all_z<= zk_max_vec];
        
        %推力限幅约束问题2：松弛变量范围
        %非线性约束，无法使用向量化约束
        for k = 2:N+1
            z0_k = log(m_wet - alpha*rho2*t(k));
            z_k = all_z(k);
            u1_k = rho1 * exp(-z0_k);
            u2_k = rho2 * exp(-z0_k);
            sigma_k_min = u1_k * (1 - (z_k - z0_k) + (z_k - z0_k)^2/2);
            sigma_k_max = u2_k * (1 - (z_k - z0_k));
            if k > 1
                sigma_k = all_sigma(k-1);
                constraints = [constraints, sigma_k >= sigma_k_min, sigma_k <= sigma_k_max];
            end
        end
        %===== 额外约束 ====
        %末端推力方向约束
        if CONSTRAINTS_FINAL_THRUST_DRECT==true
            constraints = [constraints, u_f == sigma_f * nf]; %末端推力方向约束
        end
        %高度>=0约束
        if CONSTRAINTS_HEIGHT==true
            constraints =[constraints,all_heights(:) >= 0];
        end
        % 滑翔角约束
        if CONSTRAINTS_GLIDE==true
            for k = 1:N+1
                r_k = all_states(1:3, k);
                constraints = [constraints, norm(S*r_k) + c*r_k <= 0];
            end
        end
        % 向量化推力指向约束
        if CONSTRAINTS_ALL_THRUST_DRECT==true
            constraints = [constraints, v' * all_u >= gamma * all_sigma];
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
        options = sdpsettings('solver', 'ECOS', 'verbose', 0, 'cachesolvers', 1);
        % 求解问题
        diagnostics = optimize(constraints,objective, options);
        %记录时间
        solver_time(N-N_min+1,1)=N;
        % 存储结果
        results(N-N_min+1).N = N;
        results(N-N_min+1).feasible = false;
        if diagnostics.problem == 0
            results(N-N_min+1).tf=N*dt;
            results(N-N_min+1).cost = value(objective);
            results(N-N_min+1).p_opt = value(p);
            results(N-N_min+1).feasible = true;
            solver_time(N-N_min+1,2)=diagnostics.solvertime;
            fprintf('  Feasible solution found.\n');
            fprintf('solvertime:%.2f\n',solver_time(N-N_min+1,2));
            fprintf('use fuel:%.2f\n',m_wet*(1-exp(-alpha*value(objective))));
        else
            results(N-N_min+1).cost = Inf;
            results(N-N_min+1).feasible = false;
            fprintf('  No feasible solution found.\n');
        end
    end
    assignin('base','solvertime',solver_time);
    % ==================== 寻找最优结果 ====================
    optimal_result=findbest(results,rocket);
end