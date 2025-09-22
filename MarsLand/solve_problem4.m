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
    %滑翔角约束矩阵
    S=[0 1 0;
        0 0 1;];
    c=[-tand(theta_alt) 0 0];
    %% 定义results返回值
    num_iterations = N_max - N_min + 1;
    results = repmat(struct('N', 0, 't_f', 0, 'cost', 0, 'p_opt', [], ...
                       'feasible', false, 'M', {}, 'T', {}, ...
                       'pos', {}, 'vel', {}), num_iterations, 1);
    time_vec=zeros(4,num_iterations);
    for N=N_min:N_max
        %测量总求解时间
        total_start=tic;
        cal_start=tic;
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
            %TODO:
            % 优化Psi计算：使用预计算C矩阵
            if k > 1
                num_blocks = min(k-1, N);
                Psi{k} = [AB{num_blocks:-1:1}, zeros(7, 4*(N - num_blocks))];
            else
                Psi{k} = zeros(7, 4*N);
            end
            %优化代码
            %TODO:debug
            %Upsilon
            if k<=N
                Upsilon{k}=[zeros(4,4*(k-1)),eye(4),zeros(4,4*(N-k))];
            else
                Upsilon{k}=zeros(4,4*N);
            end
            %debug
            %xi 重力+初始条件
            xi{k}=zeros(7,1);
            xi{k}=Phi{k}*x0+Lambda{k}*[g_mars;0];
        end
        
        %测量时间
        cal_time=toc(cal_start);
        build_start=tic;
        % ==================== 优化问题求解 ====================
        % 定义决策变量
        p = sdpvar(4*N,1,'full');  % 控制参数，现在有 N 段
        % ==================== 约束构建 ====================
        constraints = [];
        % 终止约束
        x_f = xi{N+1} + Psi{N+1} * p(:);
        constraints = [constraints, norm(x_f(1:3) - rf) <= 1e-2];
        constraints = [constraints, norm(x_f(4:6) - vf) <= 1e-2];
        u_f = Upsilon{N}(1:3, :) * p(:);  % 末端控制加速度
        sigma_f = Upsilon{N}(4, :) * p(:); % 末端松弛变量
        constraints = [constraints, u_f == sigma_f * nf]; % 推力方向约束

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
        % 向量化高度约束和质量约束
        constraints =[constraints,all_z >= zk_min_vec];
        constraints =[constraints,all_heights(:) >= 0];
        constraints =[constraints,all_z(:) <= zk_max_vec(:)];
        %{
        constraints = [constraints,...
            all_heights >= 0,...                    %保证火箭的高度始终大于0
            all_z >= zk_min_vec,...                 % 所有zk_min约束
            all_z <= zk_max_vec];                % 所有zk_max约束
        %}
            
        % 向量化推力指向约束
        % constraints = [constraints, v' * all_u >= gamma * all_sigma];

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
        % 滑翔角约束（如果启用）
        % for k = 1:N+1
        %     r_k = all_states(1:3, k);
        %     constraints = [constraints, norm(S*r_k) + c*r_k <= 0];
        % end
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
        
        build_time=toc(build_start);
        % 求解问题
        diagnostics = optimize(constraints,objective, options);

        %结束总时间测量
        total_time=toc(total_start);
        optimal_time=total_time-build_time-cal_time;
        fprintf('求解该问题的总时间为：%f\n',total_time);
        fprintf('预处理时间：%f\n',cal_time);
        fprintf('建模时间：%f\n',build_time);
        fprintf('优化时间为：%f\n',optimal_time);
        time_vec(:,N-N_min+1)=[cal_time;build_time;optimal_time;total_time];
        % 存储结果
        results(N-N_min+1).N = N;
        results(N-N_min+1).feasible = false;
        if diagnostics.problem == 0
            results(N-N_min+1).tf=N*dt;
            results(N-N_min+1).cost = value(objective);
            results(N-N_min+1).p_opt = value(p);
            results(N-N_min+1).feasible = true;
            fprintf('  Feasible solution found. Cost: %.2f\n', value(objective));
            fprintf('  Feasible solution found. use fuel: %.2f\n', m_wet*(1-exp(-alpha*value(objective))));
        else
            results(N-N_min+1).cost = Inf;
            results(N-N_min+1).feasible = false;
            fprintf('  No feasible solution found.\n');
        end
    end
    assignin('base','time',time_vec);
    % ==================== 寻找最优结果 ====================
    optimal_result=findbest(results,rocket);
end