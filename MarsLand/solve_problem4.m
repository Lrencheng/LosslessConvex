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
    N_min=rocket.N_min;
    N_max=rocket.N_max;
    dt=rocket.dt;
    n_engines=rocket.n_engines;
    T_max=rocket.T_max;
    phi_cant=rocket.phi_cant;
    A=rocket.A;
    B=rocket.B;
    %% 定义results返回值
    num_iterations = N_max - N_min + 1;
    results = repmat(struct('N', 0, 't_f', 0, 'cost', 0, 'p_opt', [], ...
                       'feasible', false, 'M', {}, 'T', {}, ...
                       'pos', {}, 'vel', {}), num_iterations, 1);

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
            constraints=[constraints,sigma_k>=sigma_k_min];
            constraints=[constraints,sigma_k<=sigma_k_max];
            %保证质量在物理规则范围内：
            zk_min=max(log(m_wet-alpha*rho2*t(k)),log(m_dry));
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
        if diagnostics.problem == 0
            results(N-N_min+1).N = N;
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
    optimal_result=findbest(results,rocket);
end