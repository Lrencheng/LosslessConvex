function optimal_result=findbest(results,rocket)
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
    % ==================== 寻找最优结果 ====================
    best_cost=inf;
    best_idx=-1;
    for i=1:N_max-N_min+1
        if results(i).feasible==true&&results(i).cost<best_cost
            best_cost=results(i).cost;
            best_idx=i;
        end
    end
    %% 错误处理
    if best_idx==-1
       error('未找到可行解，无法绘制推力图');
    end
    %%返回最优结果
    optimal_result=results(best_idx);
    optimal_result.id=best_idx;
    N=optimal_result.N;
    tf=optimal_result.tf;
    % 构建时间向量
    t = linspace(0, tf, N+1)';
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
    % 计算数值解并存储
    M_values = zeros(1,N+1);%质量
    Tc_values=zeros(3,N+1);%三维净推力
    Tnet_values = zeros(1,N+1);%净推力幅值
    pos_values=zeros(3, N+1);%位置
    vel_values=zeros(3, N+1);%速度
    acc_values=zeros(3,N+1);%加速度
    theta_values=zeros(1,N+1);%发动机推力与重力夹角角度
    thrust_ratio_values=zeros(N+1,1);
    for k=1:N+1
        state_vec_num = double(xi{k} + Psi{k} * optimal_result.p_opt);
        pos_values(:,k)=state_vec_num(1:3);
        vel_values(:,k)=state_vec_num(4:6);
        z_k_num = state_vec_num(7);
        S=[0 1 0;0 0 1];
        M_values(k) = exp(z_k_num);
        z0k=log(m_wet-alpha*rho2*t(k));
        sigma_min(k)=rho1*exp(-z0k)*(1-(z_k_num-z0k)+(z_k_num-z0k)^2/2);
        sigma_max(k)=rho2*exp(-z0k)*(1-(z_k_num-z0k));
        sigma(k)=Upsilon{k}(4,:) * optimal_result.p_opt;

        u_k_num = double(Upsilon{k}(1:3,:) * optimal_result.p_opt);
        Tc_values(:,k)=u_k_num.*M_values(k);
        Tnet_values(k) = norm(u_k_num) * M_values(k);
        theta_values(k)=atan2d(norm(u_k_num(2:3)),u_k_num(1));
    end
    thrust_ratio_values(:) = Tnet_values(:)./(n_engines *T_max*cosd(phi_cant));
    acc_values(2:3,:)=Tc_values(2:3,:)./repmat(M_values, 2, 1);
    acc_values(1,:)=Tc_values(1,:)./M_values(1,:)+g_mars(1);
    optimal_result.M = M_values;      % 存储质量
    optimal_result.Tnet = Tnet_values;% 存储净推力
    optimal_result.Tc = Tc_values;% 存储三维推力
    optimal_result.pos = pos_values;  % 存储位置
    optimal_result.vel = vel_values;  % 存储速度
    optimal_result.acc = acc_values; % 存储加速度
    optimal_result.thrust_ratio=thrust_ratio_values;%油门
    optimal_result.theta=theta_values;%角度
end

