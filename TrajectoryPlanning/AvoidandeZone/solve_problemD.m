function results=solve_problemD(x_init,y_init,params)
    dt=params.dt;
    N=params.N;

    act_iterations=1;%真实迭代次数
    %记录迭代历史
    z_history=cell(1,params.max_iterations+1);
    u_history=cell(1,params.max_iterations+1);
    x_history=zeros(N+1,params.max_iterations+1);
    y_history=zeros(N+1,params.max_iterations+1);
    cost_history=zeros(params.max_iterations);%代价函数值
    max_delta_x=zeros(params.max_iterations);
    max_delta_y=zeros(params.max_iterations);
    runtime=zeros(params.max_iterations);
    %迭代历史初始化：初始轨迹
    x_history(:,1)=x_init;
    y_history(:,1)=y_init;
    %test:验证u1^2+z3^2是否为1
    add_history=zeros(N+1,params.max_iterations+1);
    for k=1:params.max_iterations
        act_iterations=k;
        x_prev=x_history(:,k);
        y_prev=y_history(:,k);
        %the core !!!
        solution=solve_convex_problem(params,x_prev,y_prev);

        if isnan(solution.cost)==true
            if k==1
                error('第一次迭代失败>>问题无解\n');
            else
                fprintf('第%d次迭代:Feasible solution was not found.\n',k);
            end
        else
            fprintf('第%d次迭代:Feasible solution was found.\n',k);
            fprintf('cost: %.2f.\n',solution.cost);
        end
        z_history{k}=solution.z;
        u_history{k}=solution.u;
        x_history(:,k+1)=solution.z(1,:);
        y_history(:,k+1)=solution.z(2,:);
        cost_history(k)=solution.cost;
        runtime(k)=solution.runtime;
        %test:
        add_history(:,k)=solution.add;
        %设置迭代截止条件
        max_delta_x(k)=max(abs(solution.z(1,:)-x_prev'));
        max_delta_y(k)=max(abs(solution.z(2,:)-y_prev'));
        if params.USE_ITERATION==true
            if max_delta_x(k)<=params.epsilon_x && ...
               max_delta_y(k)<=params.epsilon_y
                break;
            end
        end
    end
    %存储结果
    results.z_history=z_history;
    results.u_history=u_history;
    results.x_history=x_history;
    results.y_history=y_history;
    results.cost_history=cost_history;
    results.act_iterations=act_iterations;
    results.max_delta_x=max_delta_x;
    results.max_delta_y=max_delta_y;
    results.runtime=runtime;
    %test:
    results.add=add_history;
end
function solution=solve_convex_problem(params,x_prev,y_prev)
    N=params.N;
    dt=params.dt;
    % ==================== 优化问题建模 ====================
    constraints=[];
    %定义优化变量
    z=sdpvar(3,N+1);    %状态变量：x,y,theta_sine
    u=sdpvar(2,N+1);    %控制输入: theta_cosine,theta_d
    %状态变量初始化
    z(:,1)=params.z0;
    for i=1:N
        constraints=[constraints,z(:,i+1) == z(:,i)+dt*(params.Ac*z(:,i) + params.Bc*u(:,i))];
    end
    Wmax=[-params.w_max_rad 1;-params.w_max_rad -1];
    for i=1:N+1
        constraints=[constraints,Wmax*u(:,i)<=0];%|u2|<=wmax*|u1|
        constraints=[constraints,u(1,i)^2+z(3,i)^2<=1];%u1^2+z3^2<=1
        constraints=[constraints,u(1,i)>=0.03];%assumption 1:|theta|<=pi/2
    end

    %approach cone约束：
    Angle1=params.cone_horizon_deg-params.cone_deg/2;
    Angle2=params.cone_horizon_deg+params.cone_deg/2;
    n1=[-cosd(Angle1) sind(Angle1)];
    n2=[cosd(Angle2) -sind(Angle2)];
    start_idx=round((1-params.Inside_cone_percent)*(N+1));
    for i=start_idx:N+1
        rel_pos=[params.zf(2)-z(2,i);
                 params.zf(1)-z(1,i)];     %相对位置
        constraints=[constraints,n1*rel_pos<=0];
        constraints=[constraints,n2*rel_pos<=0];
    end

    % 非凸约束线性化：避障
    for i=1:N+1
        prev.x=x_prev(i);
        prev.y=y_prev(i);
        curr.x=z(1,i);
        curr.y=z(2,i);
        for j=1:length(params.obstacles)
            [grad_x,grad_y,dconst]=linearlize_gnc(prev,...
                                                  params.obstacles(j).xc,params.obstacles(j).yc,...
                                                  params.obstacles(j).a,params.obstacles(j).b ...
                                                 );
            constraints=[constraints,grad_x*curr.x+grad_y*curr.y+dconst<=0];
        end
    end
    % ==================== 计算代价函数 ====================
    %J=k1*|xN-xf|+k2*|yN-yf|+k3*∑(yi − yci )^2
    objective=0;
    for i=1:N+1
        objective=objective+params.dt*(z(2,i)-params.yc)^2;
    end
    objective=params.k1*abs(z(1,end)-params.zf(1))+...
              params.k2*abs(z(2,end)-params.zf(2))+...
              params.k3*objective;
    % ==================== 求解优化问题 ====================
    % 设置ECOS求解器
    options = sdpsettings('solver', 'ECOS', 'verbose', 0, 'cachesolvers', 1);
    % 求解问题
    diagnostics = optimize(constraints,objective, options);
    solution.runtime=diagnostics.solvertime;
    if diagnostics.problem == 0
        solution.cost=value(objective);
        solution.z=value(z);
        solution.u=value(u);
        %test:验证u1^2+z3^2是否为1
        solution.add=value(z(3,:)).^2+value(u(1,:)).^2;

    else
        solution.z = NaN(3, N+1);
        solution.u = NaN(2, N+1);
        solution.cost = NaN;
        %test:验证u1^2+z3^2是否为1
        solution.add=NaN(1,N+1);
    end
end
function [grad_x,grad_y,dconst]=linearlize_gnc(prev,xc,yc,a,b)
    grad_x=-2*(prev.x-xc)/a^2;
    grad_y=-2*(prev.y-yc)/b^2;
    grad_val=1-(prev.x-xc)^2/a^2-(prev.y-yc)^2/b^2;
    dconst=grad_val-grad_x*prev.x-grad_y*prev.y;
end




