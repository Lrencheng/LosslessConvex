function results=solve_problemD(x_init,y_init,params)
    dt=params.dt;
    N=params.N;

    act_iterations=1;%真实迭代次数
    %记录迭代历史
    x_history=zeros(N+1,params.max_iterations+1);
    y_history=zeros(N+1,params.max_iterations+1);
    cost_history=zeros(params.max_iterations+1);%代价函数值
    %迭代历史初始化：初始轨迹
    x_history(:,1)=x_init;
    y_history(:,1)=y_init;
    for k=1:params.max_iterations
        x_prev=x_history(:,k);
        y_prev=y_history(:,k);
        [x_new,y_new,cost_new]=solve_convex_problem(params,x_prev,y_prev);
        x_history(:,k)=x_new;
        y_history(:,k)=y_new;
        cost_history(k)=cost_new;

        %设置迭代截止条件
        if abs(x_new(end)-params.zf(1))<=params.max_deltax && ...
            abs(y_new(end)-params.zf(2))<=params.max_deltay
            act_iterations=k;
            break;
        end
    end
    results.x_history=x_history;
    results.y_history=y_history;
    results.cost_history=cost_history;
    results.act_iterations=act_iterations;
end
function [x,y,cost]=solve_convex_problem(params,x_prev,y_prev);
    % ==================== 优化问题建模 ====================
    constraints=[];
    %定义优化变量
    z=sdpvar(3,N+1);    %状态变量：x,y,theta_sine
    u=sdpvar(2,N+1);    %控制输入: theta_cosine,theta_d
    N=params.N;
    dt=params.dt;
    %状态变量初始化
    z(:,1)=params.z0;
    for i=1:N
        z(:,i+1)=params.A*z(:,i)+params.B*u(:,i);
    end
    Wmax=[-params.w_max_rad 1;-params.w_max_rad -1];
    for i=1:N+1
        constraints=[constraints,Wmax*u(:,i)<=0];%|u2|<=wmax*|u1|
        constraints=[constraints,u(1,i)^2+z(3,i)^2<=1];%u1^2+z3^2<=1
    end
    % 非凸约束线性化
    for i=1:N+1
        prev.x=x_prev(i);
        prev.y=y_prev(i);
        curr.x=z(1,i);
        curr.y=z(2,i);
        for j=1:length(params.obstacles)
            [grad_x,grad_y,dconst]=linearlize_gnc(prev,...
                                                  params.obstacles(j).xc,params.obstacles(j).yc,...
                                                  paramsparams.obstacles(j).a,params.obstacles(j).b ...
                                                 );
            constraints=[constraints,grad_x*curr.x+grad_y*curr.y+dconst<=0];
        end
    end
    % ==================== 计算代价函数 ====================

end
function [grad_x,grad_y,dconst]=linearlize_gnc(prev,xc,yc,a,b)
    grad_x=-2*(prev.x-xc)/a^2;
    grad_y=-2*(prev.y-yc)/b^2;
    grad_val=1-(prev.x-xc)^2/a^2-(prev.y-yc)^2/b^2;
    dconst=grad_val-grad_x*prev.x-grad_y*prev.y;
end




