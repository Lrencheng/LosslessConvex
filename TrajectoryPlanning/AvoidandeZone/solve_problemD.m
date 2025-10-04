function results=solve_problemD(params)
    dt=params.dt;
    N=params.N;

    act_iterations=1;%真实迭代次数
    %记录迭代历史
    x_history=zeros(N+1,params.max_iterations+1);
    y_history=zeros(N+1,params.max_iterations+1);
    cost_history=zeros(params.max_iterations+1);%代价函数值
    for k=1:params.max_iterations
        [x_new,y_new,cost_new]=solve_convex_problem(params);
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
function [x,y,cost]=solve_convex_problem(params);
    
end




