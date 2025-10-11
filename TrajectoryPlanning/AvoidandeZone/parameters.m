function parameters()
    tf=26.5;%总时间 单位[s]
    N=100;%离散点数
    dt=tf/N;%时间间隔
    max_iterations=5;%最大迭代次数
    vel=1;%单位 m/s
    z0=[0;0;0];%初始条件
    zf=[25;6];%终点位置
    % approach cone约束参数
    cone_deg=6;
    cone_horizon_deg=20;
    Inside_cone_percent=0.2;%在cone内的点数占总点数的百分比

    w_max_deg=8;
    w_max_rad=w_max_deg*pi/180;

    %目标函数参数
    k1=100;
    k2=100;
    k3=1;   
    yc=-1;%保证y-yc不为0
    
    %迭代截止条件
    epsilon_x=0.1;
    epsilon_y=0.1;
    %状态空间参数
    Ac=[0 0 0;
       0 0 vel;
       0 0 0;];
    Bc=[vel 0;0 0;0 1;];
    A=expm(Ac*dt);
    B=integral(@(tau) expm(Ac * (dt - tau)) * Bc,...
               0, dt, 'ArrayValued', true);

    %定义障碍物参数
    obstacles = [
        struct('xc', 5, 'yc', -2.2, 'a', 1, 'b', 1.8);   % 障碍物1
        struct('xc', 7, 'yc', 2, 'a', 2, 'b', 2.2);     % 障碍物2  
        struct('xc', 16, 'yc', 1, 'a', 3, 'b', 3); % 障碍物3
    ];
    % 保存所有变量到para.mat
    filePath = fullfile(pwd, 'para.mat');
    save(filePath, 'tf', 'z0', 'zf', 'cone_deg', ...
         'cone_horizon_deg','w_max_deg', 'w_max_rad',...
         'N', 'k1', 'k2', 'k3', 'yc', 'vel','Inside_cone_percent',...
         'A','B','dt','obstacles','max_iterations','epsilon_x','epsilon_y');
end