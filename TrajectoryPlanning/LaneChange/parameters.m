function parameters()
    tf=14;%总时间 单位[s]
    N=100;%离散点数
    dt=tf/N;%时间间隔
    max_iterations=5;%最大迭代次数
    USE_ITERATION=true;%使用迭代条件
    vel=7;%单位 m/s
    z0=[0;1.75;0];%初始条件
    zf=[100;5.25];%终点位置
    %最大角速度限制
    w_max_deg=15;
    w_max_rad=w_max_deg*pi/180;

    %目标函数参数
    k1=1;
    k2=1;
    k3=1;%k3=1  | k3=1000
    yc1=1.75;%x在[0,70)
    yc2=5.25;%x在[70,100]
    change_idx=70;
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
    USE_AVOIDANCE=true;
    obstacles = [
        struct('xc', 25, 'yc', 1, 'a', 5, 'b', 2.5);   % 障碍物1
    ];
    % 保存所有变量到para.mat
    filePath = fullfile(pwd, 'para.mat');
    save(filePath, 'tf', 'z0', 'zf','yc1','yc2','USE_ITERATION', ...
         'w_max_deg', 'w_max_rad','change_idx','USE_AVOIDANCE',...
         'N', 'k1', 'k2', 'k3',  'vel','Ac','Bc',...
         'A','B','dt','obstacles','max_iterations','epsilon_x','epsilon_y');
end