function parameters()
    tf=26.5;%总时间 单位[s]
    N=100;%离散点数
    dt=tf/N;
    z0=[0;0;0];%起始位置
    zf=[25;6];%终点位置
    % approach cone:达到终点的角度要求
    cone_deg=6;
    cone_horizon_deg=20;

    w_max_deg=8;
    w_max_red=w_max_deg*pi/180;


    %目标函数参数
    k1=100;
    k2=100;
    k3=1;   
    yc=-1;%保证y-yc不为0
    epsilon=[0.1;0.1];
    
    %状态空间参数
    A=[0 0 0;
       0 0 vel;
       0 0 0;];
    B=[v 0;0 0;0 1;];

    vel=1;%单位 m/s
    %定义障碍物参数
    obstacles = [
        struct('xc', 5, 'yc', -2.2, 'a', 2, 'b', 1);   % 障碍物1
        struct('xc', 7, 'yc', 2, 'a', 2, 'b', 1.5);     % 障碍物2  
        struct('xc', 15.2, 'yc', 1, 'a', 3, 'b', 3); % 障碍物3
    ];
    % 保存所有变量到para.mat
    save('para.mat', 'tf', 'z0', 'zf', 'cone_deg', ...
         'cone_horizon_deg','w_max_deg', 'w_max_red',...
         'N', 'k1', 'k2', 'k3', 'yc', 'epsilon', 'vel',...
         'A','B','dt','obstacles');
end