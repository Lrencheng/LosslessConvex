parameters();
params=load('para.mat');

%获得初始轨迹
[x_init,y_init]=initialize_trajectory(params);

%迭代
%有障碍物
fprintf('>>有障碍物迭代：\n');
params.USE_AVOIDANCE=true;
results1=solve_problemD(x_init,y_init,params);
%无障碍物
fprintf('>>无障碍物迭代：\n');
params.USE_AVOIDANCE=false;
results2=solve_problemD(x_init,y_init,params);
%绘图
plot_all(params,results1,results2);