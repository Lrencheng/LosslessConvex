%parameters();
params=load('para.mat');

%获得初始轨迹
[x_init,y_init]=initialize_trajectory(params);

%迭代
history=solve_problemD(x_init,y_init,params);

%绘图
plot_all(params,history)