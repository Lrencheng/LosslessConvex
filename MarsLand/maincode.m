%坐标系建立：[i,j,k]
%i:高度
%j,k:水平位移

rocket.g_mars=[-3.7114;0;0];
rocket.m_dry=1505;
rocket.m_wet=1905;
rocket.m_fuel=rocket.m_wet-rocket.m_dry;
rocket.Isp=225;%比冲
rocket.T_max=3100;
rocket.T1=0.3*rocket.T_max;
rocket.T2=0.8*rocket.T_max;
rocket.phi_cant=27;
rocket.n_engines=6;%引擎数

% 计算推力
rocket.rho1 = rocket.n_engines * rocket.T1 * cosd(rocket.phi_cant);  % Lower thrust bound [N]
rocket.rho2 = rocket.n_engines * rocket.T2 * cosd(rocket.phi_cant);  % Upper thrust bound [N]
% 燃油质量消耗参数
rocket.g_earth = 9.807;    % Earth gravity [m/s^2]
rocket.alpha = 1 / (rocket.Isp * rocket.g_earth * cosd(rocket.phi_cant));

% 滑翔角约束
rocket.theta_alt=86;%单位：[deg]
%% 初始条件
rocket.r0=[1500;0;2000];
rocket.r0_dot=[-75;0;100];
rocket.z0 = log(rocket.m_wet);
rocket.x0 = [rocket.r0; rocket.r0_dot;rocket.z0];

%% 终止条件
rocket.rf=[0;0;0];
rocket.vf=[0;0;0];

%% 初始和结束的推力方向约束
rocket.n0=[1;0;0];
rocket.nf=[1;0;0];

%% 时间参数
rocket.dt=1;%单位[s]
rocket.t_min=(rocket.m_wet-rocket.m_dry)*norm(rocket.r0_dot)/rocket.rho2;
rocket.t_max=rocket.m_fuel/(rocket.alpha*rocket.rho1);
rocket.N_min=fix(rocket.t_min/rocket.dt)+1;
rocket.N_max=fix(rocket.t_max/rocket.dt);
rocket.N_min=78;
rocket.N_max=82;

%% 预先计算状态空间方程参数
rocket.A_c=[zeros(3,3),eye(3,3),zeros(3,1);
    zeros(3,3),zeros(3,3),zeros(3,1);
    zeros(1,3),zeros(1,3),0;];
rocket.B_c=[zeros(3,3),zeros(3,1);
    eye(3,3),zeros(3,1);
    zeros(1,3),-rocket.alpha;];
rocket.A=expm(rocket.A_c*rocket.dt);
rocket.B=integral(@(tau) expm(rocket.A_c * (rocket.dt - tau)) * ...
         rocket.B_c, 0, rocket.dt, 'ArrayValued', true);

%% 调用函数并结算
optimal_result=solve_problem4(rocket);
plotData(optimal_result);