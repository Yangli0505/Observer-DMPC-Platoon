clear;
clc;close all
f=@(i,j) (i-j)+i*(i==j);

%% 过程控制
doPic = true; %绘图
doSaveFig = true; %保存图像
doPicOptm = false; %绘图（优化过程）
isNonlinear = true; % 是否采用非线性动力学
doDesiredAccelerationInput = false; %领航车运行方式，true为期望加速度输入，false为实际加速度输入
doAccelerate = true; %领航车是否加速
doSinWave = false; %领航车是否进行正弦波动
doStep = false; %领航车阶跃输入
doHighwayloop = false; % 高速循环
doCityloop = false; % 城市循环
doCitySinLoop = false; % 城市循环
isLocalError = false; % 绘图时依据前车计算误差
isTimeHeadwayIncluded = true; % 绘图时是否将车头时距考虑进误差
isStable = true; %队列是否稳定
isRandom = false; %是否随机初始化
existModelMismatch = false; % 预测与控制模型是否不匹配
existDisturbance = false; % 控制输入扰动
useLinearController = false; % test，使用线性控制器
existInitialSpdErr = false; %与领航车是否存在初始速度误差

Ntopo = 6; %初始化前 Ntopo个拓扑： 1.PF 2.PLF 3.TPF 4.TPLF 5.BD 6.PF-failure 7.MF 8.MPF
IndexTopo = 3; %拓扑类型

%% 初始化
d0=20; %初始车距
vd=0; %初始车速
D0=[d0;0;0];%状态偏移
g=9.8;
p0=0;                           %领航车辆初始位移
v0=0;
a0=0;                           %领航车辆起始加速度
a1=1;

Nveh = 5; %车辆数
Gamma = 2;

Umin = 3; %最大减速度
Umax = 3; %最大加速度

Np = 10  ; % 预测步长10

Tstep=0.1; %仿真步长
Nstep=1000; %仿真步数
Tstep_hat=0.05;                 %观测仿真步长
Nstep_hat=2000;                %观测仿真步数

T=zeros(Nveh,Nstep);%计时

if ~isNonlinear % 线性动力学，匀质时滞
    tau = [0.5 0.5 0.5 0.5 0.5];
    tau_cap=tau;
else % 非线性动力学
    tau = 0.4+ 0.02 * [1:Nveh]';
end

%% 计算时间统计初始化
comp_time = zeros(Nveh, Nstep); % 存储每辆车每步的计算时间（Nveh辆车 × Nstep步）
comp_time_MCT = zeros(Nveh, 1); % 每辆车的最大计算时间（MCT）
comp_time_ACT = zeros(Nveh, 1); % 每辆车的平均计算时间（ACT）
total_comp_time = zeros(1, Nstep); % 每步所有车辆的总计算时间（可选）

%% Markov随机过程
% 定义转移率矩阵（Q矩阵）
Q = [ -2, 0.8, 0.8, 0.4;
    1.2, -2.4, 0.8, 0.4;
    0.4, 0.4, -1.2, 0.4;
    1.2, 0.8, 0.8, -2.8];

% 初始状态分布
initial_state = [0, 0, 1, 0];

% 初始化状态和时间
current_state = initial_state;

% 存储状态随时间的变化
states_over_time = zeros(Nstep_hat,4);
states_over_time(1,:) = current_state;

% 模拟CTMC
for n = 2:Nstep_hat
    current_state = current_state + Tstep * (current_state * Q);
    states_over_time(n,:) = current_state;
end

%定义图的状态
sigma = zeros(size(states_over_time,1),1);
sigma(1) = 3;

% 通信拓扑定义
M=cell(Ntopo,1); %邻接矩阵
P=cell(Ntopo,1); %牵引矩阵

clear Mtemp
Mtemp = zeros(Nveh,Nveh); %邻接矩阵
for i = 1:Ntopo
    M{i} = Mtemp;
    P{i} = Mtemp;
end
clear Mtemp

K = zeros(3,Nveh);%控制增益
A = cell(Nveh,1); %离散状态矩阵
B = cell(Nveh,1); %离散输入矩阵

Xtemp = zeros(3,Nstep);
X = cell(Nveh,1); %车辆状态
for i = 1:Nveh
    X{i} = Xtemp;
end
U = zeros(Nveh,Nstep); % 输入

p_terminal_save = zeros(Nveh,Nstep);
v_terminal_save = zeros(Nveh,Nstep);
a_terminal_save = zeros(Nveh,Nstep);
Xa = cell(Nveh,1); % 假设车辆状态
Xtemp = zeros(3,Np+1);
for i = 1:Nveh
    Xa{i} = Xtemp;
end
Ua = zeros(Nveh,Np+1);% 假设车辆输入

Xa_next = cell(Nveh,1); % 下一时刻假设车辆状态
for i = 1:Nveh
    Xa_next{i} = Xtemp;
end
Ua_next = zeros(Nveh,Np+1);  % 下一时刻假设车辆输入
Xa_last = cell(Nveh,1); % 上一时刻假设车辆状态
for i = 1:Nveh
    Xa_last{i} = Xtemp;
end
clear Xtemp

%  权重矩阵
RR = cell(Nveh,1);
FF = cell(Nveh,1);
GG = cell(Nveh,1);
SS = cell(Nveh,1);%SSi 惩罚车辆𝑖的预测状态与车辆i-1的假设状态之间的偏差（或邻居偏差）
Cost = zeros(Nveh,Nstep); % 预测的控制代价函数
ExitFlag = zeros(Nveh,Nstep); % 退出机制-控制
Cost_self_GG = 100*ones(Nveh,1);% 自偏移代价，权重GG，初始值取100
Tol = 1e-6;
Iter = 10000;

options = optimoptions('fmincon', ...
    'Display','off', ...
    'FunctionTolerance', 1e-4, ... % 替代TolFun，函数值容忍度
    'ConstraintTolerance', 1e-3, ... % 替代TolCon，约束容忍度
    'MaxIterations', 5000, ... % 替代MaxIter，最大迭代数
    'HessianApproximation', 'lbfgs', ... % L-BFGS近似海森矩阵（fmincon支持）
    'ScaleProblem', 'obj-and-constr', ... % 自动缩放目标函数和约束
    'Algorithm', 'sqp' ... % 指定求解算法（可选：interior-point, sqp, active-set等）
);

%% 动力学
for i=1:Nveh
    randp = [0.3517    0.8308    0.5853    0.5497    0.9172    0.2858    0.7572    0.7537    0.3804    0.5678]*2-1; % 随机初始位置误差
    randv = [0.0759    0.0540    0.5308    0.7792    0.9340    0.1299    0.5688    0.4694    0.0119    0.3371]*2-1; % 随机初始速度误差
    if isRandom
        X{i}(:,1)=[-d0*i + randp(i); vd + randv(i); 0];% 初始状态
    else
        X{i}(:,1)=[-d0*i; vd;0]; % 零误差初始状态
    end

    Ai=[1 Tstep 0;
        0 1   Tstep
        0 0     1];
    Bi=[0 0 Tstep]';
    
    Ki = [0.88628;2.27529;2.3637];
    K(:,i) = Ki';
    A{i} = Ai;
    B{i} = Bi;
    
    %多车辆情况下的权重系数
    for k = 1:Nveh
        GG{k} = diag([50,25,10]); 
        RR{k} = 0.1;
        FF{k} = diag([10 5 2]);
        SS{k} = diag([5 2.5 1]);
    end

    %权重系数
    GG{1} = diag([50,25,10]); 
    GG{2} = diag([50,25,10]); 
    GG{3} = diag([50,25,10]); 
    GG{4} = diag([50,25,10]); 
    GG{5} = diag([50,25,10]); 
    RR{1} = 0.1;
    RR{2} = 0.1;
    RR{3} = 0.1;
    RR{4} = 0.1;
    RR{5} = 0.1;
    FF{1} = diag([10 5 2]);
    FF{2} = diag([10 5 2]);
    FF{3} = diag([10 5 2]);
    FF{4} = diag([10 5 2]);
    FF{5} = 0*diag([5 2.5 1]);
    SS{1} = diag([5 2.5 1]);
    SS{2} = diag([5 2.5 1]);
    SS{3} = diag([5 2.5 1]);
    SS{4} = diag([5 2.5 1]);
    SS{5} = diag([5 2.5 1]);
end

% 初始化假设轨迹
for i = 1:Nveh
    Ai = A{i};
    Bi = B{i};
    for n = 1:Np+1
        if n == 1
            Xa{i}(:,n) = X{i}(:,1);
            Ua(i,n) = 0; % k=0时，假设输入为0  
        else
            Xa{i}(:,n) = Ai * Xa{i}(:,n-1) + Bi * Ua(i,n-1);
            Ua(i,n) = 0;
        end
    end
end

%% 观测器
X0_hat=zeros(3,Nstep_hat+Np); %领航车状态
X0_hat(:,1)=[p0;v0;a0];
for i=2:Nstep_hat+(Tstep/Tstep_hat)*Np
    if  i<=501
        X0_hat(1,i) = X0_hat(1,i-1) + X0_hat(2,i-1) * Tstep_hat;
        X0_hat(2,i) = X0_hat(2,i-1) + 1 * Tstep_hat;
        X0_hat(3,i) = 1;
    elseif i<=1001
        X0_hat(1,i) = X0_hat(1,i-1) + X0_hat(2,i-1) * Tstep_hat;
        X0_hat(2,i) = X0_hat(2,i-1);
        X0_hat(3,i) = 0;
    elseif i<=1201
        X0_hat(1,i) = X0_hat(1,i-1) + X0_hat(2,i-1) * Tstep_hat;
        X0_hat(2,i) = X0_hat(2,i-1) - 1.2 * Tstep_hat;
        X0_hat(3,i) = -1.2;
    else
        X0_hat(1,i) = X0_hat(1,i-1) + X0_hat(2,i-1) * Tstep_hat;
        X0_hat(2,i) = X0_hat(2,i-1);
        X0_hat(3,i) = 0;
    end
end

% 领航车状态矩阵
A0=[0 1 0;0 0 1;0 0 0];
% 初始观测状态
pd_hat = zeros(Nveh,1);
vd_hat = zeros(Nveh,1);
ad_hat = zeros(Nveh,1);
for i = 1:Nveh
    pd_hat(i)=p0;
    vd_hat(i)=v0;
    ad_hat(i)=a0;
end

% 定义观测值
xd_hat_vir = zeros(3,Nstep_hat);
Xd_hat_vir = cell(Nveh,1);
for i = 1:Nveh
    Xd_hat_vir{i} = xd_hat_vir;
end
% 定义初始观测值
for i = 1:Nveh
    Xd_hat_vir{i}(1,1) = pd_hat(i);
    Xd_hat_vir{i}(2,1) = vd_hat(i);
    Xd_hat_vir{i}(3,1) = ad_hat(i);
end
% 定义相对观测误差
phi_vir = zeros(3,Nstep_hat);
Phi_vir = cell(Nveh,1);
for i=1:Nveh
    Phi_vir{i} = phi_vir;
end
% 定义自适应增益
Theta_hat_vir = cell(Nveh,1);
theta_hat_vir = zeros(1,Nstep_hat);
theta_hat_vir(1) = 1;
for i=1:Nveh
    Theta_hat_vir{i} = theta_hat_vir;
end
Theta_vir = cell(Nveh,1);
theta_vir = zeros(1,Nstep_hat);
for i=1:Nveh
    Theta_vir{i} = theta_vir;
end
% 定义自适应耦合增益
Kappa_vir = cell(Nveh,1);

Q = 2*eye(size(A0));
Lyapunov_eqn = @(Pl)Pl*A0 + A0'*Pl - 2*Pl + Q;
P0 = eye(size(A0));
options2 = optimoptions('fsolve', 'Display', 'off');
Pl = fsolve(Lyapunov_eqn, P0, options2);
% 观测器更新
num_1 = 0;
num_2 = 0;
num_3 = 0;
num_4 = 0;
Prob = zeros(1,Nstep_hat);

for n = 1:Nstep_hat-1
    Prob(n) = rand();
end

% 采用随机数种子的方式导入参数变量
myProb = load('myProb.mat');
Prob = struct2array(myProb);

% 1 PLF;2 PLF链路失效-behind;3 PF;4 PLF链路失效-forward
for n=1:Nstep_hat-1
    %信息流拓扑矩阵
    M=zeros(Nveh,Nveh); %邻接矩阵
    P=zeros(Nveh,Nveh); %牵引矩阵
    if Prob(n) <= states_over_time(n,4)%0.125
        sigma(n+1) = 4;
        num_4 = num_4 + 1;
        Type_Topo = 4;
    elseif Prob(n) <= states_over_time(n,4) + states_over_time(n,2)%0.2
        sigma(n+1) = 2;
        num_2 = num_2 + 1;
        Type_Topo = 2;
    elseif Prob(n) <= states_over_time(n,4) + states_over_time(n,2) + states_over_time(n,1)%0.275
        sigma(n+1) = 1;
        num_1 = num_1 + 1;
        Type_Topo = 1;
    elseif Prob(n) <= 1%0.4 
        sigma(n+1) = 3;
        num_3 = num_3 + 1;
        Type_Topo = 3;
    end

    switch Type_Topo
        case 1 %PLF
            M(2:Nveh+1:Nveh^2)=1;
            P=diag(ones(1,Nveh));
        case 2
            M(2:Nveh+1:Nveh^2)=1;
            P(1,1)=1;
            P(2,2)=1;
            P(3,3)=1;
        case 3 %PF
            M(2:Nveh+1:Nveh^2)=1;
            P(1,1)=1;
        case 4 %模拟丢包
            M(2:Nveh+1:Nveh^2)=1;
            P(1,1)=1;
            M(3,2)=0;
    end
    MP=M+P;%特征矩阵
    DP=diag((MP(:,:)>0)*ones(Nveh,1));%入度矩阵
    LP=DP-M;%信息流矩阵

    %自设观测器
    for i=1:Nveh
        for j=1:Nveh
            Phi_vir{i}(:,n) = Phi_vir{i}(:,n) + M(i,j)*(Xd_hat_vir{i}(:,n)-Xd_hat_vir{j}(:,n))+P(i,j)*(Xd_hat_vir{i}(:,n)-X0_hat(:,n));
        end
        % Pl = 1.2*[1.7146 0.2450 0.0175;0.2450 1.7671 0.2500;0.0175 0.2500 1.7853];
        Pl = [1.5602 0.2230 0.0159; 0.2230 1.6081 0.2275; 0.0159 0.2275 1.6246];
        Theta_vir{i}(n) = (Phi_vir{i}(:,n))'*eye(3)*Phi_vir{i}(:,n);
        Theta_hat_vir{i}(n+1) = Tstep_hat*(Phi_vir{i}(:,n))'*Pl^(-1)*Phi_vir{i}(:,n) + Theta_hat_vir{i}(n);
        Xd_hat_vir{i}(:,n+1) = (A0*Tstep_hat+eye(3))*Xd_hat_vir{i}(:,n)-Tstep_hat*(Theta_vir{i}(n)+Theta_hat_vir{i}(n))*(1+(Theta_vir{i}(n))^(1/2))*Pl*Phi_vir{i}(:,n);
        Kappa_vir{i}(n) = (Theta_vir{i}(n)+Theta_hat_vir{i}(n))*(1+(Theta_vir{i}(n))^(1/4));
    end
end

% 集总观测器误差
Xd_tilde_vir = cell(Nveh,1);
for i = 1:Nveh
    Xd_tilde_vir{i} = Xd_hat_vir{i} - X0_hat(:,1:Nstep_hat);
end

% 观测器更新
Phi = cell(Nveh,1);
Theta_hat = cell(Nveh,1);
Theta = cell(Nveh,1);
Xd_hat = cell(Nveh,1);
Xd_tilde = cell(Nveh,1);
Kappa = cell(Nveh,1);
for i = 1:Nveh
    Phi{i} = Phi_vir{i}(:,1:Tstep/Tstep_hat:end);
    Theta_hat{i} = Theta_hat_vir{i}(:,1:Tstep/Tstep_hat:end);
    Theta{i} = Theta_vir{i}(:,1:Tstep/Tstep_hat:end);
    Xd_hat{i} = Xd_hat_vir{i}(:,1:Tstep/Tstep_hat:end);
    Xd_tilde{i} = Xd_tilde_vir{i}(:,1:Tstep/Tstep_hat:end);
    Kappa{i} = Kappa_vir{i}(:,1:Tstep/Tstep_hat:end);
end

X0 = X0_hat(:,1:Tstep/Tstep_hat:Nstep_hat+(Tstep/Tstep_hat)*Np);
Xd_hat_hat = cell(Nveh,1);
Xd_hat_hat_nb_avg = cell(Nveh,1);
for i=1:Nveh
    Xd_hat_hat{i} = Xd_hat{i} - repmat([i*d0;0;0],1,Nstep);%每辆跟随车要跟随的状态（期望跟随状态）
    Xd_hat_hat_nb_avg{i} = zeros(3,Nstep);
end

%% 观测器绘图
p_0=X0(1,1:Nstep);
v_0=X0(2,1:Nstep);
a_0=X0(3,1:Nstep);

XMat_hat = cell2mat(Xd_hat_hat);
p_d_hat = XMat_hat(1:3:end,:);
v_d_hat = XMat_hat(2:3:end,:);
a_d_hat = XMat_hat(3:3:end,:);

XMat_tilde = cell2mat(Xd_tilde);%将元胞数组转化为数值数组
p_d_tilde = XMat_tilde(1:3:end,:);
v_d_tilde = XMat_tilde(2:3:end,:);
a_d_tilde = XMat_tilde(3:3:end,:);

XMat_Kappa = cell2mat(Kappa);

t=(1:Nstep)*Tstep;
colors = {"#0072BD","#D95319","#EDB120","#7E2F8E","#77AC30","#4DBEEE"... % 原有
    "#A2142F", "#8C564B", "#17BECF", "#BCBD22", "#E377C2", ...
    "#9467BD", "#2CA02C", "#FF7F0E", "#1F77B4", "#7F7F7F" };

hf(1)=figure(1);%
hold on
hp(1)=plot(t,p_0,':k');
for i = 1:size(p_d_hat, 1)
    hp=plot(t,p_d_hat(i,:),'Color',colors{i});
    hold on;
end
legend('Veh 0','Veh 1','Veh 2','Veh 3','Veh 4','Veh 5', 'Interpreter', 'latex');
legend('NumColumns',2);
legend('Location','best');
legend('boxoff');
legend('FontSize',16);
xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
ylabel('$\hat{p}\,\rm[m]$', 'Interpreter', 'latex');
ha(1)=gca;

hf(2)=figure(2);%
hold on
hp(1)=plot(t,v_0,':k');
for i = 1:size(v_d_hat, 1)
    hp=plot(t,v_d_hat(i,:),'Color',colors{i});
    hold on;
end
legend('Veh 0','Veh 1','Veh 2','Veh 3','Veh 4','Veh 5', 'Interpreter', 'latex');
legend('NumColumns',2);
legend('Location','best');
legend('boxoff');
legend('FontSize',16);
xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
ylabel('$\hat{v}\,\rm[m/s]$', 'Interpreter', 'latex');
ha(2)=gca;

hf(3)=figure(3);%
hold on
hp(1)=plot(t,a_0,':k');
for i = 1:size(a_d_hat, 1)
    hp=plot(t,a_d_hat(i,:),'Color',colors{i});
    hold on;
end
legend('Veh 0','Veh 1','Veh 2','Veh 3','Veh 4','Veh 5', 'Interpreter', 'latex');
legend('NumColumns',2);
legend('Location','best');
legend('boxoff');
legend('FontSize',16);
xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
ylabel('$\hat{a}\,\rm[m/s^2]$', 'Interpreter', 'latex');
ha(3)=gca;

hf(4)=figure(4);%领航车及跟随车位移曲线
hold on
for i = 1:size(p_d_tilde, 1)
    hp=plot(t,p_d_tilde(i,:),'Color',colors{i});
    hold on;
end
legend('Veh 1','Veh 2','Veh 3','Veh 4','Veh 5', 'Interpreter', 'latex');
legend('NumColumns',2);
legend('Location','best');
legend('boxoff');
legend('FontSize',16);
xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
ylabel('$\theta_p\,\rm[m]$', 'Interpreter', 'latex');
ha(4)=gca;

hf(5)=figure(5); %领航车及跟随车速度曲线
hold on
for i = 1:size(v_d_tilde, 1)
    hp=plot(t,v_d_tilde(i,:),'Color',colors{i});
    hold on;
end
legend('Veh 1','Veh 2','Veh 3','Veh 4','Veh 5', 'Interpreter', 'latex');
legend('NumColumns',2);
legend('Location','best');
legend('boxoff');
legend('FontSize',16);
xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
ylabel('$\theta_v\,\rm[m/s]$', 'Interpreter', 'latex');
ha(5)=gca;

hf(6)=figure(6); %领航车及跟随车加速度曲线
hold on
for i = 1:size(a_d_tilde, 1)
    hp=plot(t,a_d_tilde(i,:),'Color',colors{i});
    hold on;
end
legend('Veh 1','Veh 2','Veh 3','Veh 4','Veh 5', 'Interpreter', 'latex');
legend('NumColumns',2);
legend('Location','best');
legend('boxoff');
legend('FontSize',16);
xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
ylabel('$\theta_a\,\rm[m/s^2]$', 'Interpreter', 'latex');
ha(6)=gca;

hf(7)=figure(7); %自适应耦合增益图
hold on
for i = 1:size(XMat_Kappa, 1)
    hp=plot(t,XMat_Kappa(i,:),'Color',colors{i});
    hold on;
end
legend('Veh 1','Veh 2','Veh 3','Veh 4','Veh 5', 'Interpreter', 'latex');
legend('NumColumns',2);
legend('Location','best');
legend('boxoff');
legend('FontSize',16);
xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
ylabel('$\|\kappa_i\|$', 'Interpreter', 'latex');
ha(7)=gca;

t_hat = (1:Nstep_hat)*Tstep_hat;
hf(8)=figure(8);
plot(t_hat, sigma,'LineWidth', 0.1);
xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
ylabel('$\sigma(t)={1,2,3,4}$', 'Interpreter', 'latex');
legend('Markovian Switching Signal', 'Interpreter', 'latex');
legend('boxoff');
legend('FontSize',16);
title('The Markov switching process');
ha(8)=gca;

figure_FontSize=20;
figure_FontName='Times New Roman';
figure_LineWidth=3;
set(findobj('LineWidth',0.5),'LineWidth',figure_LineWidth);
set(findobj('FontSize',10),'FontSize',figure_FontSize,'FontName',figure_FontName);

%% 闭环动力学-跟随车辆
% 初始化指标存储变量
converge_time = 0; % 收敛时间
fuel_consumption = zeros(Nveh,Nstep); % 每辆车每步油耗
compute_time = zeros(Nveh,Nstep); % 每辆车每步计算时间
string_stable_error = zeros(Nveh,Nstep); % 弦稳定误差

for n = 2:Nstep %步数
    for i = 1:Nveh %MP矩阵的行,即第n辆车
        % tic % 记录计算开始时间
        Xi = X{i}(:,n-1);
        
        % 状态方程矩阵
        Ai = A{i};
        Bi = B{i};
        Ki = K(:,i);
        
        if existModelMismatch %存在系统模型不匹配
            Ai_prediction = A{1}; 
            Bi_prediction = B{1}; 
        else
            Ai_prediction = Ai; 
            Bi_prediction = Bi; 
        end
        
        % 权重矩阵
        GGi = GG{i};
        FFi = FF{i};
        RRi = RR{i};
        SSi = SS{i};

        % MPC优化
        Xa_nb = zeros(3,Np+1); % 前车假设状态
        X_nb = zeros(3,Nstep); %前车实际状态

        % 邻居平均观测状态
        Num_nb = 0; % 当前邻居数目
        Xd_hat_nb = cell(0);
        for j = 1:Nveh
            if MP(i,j) ~= 0 % 存在邻车
                Num_nb = Num_nb + 1;
                if i == j % 邻车为领航车
                    Xd_hat_nb{Num_nb}(:,n-1) = X0(:,n-1);
                else % 邻车为非领航车
                    Xd_hat_nb{Num_nb}(:,n-1) = Xd_hat{j}(:,n-1);
                end
            else %不存在邻车
                continue;
            end
        end
        if Num_nb ~= 0
            for k = 1:Num_nb
                Xd_hat_hat_nb_avg{i}(:,n-1) = Xd_hat_hat_nb_avg{i}(:,n-1) + Xd_hat_nb{k}(:,n-1);
            end
        end
        Xd_hat_hat_nb_avg{i}(:,n-1) = (Xd_hat_hat_nb_avg{i}(:,n-1) + Xd_hat{i}(:,n-1)) / (Num_nb+1) - [i*d0;0;0];

        % DMPC的通信拓扑结构是PF
        M = zeros(Nveh,Nveh);
        P = zeros(Nveh,Nveh);
        M(2:Nveh+1:Nveh^2)=1;
        P(1,1)=1;
        MP_DMPC = M + P;
        mode = '011';
        for j = 1:Nveh %MP矩阵的列,即第n个邻车
            if MP_DMPC(i,j) ~= 0 %存在邻车
                if i == 1 % 前车为领航车
                    Xa_nb = X0(:,n-1:n-1+Np) - repmat([i*d0;0;0],1,Np+1);
                    X_nb(:,1:(n-1)) = X{i}(:,1:(n-1)) - Xd_hat_hat{i}(:,1:(n-1));
                    mode = '010';
                elseif j == (i-1) % 前车为跟随车
                    Xa_nb = Xa{j} - repmat([f(i,j)*d0;0;0],1,Np+1);
                    X_nb(:,1:(n-1)) = X{j}(:,1:(n-1)) - Xd_hat_hat{j}(:,1:(n-1));
                end
            else %不存在邻车
                continue;
            end
        end
           
        Aneq = []; bneq = []; Aeq = []; beq = []; % 没有线性约束

        % 控制量上下界
        lb = -Umin*ones(Np,1);
        ub = Umax*ones(Np,1); 
        
        % 备选的起始搜索点
        U0_list{1} = Ua_next(i,1:Np)';   
        U0_list{2} = zeros(Np,1);
        U0_list{3} = Umax * ones(Np,1);
        U0_list{4} = -Umin * ones(Np,1);
        
        

        % 记录每辆车的优化计算时间
        tic; % 开始计时（针对第i辆车第n步）                      
        
        %求解稳定优化问题
        for k = 1:4
            U0 = U0_list{k};
            [UTemp, Cost(i,n), ExitFlag(i,n), output] = fmincon( ...
                @(u) MyCostFunction(Np,Xi,Xa{i},u,Xd_hat_hat_nb_avg{i}(:,n-1),Xa_nb,Ai_prediction,Bi_prediction,SSi,RRi,FFi,GGi,Tstep), ...
                U0,Aneq,bneq,Aeq,beq,lb,ub, ...
                @(u) MyConstraints(Np,Xi,Xa{i},u,Xa_nb,Ki,Xd_hat_hat_nb_avg{i}(:,n-1),X_nb,Ai_prediction,Bi_prediction,GGi,Cost_self_GG(i),Gamma,mode,Tstep), ...
                options);
            if ExitFlag(i,n) ~= -2  % 求得可行解
                break
            else % 未求得可行解
                if output.iterations == 1 % 只迭代了一次，表示初始不可行
                    continue % 继续for循环
                else  % 初始不可行，但可迭代
                    continue
                end
            end
        end
        
        if ExitFlag(i,n) == -2 % 仍未求得可行解，则去掉终端等式约束，重新求解
            [UTemp, Cost(i,n), ExitFlag(i,n), output] = fmincon( ...
                @(u) MyCostFunction(Np,Xi,Xa{i},u,Xd_hat_hat_nb_avg{i}(:,n-1),Xa_nb,Ai_prediction,Bi_prediction,SSi,RRi,FFi,GGi,Tstep), ...
                U0,Aneq,bneq,Aeq,beq,lb,ub, ...
                @(u) MyConstraints(Np,Xi,Xa{i},u,Xa_nb,Ki,Xd_hat_hat_nb_avg{i}(:,n-1),X_nb,Ai_prediction,Bi_prediction,GGi,Cost_self_GG(i),Gamma,'001',Tstep), ...
                options);
              Cost(i,n) = 1000; % 取一个较大的代价以便于记录
        end
        
        % 记录计算时间
        comp_time(i,n) = toc; % 结束计时，记录第i辆车第n步的计算时间（单位：s）
        total_comp_time(n) = sum(comp_time(:,n)); % 计算当前步所有车辆的总计算时间（可选）
        
        % 车辆往前走一步
        if existDisturbance
            U(i,n) = UTemp(1) + 0.2 * (rand(1)-0.5); %给输入增加一个随机扰动
        else
            U(i,n) = UTemp(1);
        end
        X{i}(:,n) = MyVehicleDynamics(Xi,U(i,n),Ai,Bi); % 这里由真实模型导出
        
        if ~useLinearController
            % 预测下一步状态
            Xa_next{i}(:,1) = X{i}(:,n);
            Ua_next(i,1:Np-1) = UTemp(2:Np);
            for j = 1:Np
                Xa_next{i}(:,j+1) = MyVehicleDynamics(Xa_next{i}(:,j),Ua_next(i,j),Ai_prediction,Bi_prediction); % 这里由预测模型导出
                p_terminal_save(i,n)=Xa_next{i}(1,Np+1);
                v_terminal_save(i,n)=Xa_next{i}(2,Np+1);
                a_terminal_save(i,n)=Xa_next{i}(3,Np+1);
            end
        end
        % 计算自偏移代价
        Cost_self_GG(i) = 0;
        for j = 2:Np 
            Cost_self_GG(i) = Cost_self_GG(i) + sqrt((Xa_next{i}(:,j-1)-Xa{i}(:,j))'*GGi*(Xa_next{i}(:,j-1)-Xa{i}(:,j)));
        end
        
        % 计算油耗
        v_i = X{i}(2,n); % 当前车辆速度
        a_i = X{i}(3,n); % 当前车辆加速度
        G = 0; % 道路坡度（默认0）
        RT = 0.333 + 0.00108*v_i^2 + 1.200*a_i + 0.118*G;
        if RT>0 && a_i>0
            fuel_consumption(i,n) = 0.444 + 0.090*RT*v_i + 0.054*a_i^2*v_i;
        else
            fuel_consumption(i,n) = 0.444;
        end
        
        % 计算弦稳定误差（简化版，可根据实际定义调整）
        if i>1
            string_stable_error(i,n) = abs(X{i}(1,n) - X{i-1}(1,n) + d0);
        else
            string_stable_error(i,n) = abs(X{i}(1,n) - X0(1,n) + d0);
        end
    end
    
    Xa = Xa_next;
    digits(2);
    disp(['indexTopo = ',num2str(IndexTopo),', n = ', num2str(n), ', elapsed time = ', num2str(toc,'%.2f')]);
end

%% 计算收敛时间
% 计算车辆间距误差
dp_converge = zeros(Nveh,Nstep);
for i=1:Nveh
    if isLocalError
        if i==1
            dp_converge(i,:) = X{i}(1,:) - X0(1,:) - d0;
        else
            dp_converge(i,:) = X{i}(1,:) - X{i-1}(1,:) - d0;
        end
    else
        dp_converge(i,:) = X{i}(1,:) - X0(1,1:Nstep) + i*d0;
    end
end
dp_abs = abs(dp_converge);
% 寻找所有车辆间距误差≤0.2m且持续1s的时刻
stable_flag = false;
stable_start = 0;
for n=1:Nstep
    all_stable = all(dp_abs(:,n)<=0.5);
    if all_stable && ~stable_flag
        stable_start = n;
        stable_flag = true;
    elseif ~all_stable && stable_flag
        stable_start = 0;
        stable_flag = false;
    end
    if stable_flag && (n - stable_start)*Tstep >= 10
        converge_time = stable_start*Tstep;
        break;
    end
end

%% 计算效率指标（总计算时间/总仿真时间）
total_compute_time = sum(sum(compute_time));
total_simulation_time = Nstep*Tstep;
compute_efficiency = total_compute_time / total_simulation_time;

%% 输出量化指标结果
% 定义变量存储所有车辆的总油耗，方便后续计算平均值
all_total_fuel = zeros(1, Nveh);

fprintf('收敛时间：%.2f s\n', converge_time);
fprintf('计算效率：%.4f (总计算时间/总仿真时间)\n', compute_efficiency);
for i=1:Nveh
    total_fuel = sum(fuel_consumption(i,:))*Tstep; % 总油耗（累计）
    avg_string_error = mean(string_stable_error(i,:)); % 平均弦稳定误差
    fprintf('车辆%d - 总油耗：%.2f 单位，平均弦稳定误差：%.4f m\n', i, total_fuel, avg_string_error);
    % 将当前车辆的总油耗存入数组
    all_total_fuel(i) = total_fuel;
end
% 计算所有车辆总油耗的平均值
avg_total_fuel = mean(all_total_fuel);
% 输出平均值
fprintf('所有车辆平均总油耗：%.2f 单位\n', avg_total_fuel);

%% 计算每辆车的MCT和ACT（论文量化指标）
for i = 1:Nveh
    % 最大计算时间（MCT）：第i辆车所有步骤中的最大计算时间
    comp_time_MCT(i) = max(comp_time(i,:)); 
    % 平均计算时间（ACT）：第i辆车所有步骤中的平均计算时间（排除异常值0）
    valid_comp_time = comp_time(i, comp_time(i,:) > 1e-6); % 排除极小值（避免初始化0的影响）
    comp_time_ACT(i) = mean(valid_comp_time); 
end

% 打印每辆车的量化指标结果
fprintf('==================== 计算时间量化指标（论文定义）====================\n');
fprintf('车辆编号\t最大计算时间MCT(s)\t平均计算时间ACT(s)\n');
for i = 1:Nveh
    fprintf('%d\t\t%.6f\t\t%.6f\n', i, comp_time_MCT(i), comp_time_ACT(i));
end
fprintf('====================================================================\n');

% 可选：计算所有车辆的整体MCT和ACT
global_MCT = max(comp_time_MCT);
global_ACT = mean(comp_time_ACT);
fprintf('整体最大计算时间（所有车辆）：%.6f s\n', global_MCT);
fprintf('整体平均计算时间（所有车辆）：%.6f s\n', global_ACT);

%% 绘图
p0=X0(1,1:Nstep);
v0=X0(2,1:Nstep);
a0=X0(3,1:Nstep);
XMat = cell2mat(X);
p = XMat(1:3:end,:);
v = XMat(2:3:end,:);
a = XMat(3:3:end,:);
t=(1:Nstep)*Tstep;

if ~isLocalError %与领航车误差
    HH=zeros(Nveh,Nveh);
    for k=1:Nveh
    end
    if isTimeHeadwayIncluded
        dp=p-kron(ones(Nveh,1),p0)+kron(ones(1,Nstep),kron((1:Nveh)',d0))+HH*v;%全局距离误差（含时距）
    else
        dp=p-kron(ones(Nveh,1),p0)+kron(ones(1,Nstep),kron((1:Nveh)',d0));%全局距离误差（不含时距）
    end
    dv=v-kron(ones(Nveh,1),v0);
    da=a-kron(ones(Nveh,1),a0);
    MPE_1 = max(max(abs(dp(1,:))));
    MPE_2 = max(max(abs(dp(2,:))));
    MPE_3 = max(max(abs(dp(3,:))));
    MPE_4 = max(max(abs(dp(4,:))));
    MPE_5 = max(max(abs(dp(5,:))));
    MPE_global = max(max(abs(dp)));
    MVE_global = max(max(abs(dv)));
    APE_global = mean(mean(abs(dp)));
    AVE_global = mean(mean(abs(dv)));
else %与前车误差
    dp=p+ones(Nveh,Nstep)*d0-[p0;p(1:Nveh-1,:)]; %前车距离误差（不含时距）
    dv=v-[v0;v(1:Nveh-1,:)];
    da=a-[a0;a(1:Nveh-1,:)];
    MPE_local = max(max(abs(dp)));
    MVE_local = max(max(abs(dv)));
    APE_local = mean(mean(abs(dp)));
    AVE_local = mean(mean(abs(dv)));
end

if doPic == true
    hf(9)=figure(9);
    hold on
    hp(1)=plot(t,p0,':k');
    for i = 1:size(p, 1)
        hp=plot(t,p(i,:),'Color',colors{i});
        hold on
    end
    legend('Veh 0','Veh 1','Veh 2','Veh 3','Veh 4','Veh 5','Interpreter', 'latex');
    legend('NumColumns',2);
    legend('Location','best');
    legend('boxoff');
    legend('FontSize',16);
    xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
    ylabel('$p\rm[m]$', 'Interpreter', 'latex');
    ha(9)=gca;
       
    hf(10)=figure(10);
    hold on
    hp=plot(t,dp);
    legend(hp(1:end),'Veh 1','Veh 2','Veh 3','Veh 4','Veh 5','Interpreter', 'latex');
    legend('NumColumns',2);
    legend('Location','best');
    legend('boxoff');
    legend('FontSize',16);
    xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
    ylabel('$e_p\rm[m]$', 'Interpreter', 'latex');
    ha(10)=gca;
    
    hf(11)=figure(11);
    hold on
    hp=plot(t,dv);
    legend(hp(1:end),'Veh 1','Veh 2','Veh 3','Veh 4','Veh 5','Interpreter', 'latex');
    legend('NumColumns',2);
    legend('Location','best');
    legend('boxoff');
    legend('FontSize',16);
    xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
    ylabel('$e_v$\rm[m/s]', 'Interpreter', 'latex');
    ha(11)=gca;
    
    hf(12)=figure(12);
    hold on
    hp=plot(t,da);
    legend(hp(1:end),'Veh 1','Veh 2','Veh 3','Veh 4','Veh 5','Interpreter', 'latex');
    legend('NumColumns',2);
    legend('Location','best');
    legend('boxoff');
    legend('FontSize',16);
    xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
    ylabel('$e_a$\rm[m/s]', 'Interpreter', 'latex');
    ha(12)=gca;
    
    hf(13)=figure(13);
    hold on
    hp=plot(t,U);
    legend(hp(1:end),'Veh 1','Veh 2','Veh 3','Veh 4','Veh 5','Interpreter', 'latex');
    legend('NumColumns',1);
    legend('Location','best');
    legend('boxoff');
    legend('FontSize',16);
    xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
    ylabel('$u\rm[m/s^2]$', 'Interpreter', 'latex');
    ha(13)=gca;
    
    hf(14)=figure(14);
    hold on
    hp(1)=plot(t,v0,'k:');
    for i = 1:size(v, 1)
        hp=plot(t,v(i,:),'Color',colors{i});
        hold on
    end
    legend('Veh 0','Veh 1','Veh 2','Veh 3','Veh 4','Veh 5', 'Interpreter', 'latex');
    legend('NumColumns',2);
    legend('Location','best');
    legend('boxoff');
    legend('FontSize',16);
    xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
    ylabel('$v\rm[m/s]$', 'Interpreter', 'latex');
    ha(14)=gca;
    
    hf(15)=figure(15);
    hold on
    hp(1)=plot(t,a0,'k:');
    for i = 1:size(a, 1)
        hp=plot(t,a(i,:),'Color',colors{i});
        hold on
    end
    legend('Veh 0','Veh 1','Veh 2','Veh 3','Veh 4','Veh 5','Interpreter', 'latex');
    legend('NumColumns',2);
    legend('Location','best');
    legend('boxoff');
    legend('FontSize',16);
    xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
    ylabel('$a\rm[m/s^2]$', 'Interpreter', 'latex');
    ha(15)=gca;

    % 新增指标绘图
    hf(16)=figure(16);
    hold on
    for i=1:Nveh
        hp=plot(t,fuel_consumption(i,:),'Color',colors{i});
        hold on
    end
    legend('Veh 1','Veh 2','Veh 3','Veh 4','Veh 5','Interpreter', 'latex');
    legend('NumColumns',2);
    legend('Location','best');
    legend('boxoff');
    legend('FontSize',16);
    xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
    ylabel('Fuel Consumption','Interpreter', 'latex');
    ha(16)=gca;
    
    hf(17)=figure(17);
    hold on
    for i=1:Nveh
        hp=plot(t,string_stable_error(i,:),'Color',colors{i});
        hold on
    end
    legend('Veh 1','Veh 2','Veh 3','Veh 4','Veh 5','Interpreter', 'latex');
    legend('NumColumns',2);
    legend('Location','best');
    legend('boxoff');
    legend('FontSize',16);
    xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
    ylabel('String Stable Error (m)','Interpreter', 'latex');
    ha(17)=gca;

    % 图1：每辆车随时间的计算时间变化曲线
    hf(18) = figure(18);
    hold on;
    for i = 1:Nveh
        plot(t, comp_time(i,:), 'Color', colors{i}, 'LineWidth', 2, 'DisplayName', sprintf('Veh %d', i));
    end
    xlabel('$t\,\rm[s]$', 'Interpreter', 'latex');
    ylabel('Computation time $\rm[s]$', 'Interpreter', 'latex');
    % title('每辆车的实时计算时间', 'FontSize', 18, 'FontWeight', 'bold');
    legend('Location', 'best', 'FontSize', 16, 'Interpreter', 'latex');
    legend('boxoff');
    ha(18)=gca;
    % set(gca, 'FontSize', 16, 'FontName', 'Times New Roman');

    figure_FontSize=20;
    figure_FontName='Times New Roman';
    figure_LineWidth=3;
    lengthOfha = length(ha);
    hax = get(ha,'XLabel');
    hay = get(ha,'YLabel');
    for kkkk = 1:lengthOfha
        set(hax{kkkk},'FontSize',figure_FontSize,'FontName',figure_FontName);
        set(hay{kkkk},'FontSize',figure_FontSize,'FontName',figure_FontName);
    end
    set(findobj('LineWidth',0.5),'LineWidth',figure_LineWidth);
    set(findobj('FontSize',10),'FontSize',figure_FontSize,'FontName',figure_FontName);
end

delta = 0.1;
dp_abs=abs(dp);
dp_max=max(dp_abs);
Tc=find(dp_max>=delta,1,'last')/100;

