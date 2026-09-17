function Cost = MyCostFunction(Np,Xi,Xai,U,Xd_hat_i,Xa_nb_i,Ai,Bi,SSi,RRi,FFi,GGi,Tstep)
% 代价函数计算
% 输入预测时域长度等信息，给出预测时域内跟踪误差组成的代价函数
% Xt = 3*1; Xa = 3*Np; Xa_nb = Num_nb*Np*3
Xp = zeros(3,Np+1);
Xp(:,1) = Xi;

Xd_hat_a = zeros(3,Np+1);
Xd_hat_a(:,1) = Xd_hat_i;
A0=[0 1 0;0 0 1;0 0 0];
% ======================================================================
% 函数中的待优化变量U用来生成约束与代价，无需定义
% ======================================================================
for k = 1:Np
    Xp(:,k+1) = MyVehicleDynamics(Xp(:,k),U(k),Ai,Bi);
    Xd_hat_a(:,k+1) = (A0*Tstep+eye(3)) * Xd_hat_a(:,k);
end

% 观测偏移惩罚
Cost_leader = 0;
for i = 1:Np % k=[0,Np-1]
    Cost_leader = Cost_leader + sqrt((Xp(:,i)-Xd_hat_a(:,i))' * GGi * (Xp(:,i)-Xd_hat_a(:,i)));
end

% 前车偏移惩罚
Cost_nb = 0;
for i = 1:Np % k=[0,Np-1]
    Cost_nb = Cost_nb + sqrt((Xp(:,i)-Xa_nb_i(:,i))' * SSi * (Xp(:,i)-Xa_nb_i(:,i)));
end

% 自偏移惩罚
Cost_self = 0;
% 输入惩罚
Cost_u = 0;
for i = 1:Np % k=[0,Np-1]
    Cost_self = Cost_self + sqrt((Xp(:,i)-Xai(:,i))' * FFi * (Xp(:,i)-Xai(:,i)));
    Cost_u = Cost_u + sqrt((U(i))'* RRi *(U(i)));
end

Cost = Cost_leader + Cost_self + Cost_u + Cost_nb;
end

% Cost_nb = 0; % 邻域偏移惩罚
% % Cost_terminal = 0; % 终端邻域偏移惩罚
% for k = 1:length(Xa_nb)
%     Xa_nb_k = Xa_nb{k};
%     for i = 1:Np % k=[0,Np-1]  
%         X_error = Xp(:,i)-Xa_nb_k(:,i);
%         Cost_nb = Cost_nb + sqrt(X_error' * GGi * X_error);
%     end
% %  Cost_terminal = Cost_terminal + sqrt((Xp(:,Np+1)-Xa_nb_k(:,Np+1))' *  GGi * (Xp(:,Np+1)-Xa_nb_k(:,Np+1)));
% end
% 
% Cost_self = 0; % 自偏移惩罚
% Cost_u = 0; % 输入惩罚
% for i = 1:Np % k=[0,Np-1]
%     X_error = Xp(:,i)-Xai(:,i);
%     Cost_self = Cost_self + sqrt(X_error' * FFi * X_error);
%     Cost_u = Cost_u + sqrt(U(i)*RRi*U(i)');
% end
% 
% Cost = Cost_nb + Cost_self + Cost_u; %+ Cost_terminal;
