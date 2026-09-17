function [C, Ceq] = MyConstraints(Np,Xi,Xai,U,Xa_nb_i,Ki,Xd_hat_a,X_nb_i,Ai,Bi,GGi,Cost_self_GG_last,Gamma,MODE,Tstep)
% 约束 C <= 0, Ceq = 0
Xp = zeros(3,Np+1);
Xp(:,1) = Xi;
% ======================================================================
% 函数中的待优化变量U用来生成约束与代价，无需定义
% ======================================================================
A0=[0 1 0;0 0 1;0 0 0];
gamma = [1 0 0];

for k = 1:Np
    Xp(:,k+1) = MyVehicleDynamics(Xp(:,k),U(k),Ai,Bi);
    Xd_hat_a(:,k+1) = (A0*Tstep+eye(3)) * Xd_hat_a(:,k);
end

if MODE(1) == '1' %自偏移代价要求
    Cost_self_GG = 0;
    for k = 2:Np % k=[1,Np-1]
        Cost_self_GG = Cost_self_GG  + sqrt((Xp(:,k)-Xai(:,k))' * GGi * (Xp(:,k)-Xai(:,k)));
    end
    C = Gamma * Cost_self_GG - Cost_self_GG_last; 
else
    C = [];
end

% if MODE(2) == '1' % 终端等式要求
%     hi = zeros(3,1);
%     for k = 1:length(Xa_nb)
%         Xa_nb_k = Xa_nb{k};
%         % ======================================================================
%         % 此处接收到的邻居信息已经是通信失效后的信息，直接累加即可
%         % ======================================================================
%         hi = hi + Xa_nb_k(:,Np)-Xai(:,Np); %xigma(丢包概率ij*aij（Xaj-Xai）)
%     end
%     U(Np) = Ki'*hi;
%     Xai(:,Np+1) = MyVehicleDynamics(Xai(:,Np),U(Np),Ai,Bi);
%     Ceq = Xp(:,end) - Xai(:,end);
% else
%     Ceq = [];
% end

if MODE(2) == '1' % 终端等式要求
    hi = Xd_hat_a(:,Np)-Xai(:,Np); %xigma(丢包概率ij*aij（Xaj-Xai）)
    U(Np) = Ki'*hi;
    Xai(:,Np+1) = MyVehicleDynamics(Xai(:,Np),U(Np),Ai,Bi);
    Ceq = Xp(:,end) - Xai(:,end);
else
    Ceq = [];
end

if MODE(3) == '1' %弦稳定约束
    % 该不等式约束使得后车摆烂
        % 0.999 是final的结果
        % C = max(abs(gamma*(Xp(:,1:end)-Xd_hat_a(:,1:end))))-0.999*max([max(abs(gamma*X_nb_i)),max(abs(gamma*(Xa_nb_i(:,1:end)-Xd_hat_a(:,1:end))))]);%u6
        % C = max(abs(gamma*(Xp(:,1:end)-Xd_hat_a(:,1:end))))-0.9*max([max(abs(gamma*X_nb_i)),max(abs(gamma*(Xa_nb_i(:,1:end)-Xd_hat_a(:,1:end))))]);%test1
        % C = max(abs(gamma*(Xp(:,1:end)-Xd_hat_a(:,1:end))))-0.75*max([max(abs(gamma*X_nb_i)),max(abs(gamma*(Xa_nb_i(:,1:end)-Xd_hat_a(:,1:end))))]);%
        % C = max(abs(gamma*(Xp(:,1:end)-Xd_hat_a(:,1:end))))-0.85*max([max(abs(gamma*X_nb_i)),max(abs(gamma*(Xa_nb_i(:,1:end)-Xd_hat_a(:,1:end))))]);%test3
        C = max(abs(gamma*(Xp(:,1:end)-Xd_hat_a(:,1:end))))-0.6*max([max(abs(gamma*X_nb_i)),max(abs(gamma*(Xa_nb_i(:,1:end)-Xd_hat_a(:,1:end))))]);%test4
end

end