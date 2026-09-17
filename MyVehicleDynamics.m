function X2 = MyVehicleDynamics(X1,U,Ai,Bi)
% 线性车辆动力学
% 返回值： X2 = Gi * X1 + Fi * U

X2 = Ai * X1 + Bi * U;

end
