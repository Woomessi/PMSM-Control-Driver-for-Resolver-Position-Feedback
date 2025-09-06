clear
clc

%%% 硅油在1000米压力，2℃下的物理量数值 %%%
p_oil = 10e6; % 油液压力(Pa)
v0 = 50e-6; % 液压油的额定运动黏度(m^2/s) 50 cSt
mu0 = 71.3e-3; % 液压油的额定动力黏度(Pa·s)
alpha_mu = 1/(9.43e7+2.8e-1*p_oil); % 黏性压力系数(1/Pa)
mu = mu0*exp(alpha_mu*p_oil); % 实际动力黏度(Pa·s) 

%%% 硅油在1个大气压，25℃下的物理量数值 %%%
p_oil = 1e5; % 油液压力(Pa)
v0 =  100e-6; % 83 cSt
rou = 963;
mu0 = v0*rou;
alpha_mu = 1/(1.15e8-1.2e-1*p_oil); % 黏性压力系数(1/Pa)
mu = mu0*exp(alpha_mu*p_oil); % 实际动力黏度(Pa·s) 

