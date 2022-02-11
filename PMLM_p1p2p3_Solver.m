clear 
clc
close all

global  R m_ba k_f k_e f_c f_s f_v dx_s A1 A2 A3 omega k P rou rou_e labmda varepsilon x_m x_M Amp
%% 直线电机参数
R=5.6;          %系统阻抗
m_ba=1.4;    %名义质量
k_f=260;       %力矩常数
k_e=60;        %反电动势
f_c=5;           %库伦摩擦系数
f_s=8;           %静摩擦力系数
f_v=5;           %粘滞摩擦力系数
dx_s=0.5;      %润滑条件
A1=3;           %摩擦力第一项
A2=2;           %摩擦力第二项
A3=1;           %摩擦力第三项
omega=314;  %摩擦力频率

%% 控制参数
% 
% P=6;
% k=40;
% rou=800;
% varepsilon=0.001;
% labmda=1;
% rou_e=0;


P=2;
k=5;
rou=800;
varepsilon=0.001;
labmda=0.5;
rou_e=0;
%% 不等式约束条件
x_m=-0.2;     %下界
x_M=0.2;      %上界

Amp=0.198;      %信号幅值

%% 期望初始条件
x_d0=0;
dx_d0=0;
ddx_d0=0;

%% 给定初始条件
x = 0;
dot_x = 0;
ddot_x = 0;

y0=-tan(pi/2 + (pi*(x_m - x))/(x_M - x_m));
dy0=(pi*(tan(pi/2 + (pi*(x_m - x))/(x_M - x_m))^2 + 1)*dot_x)/(x_M - x_m);
ddy0=(pi*(tan(pi/2 + (pi*(x_m - x))/(x_M - x_m))^2 + 1)*ddot_x)/(x_M - x_m) - (2*pi^2*tan(pi/2 + (pi*(x_m - x))/(x_M - x_m))*(tan(pi/2 + (pi*(x_m - x))/(x_M - x_m))^2 + 1)*dot_x^2)/(x_M - x_m)^2;

s0=[y0 dy0]';        % y(0)  dy(0)
ds0=[dy0 ddy0]';  % dy(0)  ddy(0) 
%% 求解器迭代求解微分方程
options=odeset; 
options.RelTol=1e-8;
[t,sout] = ode15i('PMLM_p1p2p3',[0 20],s0,ds0,options); %sout(1)是位置，sout(2)是速度
time=length(t);
%% 代入每次迭代结果，求解在各个迭代点下的信息
for ka=1:time
     tte=t(ka);
    
%%  期望轨迹 
      x_d= Amp*sin(tte);

%% 状态转换
% 期望轨迹约束
    y_d=-tan(pi/2 + (pi*(x_m - Amp))/(x_M - x_m));
    dy_d=(pi*(cot((pi*(x_m - Amp))/(x_M - x_m))^2 + 1)*0)/(x_M - x_m);
    ddy_d=(pi*(cot((pi*(x_m - Amp))/(x_M - x_m))^2 + 1)*0)/(x_M - x_m) + (2*pi^2*cot((pi*(x_m - Amp))/(x_M - x_m))*(cot((pi*(x_m - Amp))/(x_M - x_m))^2 + 1)*0^2)/(x_M - x_m)^2;
    
  
  %% 转换前名义参数
%   F_f=(f_c+(f_s-f_c)*(exp(1)^(-(sout(ka,2)/dx_s)^2))+f_v*sout(ka,2))*sign(sout(ka,2));
%   F_r=A1*sin(omega*sout(ka,1))+A2*sin(3*omega*sout(ka,1))+A3*sin(5*omega*sout(ka,1));
%   d=F_f+F_r;
      M_be_ba=R*m_ba/k_f;
%   C_F_ba=-(k_e*sout(ka,2)+R*d/k_f);
  %% 转换后动力学参数
  
      F_f=(f_c+(f_s-f_c)*(exp(1)^(-((x_M-x_m)*(sout(ka,2))/(pi*(1+(sout(ka,1))^2))/dx_s)^2)))*sign((x_M-x_m)*sout(ka,2)/(pi*(1+(sout(ka,1))^2)));
      F_r=A1*sin(omega*((x_M-x_m)*atan(sout(ka,1))/pi+(x_M+x_m)/2))+A2*sin(3*omega*((x_M-x_m)*atan(sout(ka,1))/pi+(x_M+x_m)/2))+A3*sin(5*omega*((x_M-x_m)*atan(sout(ka,1))/pi+(x_M+x_m)/2));
      F_d=0;
      F_l=0;
      F_e=F_f+F_r+F_d+F_l; 
      M_ba=M_be_ba*(x_M-x_m)*(1+(sout(ka,1))^2)/(pi*(1+(sout(ka,1))^2)^2);
      C_ba=(k_e+f_v)*(x_M-x_m)/(pi*(1+(sout(ka,1))^2));
      G_ba=F_e-2*M_be_ba*(x_M-x_m)*((sout(ka,1))^2)*sout(ka,2)^2/(pi*(1+(sout(ka,1))^2)^2);
      
      
%%   给出约束
        A=1;
        b=ddy_d;                            %约束二阶形式
        Adq_c=sout(ka,2)-dy_d;       %约束一阶形式
        Bq_d=sout(ka,1)-y_d;           %约束
        Baita=Adq_c+labmda*Bq_d;
    
    mu_ba=M_ba^(-1)*A'*P;
    eta=mu_ba*Baita;
    mu=eta*rou;

    if norm(mu)>varepsilon
        gamma=(1+rou_e)^(-1)/(norm(mu_ba)*norm(mu));
    else
        gamma=(1+rou_e)^(-1)/(norm(mu_ba)*norm(mu_ba)*varepsilon);
    end
 %% 计算力矩  
%     p1=M_ba^(1/2)*pinv((A*M_ba^(-1/2)))*(b+A*M_ba^-1*(C_ba+G_ba));
%     p1=M_ba^(1/2)*pinv((A*M_ba^(-1)))*(b+A*M_ba^-1*(C_ba+G_ba)-labmda*sout(ka,2)-dy_d);
    p1=M_ba^(1/2)*(A*M_ba^(-1))^(-1)*(b+A*M_ba^-1*(C_ba+G_ba)-labmda*Adq_c);
    p2=-k*M_ba^(-1)*A'*P*Baita;
    p3=-gamma*mu*rou;
    
    tau1(ka)=p1(1)+p2(1)+p3(1); 
    tau2(ka)=p1(1)+p2(1);
    tau3(ka)=p1(1);
    
    %加速度
%     ddy_d1(ka)=M^( -1)*Q+M^(-1)*(p1+p2+p3);
%     ddy_d2(ka)=M^( -1)*Q+M^(-1)*(p1+p2);
%     ddy_d3(ka)=M^( -1)*Q+M^(-1)*(p1);

    %% 再转换回x看效果。
   x(ka)=(atan(sout(ka,1))+pi/2)*(x_M-x_m)/pi+x_m;
   
end

err=Amp-x';

figure(1)

plot(t,(t*0+Amp)*100,'LineWidth',1.5);
hold on 

plot(t,x*100,'LineWidth',1.5);
hold on

% plot(t,t*0+x_m,'LineWidth',2,'LineStyle','--');
% hold on

plot(t,(t*0+x_M)*100,'LineWidth',1.5,'LineStyle','--');

grid on
legend('Ref','Displacement of x','Upper bound');
title('Displacement of x with Inequality constraints(mm)');

% 
figure(2)
plot(t,err);


