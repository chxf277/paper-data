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
P=14;
k=50;
rou=5;
varepsilon=0.001;
labmda=4;%此项大，收敛速度越快
rou_e=0;
%% 不等式约束条件
x_m=-0.2;     %下界
x_M=0.2;      %上界

Amp=0.199;      %信号幅值

%% 给定初始条件
x0=0;
dx0=0;
ddx0=0;

s0=[x0 dx0]';           % y(0)  dy(0)
ds0=[dx0 ddx0]';  % dy(0)  ddy(0) 
%% 求解器迭代求解微分方程
options=odeset; 
options.RelTol=1e-8;
[t,sout] = ode15i('PMLM_p1p2p3_noconstraint',[0 20],s0,ds0,options); %sout(1)是位置，sout(2)是速度
time=length(t);

%% 代入每次迭代结果，求解在各个迭代点下的信息
for ka=1:time
    tte=t(ka);
    
%% 期望轨迹 
% 以x为变量的 期望轨迹约束
%   我们给出的是期望信号的约束，实际的要看微分方程求解的实际信号。
      x_d= Amp;
      dx_d=0;
      ddx_d=0;

  % 转换前名义参数
  F_f=(f_c+(f_s-f_c)*(exp(1)^(-(sout(ka,2)/dx_s)^2))+f_v*sout(ka,2))*sign(sout(ka,2));
  F_r=A1*sin(omega*sout(ka,1))+A2*sin(3*omega*sout(ka,1))+A3*sin(5*omega*sout(ka,1));
  F_d=0;
  F_l=0;
  F_e=F_f+F_r+F_d+F_l;
  M_ba=R*m_ba/k_f;
  C_ba=k_e+f_v;
  G_ba=F_e;
       
%%   给出约束
    A=1;
    Adq_c=sout(ka,2)-dx_d;
    Bq_d=sout(ka,1)-x_d;
    b=ddx_d;
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
    p1=M_ba^(1/2)*pinv((A*M_ba^(-1/2)))*(b+A*M_ba^-1*(C_ba+G_ba)-labmda*Adq_c);
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
     
%     x(ka)=((x_M-x_m)*atan(sout(ka,1)+y_d)/pi+(x_M+x_m)/2);
%     dx(ka)=(x_M-x_m)*sout(ka,2)/(pi*(1+(sout(ka,1)+y_d)^2));
%     ddx(ka)=((x_M-x_m)*(1+(sout(ka,1)+y_d)^2)*ddy_d1(ka)-2*(sout(ka,1)+y_d)*sout(ka,2)^2)/(pi*(1+(sout(ka,1)+y_d)^2)^2); 
    
    x_error(ka)=x_d-sout(ka,1);
end

%% 
%  error1d=5;
figure(1)
plot(t,(0*t+Amp)*100,'LineWidth',2);
hold on
plot(t,sout(:,1)*100,'LineWidth',2);
hold on
plot(t,(0*t+x_M)*100,'LineWidth',2,'LineStyle','--');
hold on
% plot(t,(0*t+x_m)*100,'LineWidth',2,'LineStyle','--');
grid on
title('Displacement of x without Inequality constraints(mm)');
legend('Ref','Displacement of x','Upper bound');

figure(2)
plot(t,x_error);
%  figure(1)
%  plot(t,sout(:,1),'b')
%  grid on
%  hold on
%  plot(t,3*sin(t),'r')
%  title('Plot of x as a function of time')
%  xlabel('time'); ylabel('x');
%  
%  figure(2)
%  plot(t,tau1,'r');
%  grid on
%  hold on
%  plot(t,tau2,'b');
%  hold on
%  plot(t,tau3,'g');
%  title('Plot of control input as a function of time');
%  xlabel('time'); ylabel('\tau_1(t)');
% 
%  figure(3)
%  plot(t,err)
%  grid on
%  title('Plot of error as a function of time');
%  xlabel('time'); ylabel('error');