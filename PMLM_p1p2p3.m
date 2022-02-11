function f=PMLM_p1p2p3(t,s,ds)
global  R m_ba k_f k_e f_c f_s f_v dx_s A1 A2 A3 omega k P rou rou_e varepsilon x_m x_M Amp labmda

%%  轨迹约束
    y_d=-tan(pi/2 + (pi*(x_m - Amp))/(x_M - x_m));
    dy_d=(pi*(cot((pi*(x_m - Amp))/(x_M - x_m))^2 + 1)*0)/(x_M - x_m);
    ddy_d=(pi*(cot((pi*(x_m - Amp))/(x_M - x_m))^2 + 1)*0)/(x_M - x_m) + (2*pi^2*cot((pi*(x_m - Amp))/(x_M - x_m))*(cot((pi*(x_m - Amp))/(x_M - x_m))^2 + 1)*0^2)/(x_M - x_m)^2;
   
   A=1;
   b=ddy_d;
   Adq_c=s(2)-dy_d;
   Bq_d=s(1)-y_d;
   Baita=Adq_c+labmda*Bq_d;
    
%% 动力学方程
      M_be_ba=R*m_ba/k_f;   %转换前M

      F_f=(f_c+(f_s-f_c)*(exp(1)^(-((x_M-x_m)*(s(2))/(pi*(1+(s(1))^2))/dx_s)^2)))*sign((x_M-x_m)*s(2)/(pi*(1+(s(1))^2)));
      F_r=A1*sin(omega*((x_M-x_m)*atan(s(1))/pi+(x_M+x_m)/2))+A2*sin(3*omega*((x_M-x_m)*atan(s(1))/pi+(x_M+x_m)/2))+A3*sin(5*omega*((x_M-x_m)*atan(s(1))/pi+(x_M+x_m)/2));
      F_d=0;
      F_l=0;
      F_e=F_f+F_r+F_d+F_l;
      M_ba=M_be_ba*(x_M-x_m)*(1+(s(1))^2)/(pi*(1+(s(1))^2)^2);
      C_ba=(k_e+f_v)*(x_M-x_m)/(pi*(1+(s(1))^2));
      G_ba=F_e-2*M_be_ba*(x_M-x_m)*((s(1))^2)*s(2)^2/(pi*(1+(s(1))^2)^2);
     
       %质量不确定性
     m=m_ba+0.1*cos(t);
       
     % 突变力
    if t>0.8 &&t<1.2
        F_ddd=-500;
        disp(F_ddd);
    else
        F_ddd=0;
    end
      % 实际矩阵
      M=R*m/k_f;    
      C=C_ba;
      G=G_ba+0.1*sin(0.01*t)+F_ddd;
      Q=-(C*s(2)+G);
      
      %控制参数
      mu_ba=M_ba^(-1)*A'*P;
      eta=mu_ba*Baita;
      mu=eta*rou;
 
     if norm(mu)>varepsilon
        gamma=(1+rou_e)^(-1)/(norm(mu_ba)*norm(mu));
    else
         gamma=(1+rou_e)^(-1)/(norm(mu_ba)*norm(mu_ba)*varepsilon);
     end  
      
      p1=M_ba^(1/2)*(A*M_ba^(-1))^(-1)*(b+A*M_ba^-1*(C_ba+G_ba)-labmda*Adq_c);
      p2=-k*M_ba^(-1)*A'*P*Baita;
      p3=-gamma*mu*rou;


    ddq=M^( -1)*Q+M^(-1)*(p1+p2+p3);

    f=[ds(1)-s(2);
       ds(2)-ddq(1)];