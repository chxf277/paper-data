function f=PMLM_p1p2p3_noconstraint(t,s,ds)
global  R m_ba k_f k_e f_c f_s f_v dx_s A1 A2 A3 omega k P rou rou_e varepsilon x_m x_M Amp labmda
 
%% 初始值
    x_d= Amp;
    dx_d=0;
    ddx_d=0;
  
%% 轨迹约束
   A=1;
   b=ddx_d;
   Adq_c=s(2)-dx_d;
   Bq_d=s(1)-x_d;
   Baita=Adq_c+labmda*Bq_d;
    
%% 动力学方程

      F_f=(f_c+(f_s-f_c)*(exp(1)^(-(s(2)/dx_s)^2))+f_v*s(2))*sign(s(2));
      F_r=A1*sin(omega*s(1))+A2*sin(3*omega*s(1))+A3*sin(5*omega*s(1));
      F_d=0;
      F_l=0;
      F_e=F_f+F_r+F_d+F_l;
      M_ba=R*m_ba/k_f;
      C_ba=k_e+f_v;
      G_ba=F_e;

      mu_ba=M_ba^(-1)*A'*P;
      eta=mu_ba*Baita;
      mu=eta*rou;
 
     if norm(mu)>varepsilon
        gamma=(1+rou_e)^(-1)/(norm(mu_ba)*norm(mu));
    else
         gamma=(1+rou_e)^(-1)/(norm(mu_ba)*norm(mu_ba)*varepsilon);
     end  
    
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
      G=G_ba+0.1*sin(0.1*t)+F_ddd;
      Q=-(C*s(2)+G);
    
%  p1=M_ba^(1/2)*pinv((A*M_ba^(-1/2)))*(b+A*M_ba^-1*(C_ba+G_ba));
%  p2=-k*M_ba*A'*(A*A')^(-1)*P^(-1)*Baita;
%  p3=-(M_ba*A'*(A*A')^(-1)*P^(-1))*gamma*mu*rou;

    p1=M_ba^(1/2)*pinv((A*M_ba^(-1/2)))*(b+A*M_ba^-1*(C_ba+G_ba)-labmda*Adq_c);
    p2=-k*M_ba^(-1)*A'*P*Baita;
    p3=-gamma*mu*rou;


    ddq=M^( -1)*Q+M^(-1)*(p1+p2+p3);

    f=[ds(1)-s(2);
       ds(2)-ddq(1)];