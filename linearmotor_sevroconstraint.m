function f=linearmotor_sevroconstraint(t,s,ds)
global  R m_ba k_f k_e f_c f_s f_v dx_s A1 A2 A3 omiga k P rou rou_e labmda varepsilon

% gui ji yue shu  

%    A=1;
%    b=0;
%    Adq_c=s(2);
%    Bq_d=s(1)-3;
%    Baita=Adq_c+labmda*Bq_d;

    A=1;
    b=-3*sin(t)+labmda*(s(2)-3*cos(t));
    Adq_c=s(2)-3*cos(t);
    Bq_d=s(1)-3*sin(t);
    Baita=Adq_c+labmda*Bq_d;

  F_f=(f_c+(f_s-f_c)*(exp(1)^(-(s(2)/dx_s)^2))+f_v*s(2))*sign(s(2));
  F_r=A1*sin(omiga*s(1))+A2*sin(3*omiga*s(1))+A3*sin(5*omiga*s(1));
  d=F_f+F_r;
  M_ba=R*m_ba/k_f;
  Q_ba=-(k_e*s(2)+R*d/k_f);


  mu_ba=M_ba^(-1)*A'*P;
  elta=mu_ba*Baita;
  mu=elta*rou;
 
     if norm(mu)>varepsilon
        gamma=(1+rou_e)^(-1)/(norm(mu_ba)*norm(mu));
    else
         gamma=(1+rou_e)^(-1)/(norm(mu_ba)*norm(mu_ba)*varepsilon);
    end  
     
  
 m=m_ba+cos(t);
 %m2=m2_ba+0.001*cos(0.01*t);

  M=R*m/k_f;
  Q=-(k_e*s(2)+R*d/k_f);

  p1=M_ba^(1/2)*pinv((A*M_ba^(-1/2)))*(b-A*M_ba^(-1)*Q_ba);
  p2=-k*M_ba^(-1)*A'*P*Baita;
  p3=-gamma*mu*rou;


ddq=M^( -1)*Q+M^(-1)*(p1+p2+p3);

f=[ds(1)-s(2);
   ds(2)-ddq(1)];