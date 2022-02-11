%定义变量
clc
clear
syms time tte Amp x x_M x_m x y y_d x_d;
syms x_d(tte)


y_d=tan(pi/(x_M-x_m)*(x_d(tte)-x_m)-pi/2);
 
dy_d=simplify(diff(y_d,tte));

ddy_d=simplify(diff(dy_d,tte));

% 
% y=tan(pi/(x_M-x_m)*(x(t)-x_m)-pi/2);
% 
% dy=diff(y,t);
% 
% ddy=diff(dy,t);

