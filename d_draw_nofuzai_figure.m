clear,clc,
close all;
%����mat�ļ�
mpdp=load('d_constraint_withoutload.mat');
mpd=load('d_noconstrain_withoutload.mat');
% pid=load('PID_fuzai_data.mat');

LineWidth=2;
stopTime=20;
% N=4;
%% 

figure(1)
plot(mpdp.t,mpdp.Tdec{1,1},'r','LineWidth',2);%�����켣
title('Desired trajectory and actual trajectory')
axis([0 15 -100 100 ]);       %��Ϊ��ȡ
hold on
plot(mpdp.t,mpdp.Tdec{1,2},'b--','LineWidth',2);%���ٹ켣
plot(mpd.t,mpd.Tdec{1,2},'m-.','LineWidth',LineWidth);  %'g-.',
% plot(pid.t,pid.Tdec{1,2},'Color',[0.44,0.74,0.43,1],'LineStyle','-.','LineWidth',LineWidth);
grid on
xlabel('ʱ��(s)');
ylabel('�Ƕ�(1��)');
legend('ref','MPD+P','MPD','PID');


figure(2)                           %%�������
plot(mpdp.t,mpdp.Tdec{1,3},'b--','LineWidth',2);
title('error')
axis([0 15 -250 250 ]);       %��Ϊ��ȡ
hold on
plot(mpd.t,mpd.Tdec{1,3},'m-.','LineWidth',LineWidth);  %'g-.',
% plot(pid.t-1.945,pid.Tdec{1,3},'Color',[0.44,0.74,0.43,1],'LineStyle','-.','LineWidth',LineWidth);
grid on
xlabel('ʱ��(s)');
ylabel('�Ƕ�(0.001��)');
legend('MPD+P','MPD','PID');

%% 
%%����RMSE��MAXE
MPDP_rms_err=rms(mpdp.Tdec{1,3})/1000;
MPDP_max_err=max(abs(mpdp.Tdec{1,3}))/1000;

MPD_rms_err=rms(mpd.Tdec{1,3})/1000;
MPD_max_err=max(abs(mpd.Tdec{1,3}))/1000;

PID_rms_err=rms(pid.Tdec{1,3})/1000;
PID_max_err=max(abs(pid.Tdec{1,3}))/1000;


%% ����MPD+P�����MPD��PID����߾���
%�����MPD����߾���
RMSE_compare_with_MPD=((MPD_rms_err-MPDP_rms_err)/MPD_rms_err)*100
MAXE_compare_with_MPD=(MPD_max_err-MPDP_max_err)/MPD_max_err*100

%�����PID����߾���
RMSE_compare_with_PID=(PID_rms_err-MPDP_rms_err)/PID_rms_err*100
MAXE_compare_with_PID=(PID_max_err-MPDP_max_err)/PID_max_err*100
