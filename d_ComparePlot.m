close all;
clear all;
xM0=0.061;
xm0=-0.061;
fs1=200;%����Ƶ�ʣ���ʱ����ʱ��Ϊ1/fs=0.005s,ʹ���Һ�������Ϊ5s,��Ӧ�������Һ���Ƶ��0.2Hz
fs2=200;

constraint=load('RecordData 2021-07-27 19-05-33');    %%�޸���
noconstraint=load('RecordData 2021-06-10 17-19-27');

% constraint=load('constrain_sin');    %%�޸���
% noconstraint=load('noconstrain2jiaohaode_sin');
% pid=load('PID_step_kulun_nofuzai_data1_e592.mat');


% pdp=load('MPDP_step_kulun_fuzai20_450_0_16_5_3_20_data3_e34');    %%�и���
% mpd=load('MPD_step_kulun_fuzai20_data1_e-295.mat');
% pid=load('PID_step_kulun_fuzai20_data1_e593.mat');

%�����˲�
windowSize =10;         %���ڴ�С
b = (1/windowSize)*ones(1,windowSize); 
a = 1;
% y1 = filter(b,a,Tdec{1,3});
CONSTRAINT_current = filter(b,a,constraint.Data(:,4));
NOCONSTRAINT_current = filter(b,a,noconstraint.Data(:,4));
% PID_current = filter(b,a,pid.Data(:,4));
%%
t1=(1:size(constraint.Data(:,1),1))/fs1;%%ʱ��
t2=(1:size(noconstraint.Data(:,1),1))/fs2;%%ʱ��
stopTime=25;
figure(1)%%��Լ���ĸ���ͼ��
set(gcf,'color','white')
plot(t1,xM0*1000+0*t1,'m','LineWidth',2)
hold on
plot(t1,xm0*1000+0*t1,'r--','LineWidth',2)

hold on
% plot(t2,noconstraint.Data(:,2)/10,'b','LineStyle','-.','LineWidth',2)%%��Լ��ʵ��λ��

hold on
plot(t1,constraint.Data(:,2)/10,'Color',[0.513 0.435 1],'LineStyle','-.','LineWidth',2)%%��Լ��ʵ��λ��
% plot(t,sout(:,1))
hold on
plot(t1,constraint.Data(:,1)/10,'c','LineWidth',2)%%����λ��
% legend('upper bound','lower bound','x without constraint','x with constraint','desired x');
legend('upper bound','lower bound','x with constraint','desired x');
xlabel('Time(s)');
ylabel('Displacement (��)');
xlim([0,stopTime]);


figure(2)%%��Լ���ĸ���ͼ��
set(gcf,'color','white')
plot(t2,xM0*1000+0*t2,'m','LineWidth',2)
hold on
plot(t2,xm0*1000+0*t2,'r--','LineWidth',2)
hold on
plot(t2,noconstraint.Data(:,2)/10,'Color',[0.513 0.435 1],'LineStyle','-.','LineWidth',2)%%��Լ��ʵ��λ��
% plot(t,sout(:,1))
hold on
plot(t2,noconstraint.Data(:,1)/10,'c','LineWidth',2)%%����λ��
legend('upper bound','lower bound','x without constraint','desired x');
xlabel('Time(s)');
ylabel('Displacement (��)');
xlim([0,stopTime]);

figure(3)%%��Լ���ĸ������
set(gcf,'color','white')
plot(t1,0*t1,'m','LineWidth',2)
hold on
plot(t1,constraint.Data(:,3)/1000,'Color',[0.513 0.435 1],'LineStyle','-.','LineWidth',2)%%��Լ�����
legend('desired e','x with constraint');
xlabel('Time(s)');
ylabel('Error (��)');
xlim([0,stopTime]);


figure(4)%%��Լ���ĸ������
set(gcf,'color','white')
plot(t2,0*t2,'m','LineWidth',2)
hold on
plot(t2,noconstraint.Data(:,3)/1000,'Color',[0.513 0.435 1],'LineStyle','-.','LineWidth',2)%%��Լ�����
legend('desired e','x without constraint');
xlabel('Time(s)');
ylabel('Error (��)');
xlim([0,stopTime]);


figure(5)%%��Լ���ĸ��ٵ���
set(gcf,'color','white')
plot(t1,constraint.Data(:,4)/1000,'Color',[0.513 0.435 1],'LineStyle','-.','LineWidth',2)%%��Լ��ʵ�ʵ���
legend('x with constraint');
xlabel('Time(s)');ylabel('Current(A)');
xlim([0,stopTime]);


figure(6)%%��Լ���ĸ��ٵ���
set(gcf,'color','white')
plot(t2,noconstraint.Data(:,4)/1000,'Color',[0.513 0.435 1],'LineStyle','-.','LineWidth',2)%%��Լ��ʵ�ʵ���
legend('x without constraint');
xlabel('Time(s)');ylabel('Current(A)');
xlim([0,stopTime]);


%%

% %�������
% linewidth=1.5;
% figure
% % plot(t,0*t+160*sin((pi/2)*t)/101*100) ;
% plot((1:size(pdp.Data(:,1),1))/fs,pdp.Data(:,1)/10,'LineWidth',linewidth,'LineStyle','--');
% hold on
% plot((1:size(pdp.Data(:,1),1))/fs-0.01,pdp.Data(:,5),'m','LineWidth',linewidth);%%��pdp��ͼ������ƽ��
% hold on
% plot((1:size(mpd.Data(:,1),1))/fs,mpd.Data(:,5),'g','LineWidth',linewidth);
% hold on
% plot((1:size(pid.Data(:,1),1))/fs,pid.Data(:,5),'LineWidth',linewidth);
% hold on
% legend('Ref','MPDP','MPD','PID');
% xlabel('Time(s)');ylabel('Angular displacement(°)');
% axis([0 10 0 30]);
% 
% 
% %�������
% figure
% stopTime=10;
% subplot(3,1,1),plot((1:size(pdp.Data(:,1),1))/fs,pdp.Data(:,3)/1000,'LineWidth',linewidth);
% grid on;legend('MPDP');xlabel('Time /s'); ylabel('Error(deg)','interpreter','tex');
% xlim([0,stopTime]);
% ylim([min(pdp.Data(:,3)/1000),max(pdp.Data(:,3)/1000)]);
% 
% subplot(3,1,2),plot((1:size(mpd.Data(:,1),1))/fs,mpd.Data(:,3)/1000,'LineWidth',linewidth);
% grid on;legend('MPD');xlabel('Time /s'); ylabel('Error(deg)','interpreter','tex');
% xlim([0,stopTime]);
% ylim([min(mpd.Data(:,3)/1000),max(mpd.Data(:,3)/1000)]);
% 
% subplot(3,1,3),plot((1:size(pid.Data(:,1),1))/fs,-pid.Data(:,3)/1000,'LineWidth',linewidth);
% grid on;legend('PID');xlabel('Time /s'); ylabel('Error(deg)','interpreter','tex');
% xlim([0,stopTime]);
% ylim([min(pid.Data(:,3)/1000),max(pid.Data(:,3)/1000)]);
% 
% % legend('MPDP','MPD','PID');
% % xlabel('Time(s)');ylabel('Angular displacement(°)');
% % % axis([0 20 -0.2 0.2]);
% 
% %����
% figure
% hold on
% plot((1:size(pdp.Data(:,1),1))/fs,MPDP_current/1000,'LineWidth',linewidth);
% hold on
% plot((1:size(mpd.Data(:,1),1))/fs,MPD_current/1000,'LineWidth',linewidth);
% hold on
% plot((1:size(pid.Data(:,1),1))/fs,PID_current/1000,'LineWidth',linewidth);
% hold on
% legend('MPDP','MPD','PID');
% xlabel('Time(s)');ylabel('Current(A)');
% axis([0 20 -3 4]);



%% 
%%����RMSE��MAXE
% MPDP_rms_err=rms(pdp.Data(:,3))/1000;
% MPDP_max_err=max(abs(pdp.Data(:,3)))/1000;
% 
% MPD_rms_err=rms(mpd.Data(:,3))/1000;
% MPD_max_err=max(abs(mpd.Data(:,3)))/1000;
% 
% PID_rms_err=rms(pid.Data(:,3))/1000;
% PID_max_err=max(abs(pid.Data(:,3)))/1000;
% 
% 
% % %% ����MPD+P�����MPD��PID����߾���
% RMSE_compare_with_MPD=((MPD_rms_err-MPDP_rms_err)/MPD_rms_err)*100
% MAXE_compare_with_MPD=(MPD_max_err-MPDP_max_err)/MPD_max_err*100
% 
% RMSE_compare_with_PID=(PID_rms_err-MPDP_rms_err)/PID_rms_err*100
% MAXE_compare_with_PID=(PID_max_err-MPDP_max_err)/PID_max_err*100
% 
% %%�޸���
% save('increase_nofuzai.mat','MPDP_rms_err','MPDP_max_err','MPD_rms_err','MPD_max_err',...
%     'PID_rms_err','PID_max_err','RMSE_compare_with_MPD','MAXE_compare_with_MPD',...
%     'RMSE_compare_with_PID','MAXE_compare_with_PID');
% 
% 
% %%�и���
% % save('increase_fuzai.mat','MPDP_rms_err','MPDP_max_err','MPD_rms_err','MPD_max_err',...
% %     'PID_rms_err','PID_max_err','RMSE_compare_with_MPD','MAXE_compare_with_MPD',...
% %     'RMSE_compare_with_PID','MAXE_compare_with_PID');
