close all;
clear all;
name1='RecordData 2021-08-02 10-52-28';
namestr=string(name1)+'.txt';
Data=load(namestr);
% plot(T(:,1));
len=length(Data);
% for i=2:len
%     if(Data(i-1,2)<0&&Data(i,2)==0&&Data(i+1,2)>0)
%       a=i-1;
%       break;
%     end
% end
% %清空前面的周�?

% Data(1:a,:)=[];
figure(1)
plot(Data(:,1)/1);
figure(2)
plot(Data(:,2)/1);
figure(3)
plot(Data(:,3)/1000);
figure(4)
plot(Data(:,4)/1000);
% Data(:,3)=Data(:,3)-10;
% Data(:,5)=Data(:,1)/10-Data(:,3)/1000;%实际位置
namesave=string(name1)+'.mat';
save(namesave,'Data');