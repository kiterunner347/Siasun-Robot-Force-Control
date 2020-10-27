clear;clc;
close all;

load('data_dq.mat');
load('data_F.mat');
load('data_q.mat');
load('data_x.mat');

time=F.time;
data_F=F.data;
data_q=q.data;
data_x=x.data;
data_dq=dq.data;

hold on;
plot3(data_x(1:91,1),data_x(1:91,2),data_x(1:91,3));
plot3(data_x(92:905,1),data_x(92:905,2),data_x(92:905,3));
plot3(data_x(906:end,1),data_x(906:end,2),data_x(906:end,3));
legend('接近阶段','寻孔阶段','插孔阶段');
view(-45,45);
xlabel('x/m');ylabel('y/m');zlabel('z/m');

hold off;
figure()
plot(time*3,data_F(:,1:3))
xlabel('时间/s');ylabel('F/N')
legend('Fx','Fy','Fz');

figure()
plot(time*3,data_dq)
