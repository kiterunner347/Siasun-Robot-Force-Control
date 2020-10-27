clear;clc;

n=200;tw=10;phase=5;
t=linspace(0,tw,n);
t1=linspace(0,tw/phase,n/phase);
theta1=0.2*t1;
theta2=0.4*ones(1,n/phase);
theta3=0.4-0.2*t1;
theta4=zeros(1,n/phase);
theta5=0.2*t1;

sgPd.time=t;
sgPd.signals.values=transpose(ones(6,1)*[theta1,theta2,theta3,theta4,theta5]);
sgPd.signals.dimensions=6;