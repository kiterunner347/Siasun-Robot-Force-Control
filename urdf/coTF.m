clear;clc;close;

rpy=[26.6649   -4.2874 -165.7471];
%利用rpy求旋转矩阵
Trm=rpy2r(rpy,'xyz','deg')
%利用tr2rpy求rpy
Trm=[0 0 1 0;
     0 -1 0 0;
     1 0 0 0;
     0 0 0 1];
 
RPY=tr2rpy(Trm,'xyz','deg')