
clc
clear all
load('samsimjan25prime.mat');
A=angle;
[n,bin]=histc(A,unique(A));
multiple=find(n>1);
index=find(ismember(bin,multiple))
