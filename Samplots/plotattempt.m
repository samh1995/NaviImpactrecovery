clc
clear all
load('samsimjan18t6.mat');
cc=RecAttitudeSucc;
for xx=1:length(cc)
   if cc(xx)==0
       cc(xx)=3;
   end
     if cc(xx)==1
       cc(xx)=7;
   end
    
    
end
x=angle;
y=v_Impact;
scatter(x,y,[],cc,'filled')