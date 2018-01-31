load('samsimjan25velocityone.mat');
cc=RecAttitudeSucc;
for xx=1:length(cc)
   if cc(xx)==0
       cc(xx)=10;
   end
     if cc(xx)==1
       cc(xx)=7;
   end
    
    
end
x=rollangle;
y=v_Impact;
scatter(x,y,[],cc,'filled')