
clear all
clc
iBatch = 0;
pitchangle=[];
rollangle=[];

inclination=[];
iRecord=0;
for rollImpact = 0:1:20
for pitchImpact =0:1:20                      
iBatch=iBatch+1;
yawImpact=45;
WallNormal=[-1;0;0];
RotMat = quat2rotmat(angle2quat(-(deg2rad(rollImpact)+pi),deg2rad(pitchImpact), deg2rad(yawImpact) ,'xyz')');
ezi=RotMat'*(-[0;0;1]);
eTi=cross([0;0;1],WallNormal);
%   inclination(iBatch)=-sign(dot(WallNormal,bodyFrameZAxisInitial))*acos(((ezi-((ezi)'*eTi)*eTi)'*[0;0;1])/(norm(((ezi-((ezi)'*eTi)*eTi)))));
inclination=acos(((ezi-((ezi)'*eTi)*eTi)'*[0;0;1])/(norm(((ezi-((ezi)'*eTi)*eTi)))));
angle(iBatch)=inclination*180/pi;
if angle(iBatch)>14
    if angle(iBatch)<18
   iRecord=iRecord+1;
   anglecritical(1,iRecord)=rollImpact;
   anglecritical(2,iRecord)=pitchImpact;
   anglecritical(3,iRecord)=angle(iBatch);
   rollcritical(iRecord)=rollImpact;
   pitchcritical(iRecord)=pitchImpact;
%    anglecritical(iRecord)=angle(iBatch);
    end
end
end
end
