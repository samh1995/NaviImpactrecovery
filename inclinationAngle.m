


yawImpact=0;
rollImpact=0;
pitchImpact=15;
WallNormal=[-1;0;0];
RotMat = quat2rotmat(angle2quat(-(deg2rad(rollImpact)+pi),deg2rad(pitchImpact), deg2rad(yawImpact) ,'xyz')');
ezi=RotMat'*(-[0;0;1]);
eTi=cross([0;0;1],WallNormal);
%   inclination(iBatch)=-sign(dot(WallNormal,bodyFrameZAxisInitial))*acos(((ezi-((ezi)'*eTi)*eTi)'*[0;0;1])/(norm(((ezi-((ezi)'*eTi)*eTi)))));
inclination=acos(((ezi-((ezi)'*eTi)*eTi)'*[0;0;1])/(norm(((ezi-((ezi)'*eTi)*eTi)))))
angle=inclination*180/pi