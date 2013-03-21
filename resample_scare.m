function [xout,yout,thetaout,wout ] = resample_scare(Wgtthreshold,x,y,theta,w,stdangle,stdposition,dist)
%resample the particles given more particle to the heaviest one

%resamp distribution
nb=length(x);
Resamp=ones(length(dist),1); %resample distribution
Resamp=floor(nb*Resamp/sum(Resamp));

%make a matrix to link each particle x,y,theta,wgt
clear Mat;
Mat=[w;x;y;theta]';
Smat=sortrows(Mat,1);
MaxWeight = Smat(1,1);
AbsThreshold = Wgtthreshold*MaxWeight;
I=find(Smat(:,1)>AbsThreshold,1 );
nbresamp=nb-I;

n=1;
%Resampling acccording the distribution
for i=1:length(Resamp)
    for j=1:Resamp(i)
        Smat(n,:)=Smat(nb-i+1,:);
        n=n+1;
    end
end
%Give the rest from the heaviest particles
for i=n:nbresamp
    Smat(i,:)=Smat(nb-i+n,:);
end
%move the redistribuate particles around the solution
for i=1:nbresamp
    Smat(i,2)=Smat(i,2)+rand(1,1)*stdposition;
    Smat(i,3)=Smat(i,3)+rand(1,1)*stdposition;
    Smat(i,4)=Smat(i,4)+rand(1,1)*stdangle;
end
xout=Smat(:,2)';
yout=Smat(:,3)';
thetaout=Smat(:,4)';
w1=Smat(:,1)';

Resample
        S=sum(w1);
wout=w1/S;
end


        
        

