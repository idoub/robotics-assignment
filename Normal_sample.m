function [x,y,theta,w,nb] = Normal_sample(xyRes, ThetaRes,M)
%resample the particle on all the map with the resolution xyRez in space 
%and thetaRes in angle. nb is the number of particles
MaxX = max(M(:,1));
MaxY = max(M(:,2));
%1/ build a matrice of the position in a square 0,0,MaxX, MaxY
clear PosSquare
a=round(MaxY/xyRes);
for i =1: round(MaxX/xyRes)
    for j =1:round(MaxY/xyRes)
        posSquarex(j + a*(i-1) ) = i*xyRes;
        posSquarey(j + a*(i-1) ) = j*xyRes;
    end
end

%2/keep those on the map
InArena=inpolygon(posSquarex,posSquarey,M(:,1),M(:,2));

%3/initialise particles at each point in the map
k=0;
for i=1:round(MaxX/xyRes)*round(MaxY/xyRes)
    if InArena(i) == 1
        for j=1:round(360/ThetaRes)
            k=k+1;
            x(k)=posSquarex(i);
            y(k)=posSquarey(i);
            theta(k)= j*ThetaRes*pi/180;
        end
    end
end

nb = k;

for i=1:nb
    w(i)=1/nb;
end


end

