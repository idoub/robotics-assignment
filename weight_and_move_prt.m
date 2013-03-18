function [x,y,theta,w] = move_and_weight_prt(x,y,theta,w,M,nparticles,transstd,orientstd,nbmeasure,sensorstd,move,moveTheta,sensorRobot,dump)
for j=1:nparticles %repet times number of particles
        
        %-------------------------- Move particles ------------------------
        e = 0 + transstd*randn(1,1); %random Gaussian number with mean 0 and std transstd
        f = 0 + (orientstd*(pi/180)).*randn(1,1); %random Gaussian number with mean 0 and std orienstd
        theta(j)=theta(j)+moveTheta+f;
        x(j)=x(j)+(move+e)*cos(theta(j));
        y(j)=y(j)+(move+e)*sin(theta(j));
    end
        

        %--------------------------Weight of the particles-----------------
        %0/detect the particles out of the map
        InArena=inpolygon(x,y,M(:,1),M(:,2));
        %1/ p(z/x)* p(x)
     
    for j=1:nparticles 
        if InArena(j) == 1 % calculate weight only if the particles is in the map
            sensorParticles = senseParticles(x(j),y(j),theta(j),M,nbmeasure);
            wBefore=w(j);
            w(j)=1;
            for k = 1:nbmeasure %for each measure
                 w(j)= 1/sqrt(2*pi*sensorstd^2) * exp(- (sensorParticles(k) - sensorRobot(k))^2 /(2*sensorstd^2) ) * w(j) + dump;
            end
            w(j)=wBefore*w(j);
        else
            w(j) =0;
        end
        
    end


end

