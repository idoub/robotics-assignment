clear all
figure(10)
hold on
%%%%%%%%%%%%%%%%%%%%%%%%%% INITIALISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%-------------------------Map definition-----------------------------------

M=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105]    

%-------------------------Robot simulation---------------------------------
step=10; %length of step in cm
Robot=RobotModel(55,25,pi + 13 *pi/180);%start position and orientation
move=[step 0;step -13;step 0;step -90; step 0;step 0;step 0;step 0;step -90;step 0; step 0; step 0; step 0; step 0]; % sequence of motions
move(:,2)=move(:,2).*(pi/180); % convert degrees to radians 

%-------------------------Error particles----------------------------------
transstd=0.5; % translation standard deviation in cm
orientstd=1.5; % orientation standard deviation in degrees
Wgtthreshold= 0.50; % relative limit to keep the particles 
dump =0; %anti dumping coef
ScanLarge=4; % how far the resample particle are randomly distributed aroud heavy solution in space
ScanTheta=0; % how far the resample particle are randomly distributed aroud heavy solution in space
%-------------------------------Sensor------------------------------------
nbmeasure = 4; %number of measurement
sensorstd = 10; % error of sensor for calculation
sensorstdReal = 4; %real error of sensor 
%----------------------- initialisation of the particles-------------------
xyRes = 10;
ThetaRes = 36;

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

nparticles = k
plot(x,y,'+')
plot(Robot.x,Robot.y,'or'); 

%1/Weight
for i=1:nparticles
    w(i)=1/nparticles;
end
%2/position and orientation
%   (uniform distribution in the smallest square containig the map)

 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% END INITIALISATION %%%%%%%%%%%%%%%%%%%%%%%%%%

for i=1:size(move,1) % number of steps
    
    %%%%%%%%%%%%%%%%%%%%%%%%%   ROBOT   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %-----perfect Robot simulation-------
    left(Robot,move(i,2))  %rotate
    forward(Robot,step) %move

    %-----Reading Robot sensor----------
    sensorRobot =sense(Robot,M,nbmeasure); % distance from 0 to a fictional wall + error
    for h=1:nbmeasure
        sensorRobot(h) = sensorRobot(h) + sensorstdReal* randn(1,1);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%    PARTICLES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    for j=1:nparticles %repet times number of particles
        
        %-------------------------- Move particles ------------------------
        e = 0 + transstd*randn(1,1); %random Gaussian number with mean 0 and std transstd
        f = 0 + (orientstd*(pi/180)).*randn(1,1); %random Gaussian number with mean 0 and std orienstd
        theta(j)=theta(j)+move(i,2)+f;
        x(j)=x(j)+(step+e)*cos(theta(j));
        y(j)=y(j)+(step+e)*sin(theta(j));
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
                 w(j)= 1/sqrt(2*pi*sensorstd^2) * exp(- (sensorParticles(k) - sensorRobot(k))^2 /(2*sensorstd^2 + dump) ) * w(j);
            end
            w(j)=wBefore*w(j);
        else
            w(j) =0;
        end
        
    end

    %------------------------- Normalisation of particles -----------------
    S=sum(w);
    w=w/S;

    %------------------------- Resampling ---------------------------------

    %1/detect the heavy and in the map particles to keep
    keep = zeros(0);
    resamp = zeros(0);
    k=1;
    m=1;
    %Normalize the threshold
    MaxWeight = max(w);
    AbsThreshold = Wgtthreshold*MaxWeight;
    for j =1:nparticles
        if (w(j) > AbsThreshold) 
            keep(k) =j; %record the position of the heavy particles
            k= k+1;
        else
            resamp(m)=j;
            m= m+1;
        end
    end
    disp('keep = ');
    disp(length(keep));
    %3/resample around heavy particles with the same weight than the assign
    %particles. If keep is empty we are lost and make a random distribution
    %one again
    if k>1 % keep is not empty
        for j=1:length(resamp)
            x(resamp(j))=x(keep(mod(j,length(keep))+1)) + ScanLarge*rand(1,1)*transstd;
            y(resamp(j))=y(keep(mod(j,length(keep))+1)) + ScanLarge*rand(1,1)*transstd;
            theta(resamp(j))=theta(keep(mod(j,length(keep))+1)) + ScanTheta*rand(1,1)*orientstd;
            w(resamp(j))=w(keep(mod(j,length(keep))+1));
        end
    else %(keep is empty)
        x = unifrnd(0,MaxX,1,nparticles); 
        y = unifrnd(10,MaxY,1,nparticles);
        theta = unifrnd(0,2*pi,1,nparticles);
        for m=1:nparticles
             w(m)=1/nparticles;
        end
    end

    %3/ We need to re normalise the weight
    S=sum(w);
    for j=1:nparticles
        w(j)=w(j)/S;
    end
  
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  %waitforbuttonpress; % enable to plot step by step
  hold on
  plot(M(:,1),M(:,2)); 
  plot(x,y,'b+');
  [~,MaxInd]=max(w);
  plot(x(MaxInd),y(MaxInd),'xr');
  plot(Robot.x,Robot.y,'or');  
  
                                                                           
end
                                                                             