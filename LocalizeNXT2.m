clear all
figure(10)
hold on
%%%%%%%%%%%%%%%%%%%%%%%%%% INITIALISATION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%-------------------------Map definition-----------------------------------

M=[0,0;60,0;60,45;45,45;45,59;106,59;106,105;0,105];
T=[25,80];
S=[25,25];
step=10;
%nextstep = T;


%-------------------------Robot simulation---------------------------------
step=10; %length of step in cm
%RealRobot=RobotModel(80,80, 13 *pi/180);%robot use for simulating captor
%         plot(RealRobot.x,RealRobot.y,'or');
KnowRobot=RobotModel(0,0,pi/2); %Robot use for pathfinding                            
nxt = NXTRobot(0,0,0);
nxt.initAll();

%-------------------------Error particles----------------------------------
transstd=0.5; % translation standard deviation in cm
orientstd=0.5; % orientation standard deviation in degrees
Wgtthreshold= 0.30;% relative limit to keep the particles 
dump =0; %anti dumping coef
ScanLarge=3; % how far the resample particle are randomly distributed aroud heavy solution in space
ScanTheta=0.8; % how far the resample particle are randomly distributed aroud heavy solution in space
dist =100; %number of particale that beneficiat of the linear resample( heavy =. more particle in linear way)
lostthreshold=0;% 2e-8;   % the more high it is the easier it is to get lost
%-------------------------------Sensor------------------------------------
nbmeasure = 5; %number of measurement
sensorstd = 2; % error of sensor for calculation
%----------------------- initialisation of the particles-------------------
xyRes = 8;
ThetaRes = 50;
[x,y,w,theta,nparticles] = Normal_sample(xyRes, ThetaRes,M);


 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%% END INITIALISATION %%%%%%%%%%%%%%%%%%%%%%%%%%
move=0;
moveTheta=0;
stop = false;
while stop == false, % number of steps
    
    %%%%%%%%%%%%%%%%%%%%%%%%%   ROBOT   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

  
   %%%%%%%%%%%%%%%%%%%%%%%%    PARTICLES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   lost = true; % enter the loop
   lost1 = false;
   while (lost == true)
        %-----Reading Robot sensor----------
        [sensorRobot angleError] = nxt.sense(nbmeasure,60)
%         angleError=zeros(nbmeasure,1)
%         for h=1:nbmeasure
%             sensorRobot(h) = sensorRobot(h) + sensorstdReal* randn(1,1);
%         end
        %%%%%%%%%%%%%%%%%%%%%%%%    PARTICLES %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        [x,y,theta,w] = weight_and_move_prt(x,y,theta,w,M,nparticles,transstd,orientstd,nbmeasure,sensorstd,move,moveTheta,sensorRobot,dump,angleError);
        Maxweight= max(w);
        %------------------------- Normalisation of particles -----------------
            S=sum(w);
            w=w/S;
        %detect if lost
        if Maxweight < 1/nparticles*lostthreshold
            clear x
            clear y 
            clear theta
            clear w
           [x,y,w,theta,nparticles] = Normal_sample(xyRes, ThetaRes,M)
            clf(figure)
            plot(x,y,'+')
            disp('lost');
           if lost1 == true %anti stuck in lost place
               lost=false
           end
           lost1=true
        else
            lost = false;
             %------------------------- Resampling ---------------------------------
            [x,y,theta,w ]=resample(Wgtthreshold,x,y,theta,w,ScanTheta*orientstd,ScanLarge*transstd,dist);
            disp('notlost')

        end
        
    end
   

 %%%%%%%%%%%%%%%%%%%%%%%%%   ROBOT MOVE  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %-----------------------  change position  ------------------------------
  [~,MaxInd]=max(w); %MaxInd is the indice of the heaviest particle
  KnowRobot=RobotModel(x(MaxInd),y(MaxInd),theta(MaxInd));
  %nxt.position(KnowRobot.x,KnowRobot.y,KnowRobot.theta);
  nxt.position(x(MaxInd),y(MaxInd),theta(MaxInd));
  newPath = Pathfinding(M, [x(MaxInd) y(MaxInd)], T);
  
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% PLOTTING %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
  waitforbuttonpress; % enable to plot step by step
  hold on
  plot(M(:,1),M(:,2));  %map 
  plot(T(1),T(2),'*r'); %plot goal
  plot(x,y,'b+');    %particles
  plot(KnowRobot.x,KnowRobot.y,'xr');%know position of the robot 
  legend('True position','map','goal','paticles');
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%% END PLOTTING %%%%%%%%%%%%%%%%%%%%%%%%
    
  %-----------------------  Path finding  ---------------------------------

  %-----------------------  Motion  ---------------------------------------
  if ~isempty(newPath)
      nextstep = newPath;
  end
  dist = sqrt( (nextstep(1,1)-KnowRobot.x)^2 + (nextstep(1,2)-KnowRobot.y)^2 );
%   if dist < 5
%       stop = true;
%   end
  if dist > step
      xgo= (nextstep(1,1)-KnowRobot.x)*step/dist + KnowRobot.x;
      ygo= (nextstep(1,2)-KnowRobot.y)*step/dist + KnowRobot.y;
      [move,moveTheta] = nxt.goto(xgo,ygo)
  else
      [move,moveTheta] = nxt.goto(nextstep(1,1),nextstep(1,2));
  end
  
%------------------------ Simulated real  robot----------------------------
% Evaluate if we are arrive
per = Circle_probabilie(T(1),T(2),1.4,x,y,w)
if per > 0.5
    stop = true
end

                                                                           
end