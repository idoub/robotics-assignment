nparticles=100; %number of particles 
step=300 %length of step in cm

%-----Error particles--------
transstd=5; % translation standard deviation in cm
orientstd=1.5; % orientation standard deviation in degrees
wgtthreshold= 0.01; %Limit to keep the particles
%-----Error sensor-----------
sensorstd = 10;

x=zeros(1,nparticles);
y=zeros(1,nparticles);
theta=zeros(1,nparticles); move=[step 0;step 0;step 0;step 0;]; % sequence of motions
move(:,2)=move(:,2).*(pi/180); % convert degrees to radians 
plot(0,0,'rx','LineWidth',2,'MarkerSize',10) %plot starting position
hold on

%initialisation weight of particles
for i=1:nparticles
    w(i)=1/nparticles;
end

for i=1:size(move,1) % number of steps
    %-----Reading sensor----------
    sensor =100 + sensorstd*randn(1,1); % distance from 0 to a fictional wall + error

    for j=1:nparticles %repet times number of particles
        e = 0 + transstd.*randn(1,1); %random Gaussian number with mean 0 and std transstd
        f = 0 + (orientstd*(pi/180)).*randn(1,1); %random Gaussian number with mean 0 and std orienstd
        theta(j)=theta(j)+move(i,2)+f;
        x(j)=x(j)+(step+e)*cos(theta(j));
        y(j)=y(j)+(step+e)*sin(theta(j));

        %-----Weight of particles-----
        %1/ p(z/x) * p(x)
        distparticles = 100 + y(j);                                        % The distance to the wall
        w(j)= 1/sqrt(2*pi*sensorstd^2) * exp(- (distparticles - sensor)^2/(2*sensorstd^2)) * w(j);
    end
    %------Normalisation of particles--
    S=sum(w);
    for j=1:nparticles
        w(j)=w(j)/S;
    end

    %------Resampling-----------------
    %1/detect the heavy particles to keep
    keep = zeros(0);
    resamp = zeros(0);
    k=1;
    m=1;
    for j =1:nparticles
        if w(j) > wgtthreshold
            keep(k) =j; %record the position of the heavy particles
            k= k+1;
        else
            resamp(m)=j;
            m= m+1;
        end
    end
    %2/resample around heavy particles with the same weight than the assign
    %particles
    for j=1:length(resamp)
        x(resamp(j))=x(keep(mod(j,length(keep))+1));
        Y(resamp(j))=y(keep(mod(j,length(keep))+1));
        theta(resamp(j))=theta(keep(mod(j,length(keep))+1));
        w(resamp(j))=w(keep(mod(j,length(keep))+1));
    end

    %3/ We need to re normalise the weight
    S=sum(w);
    for j=1:nparticles
        w(j)=w(j)/S;
    end

  plot(x,y,'g+'); plot(mean(x),mean(y),'rx','LineWidth',2,'MarkerSize',10)
end