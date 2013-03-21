nparticles=100; %number of particles
step=300 %length of step in cm
transstd=5; % translation standard deviation in cm
orientstd=1.5; % orientation standard deviation in degrees
x=zeros(1,nparticles);
y=zeros(1,nparticles);
theta=zeros(1,nparticles);
move=[step 0;step 0;step 0;step 0]; % sequence of motions
move(:,2)=move(:,2).*(pi/180); % convert degrees to radians
plot(0,0,'rx','LineWidth',2,'MarkerSize',10) %plot starting position
hold on
for i=1:size(move,1) % number of steps
    for j=1:nparticles %repet times number of particles
        e = 0 + transstd.*randn(1,1); %random Gaussian number with mean 0 and std transstd
        f =  0 + (orientstd*(pi/180)).*randn(1,1); %random Gaussian number with mean 0 and std orienstd
        theta(j)=theta(j)+move(i,2)+f;
        x(j)=x(j)+(step+e)*cos(theta(j));
        y(j)=y(j)+(step+e)*sin(theta(j));
    end
plot(x,y,'.'); plot(mean(x),mean(y),'rx','LineWidth',2,'MarkerSize',10)
end