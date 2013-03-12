function percentage = acuracy(x,y,xmax,ymax,nparticle,R)
%Acurancy return the percentage of particles in a circle a radius R around
%the heaviest particles
count = 0;
for i =1:nparticle
    if ( (x(i)-xmax)^2 + (y(i)-ymax)^2) < R^2
        count = count +1;
    end
percentage = count/nparticle; 
end

