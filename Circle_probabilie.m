function percent = Circle_probabilie(xc,yc,r,x,y,w)
%percent is the percentqge of the robot to be in the circle define by xc xy
% of radius r

nb =length(x);
per=0;
for i =1:nb
    if ( (x(i)-xc)^2 +(y(i)-yc)^2  < r^2)
    per=per + w(i);
    end
end
percent =per;

end

