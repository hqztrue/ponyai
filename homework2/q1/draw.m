id=zeros(1,100000);
x=zeros(1,100000);
y=zeros(1,100000);
z=zeros(1,100000);
[id,x,y,z]=textread('d:\1.txt','%d,%f,%f,%f')
%disp(num2str(x));
plot3(x,y,z,'.');

