t=zeros(1,100000);
v=zeros(1,100000);
[t,v]=textread('C:\Users\hp\Desktop\v.txt','%f %f');
n=length(t);


figure(1);
plot(t, v);
%xlabel('t');
%ylabel('v');
%grid on

