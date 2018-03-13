id=zeros(1,100000);
x=zeros(1,100000);
y=zeros(1,100000);
z=zeros(1,100000);
[id,x,y,z]=textread('d:\1.txt','%d,%f,%f,%f');
n=length(x);
%fprintf('%d %f %f %f\n',n,x(n),y(n),z(n));
%disp(num2str(x));
%plot3(x,y,z,'.');

range=zeros(1,n);
for i=1:n
    range(i)=sqrt(x(i)*x(i)+y(i)*y(i)+z(i)*z(i));
end

M=20;

%histogram(range, M, 'range');
%figure(2);
%histogram(z, M, 'height');

figure(1);
hist(range,20);
grid on
xlabel('range');
%ylabel('%');
figure(2);
hist(z,20);
grid on
xlabel('height');
%ylabel('%');



