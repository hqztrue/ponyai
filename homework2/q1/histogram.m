function his=histogram(a, M, label)
    n=length(a);
	his=zeros(1,M);
    min_=min(a);
    max_=max(a);
    for i=1:n
        j=ceil((a(i)-min_)/(max_-min_)*M);
        j=max(j,1);
        his(j)=his(j)+1.0/n;
    end
    X=1:M;
    plot(X,his,'--b');
    grid on
    xlabel(label)
    ylabel('%')
end

