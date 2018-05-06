% for testing purposes, this is the correct answer
x_correct = [30;12;24];
% Generate test dataset
x_rel = [0;0;0];
for i=1:30
    x_rel = [x_rel;1;0.4;0.8];
end

x_rel = x_rel + 0.01*randn(size(x_rel));

x_g = [0;0;0;10;4;8;20;8;16;30;12;24];
x_g = x_g+0.05*randn(12,1);
x_gps = zeros(size(x_rel));
for i =31:30:size(x_rel,1)
    x_gps(i,1) = x_g((i-1)/10+1);
    x_gps(i+1,1) = x_g((i-1)/10+2);
    x_gps(i+2,1) = x_g((i-1)/10+3);
end


%set dummy parameters
s = 1;

ratio = 10;
lambda = [1e-5,1e-4,1e-3,0.01,0.1,1,10,50,100,1000];

%% ----------Function begins----------

A2 = zeros(size(x_rel,1),size(x_rel,1));
A2(1,1)=1;
A2(2,2)=1;
A2(3,3)=1;
for i=4:size(x_rel,1)
    A2(i,i)=1;
    A2(i,i-3)=-1;
end

A1 = zeros(size(x_rel,1),size(x_rel,1));
A1(1,1)=1;
A1(2,2)=1;
A1(3,3)=1;
for i=31:30:size(x_rel,1)
    if(i+1<size(x_rel,1))
        A1(i,i)=1;
        A1(i+1,i+1)=1;
        A1(i+2,i+2)=1;
    end
end


for i =1:size(lambda,2)
    A = A1+lambda(i)*A2;

    b = x_gps + lambda(i)*s*x_rel;

    x_optimal = A\b;
    error(i) = sum((x_optimal(end-2:end)-x_correct).^2);
end


figure(1)
plot(lambda,error);