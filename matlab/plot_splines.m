REF_PATH_Xa = [-0.0025 0.0025 0];
REF_PATH_Xb = [0 -0.075 0];
REF_PATH_Xc = [1.25 10 -0.25];
REF_PATH_Xd = [0 10 10];
REF_PATH_Ya = [0.0025 -0.0025 0];
REF_PATH_Yb = [0 0.075 0];
REF_PATH_Yc = [-0.25 0.5 1.25];
REF_PATH_Yd = [0 0.5 10];

x=[];
y=[];
k=1;
for i=0:0.1:10
   x(k) = REF_PATH_Xa(1)*i^3 +REF_PATH_Xb(1)*i^2+REF_PATH_Xc(1)*i+REF_PATH_Xd(1); 
   y(k) = REF_PATH_Ya(1)*i^3 +REF_PATH_Yb(1)*i^2+REF_PATH_Yc(1)*i+REF_PATH_Yd(1); 
   k=k+1;
end
for i=10:0.1:20
   x(k) = REF_PATH_Xa(2)*i^3 +REF_PATH_Xb(2)*i^2+REF_PATH_Xc(2)*i+REF_PATH_Xd(2); 
   y(k) = REF_PATH_Ya(2)*i^3 +REF_PATH_Yb(2)*i^2+REF_PATH_Yc(2)*i+REF_PATH_Yd(2); 
   k=k+1;
end
figure;
hold on;
plot(x,y);

s=[0 10 20];
x_spline=[0 10 10];
y_spline =[0 0 10];
Sx=spline(s,x_spline)
Sy=spline(s,y_spline)