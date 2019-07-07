data = csvread('fl.csv');
x = data(:,1);
y = data(:,2);

coeff = polyfit(x,y,3)
x2 = linspace(-255,255);
y2 = polyval(coeff,x2);

plot(x,y,x2,y2)
title('Motor Characteristics curve');
xlabel('PWM');
ylabel('RPM');