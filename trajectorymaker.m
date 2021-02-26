clc; close all; clear vars;

time = [0:0.001:5]';

Lp0 = 0;
Lp1 = -2;
Lp2 = -2;
Lp3 = 0;
Lp4 = 2;
Lp5 = 2;

Rp0 = 0;
Rp1 = 5;
Rp2 = 7;
Rp3 = 4;
Rp4 = 2;
Rp5 = 5;

Lpart1 = linspace(Lp0,Lp1,1000);
Lpart2 = linspace(Lp1,Lp2,1000);
Lpart3 = linspace(Lp2,Lp3,1000);
Lpart4 = linspace(Lp3,Lp4,1000);
Lpart5 = linspace(Lp4,Lp5,1000);

Rpart1 = linspace(Rp0,Rp1,1000);
Rpart2 = linspace(Rp1,Rp2,1000);
Rpart3 = linspace(Rp2,Rp3,1000);
Rpart4 = linspace(Rp3,Rp4,1000);
Rpart5 = linspace(Rp4,Rp5,1000);


leftV = [0; Lpart1'; Lpart2'; Lpart3'; Lpart4'; Lpart5'];
rightV = [0; Rpart1'; Rpart2'; Rpart3'; Rpart4'; Rpart5'];

data = table(time, leftV, rightV);
writetable(data,'STrajectory6SM.txt')

