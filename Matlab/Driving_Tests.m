%0.5 Hz update experiment, running GPS_funtion_tester.ino

%Data output read from serial monitor.
% testing the thingy
% 0
% ************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
% TEST OF CONTROLLER 1: DRIVE AROUND WITH GPS OUT THE WINDOW!
% location of origin: 33.85, -118.13
% altitude: 12.80
% data to come will be in the following format:
% x,   y,   z,   region;
home_depot = [0.0,0.0,0.0,2;
0.0,0.0,0.0,2;
0.0,0.0,0.0,2;
0.0,0.0,0.0,2;
0.0,0.0,0.0,2;
16.2,9.7,0.0,2;
20.8,34.8,0.0,2;
25.4,62.6,0.0,2;
25.4,91.8,0.0,2;
25.4,112.6,0.0,2;
30.0,123.8,0.0,2;
50.8,126.6,0.0,2;
64.7,125.2,0.0,2;
80.9,125.2,0.0,2;
115.5,125.2,0.0,2;
159.4,125.2,0.0,1;
198.7,123.8,0.0,1;
212.5,112.6,0.0,1;
217.1,90.4,0.0,1;
217.1,73.7,0.0,1;
217.1,65.4,0.0,1;
212.5,55.6,0.0,1;
212.5,50.1,0.0,1;
207.9,43.1,0.0,1;
207.9,23.6,0.0,1;
207.9,-1.4,0.0,1;
217.1,-19.5,0.0,1;
221.8,-37.5,0.0,8;
212.5,-64.0,0.0,8;
203.3,-98.7,0.0,8;
198.7,-112.6,0.0,8;
198.7,-123.8,0.0,8;
182.5,-139.1,0.0,8;
154.8,-148.8,0.0,7;
134.0,-154.4,0.0,7;
120.1,-159.9,0.0,7;
99.3,-162.7,0.0,7;
80.9,-169.7,0.0,7;
50.8,-176.6,0.0,7;
6.9,-178.0,0.0,7;
-39.3,-176.6,0.0,6;
-83.2,-176.6,0.0,6;
-131.7,-176.6,0.0,6;
-170.9,-176.6,0.0,6;
-201.0,-175.2,0.0,5;
-231.0,-176.6,0.0,5;
-249.5,-171.1,0.0,5;
-261.0,-148.8,0.0,5;
-261.0,-108.5,0.0,5;
-254.1,-58.4,0.0,5;
-254.1,-5.6,0.0,4;
-249.5,44.5,0.0,4;
-240.2,82.1,0.0,4;
-214.8,107.1,0.0,4;
-175.6,125.2,0.0,4;
-147.8,127.9,0.0,4;
-127.1,126.6,0.0,3;
-97.0,126.6,0.0,3;
-53.1,126.6,0.0,3;
-4.6,126.6,0.0,2;
41.6,125.2,0.0,2;
64.7,122.4,0.0,2;
80.9,114.0,0.0,2;
80.9,90.4,0.0,2;
69.3,59.8,0.0,1;
69.3,27.8,0.0,1;
85.5,8.3,0.0,1;
104.0,8.3,0.0,1;
104.0,9.7,0.0,1;
108.6,11.1,0.0,1;
];

signal_hill = [0.0,0.0,0.0,2;
0.0,0.0,0.3,2;
0.0,0.0,0.7,2;
0.0,0.0,1.3,2;
0.0,-1.4,1.6,2;
0.0,-1.4,2.0,2;
0.0,-1.4,3.0,2;
0.0,-1.4,3.9,2;
0.0,-2.8,5.2,2;
0.0,-5.6,6.9,2;
0.0,-5.6,8.2,2;
0.0,-7.0,9.5,2;
6.9,-15.3,11.2,2;
11.6,-34.8,13.8,7;
11.6,-68.1,14.8,7;
11.6,-109.9,12.5,7;
11.6,-153.0,9.5,7;
11.6,-187.7,6.6,7;
11.6,-214.2,3.9,7;
6.9,-230.9,1.6,7;
0.0,-250.3,-1.0,7;
-23.1,-262.8,-3.3,7;
-57.8,-265.6,-6.2,6;
-101.6,-260.1,-9.8,6;
-152.5,-247.6,-14.4,6;
-201.0,-230.9,-19.0,6;
-256.4,-211.4,-23.3,5;
-304.9,-187.7,-27.2,5;
-348.8,-150.2,-31.2,5;
-378.8,-101.5,-35.1,5;
-392.7,-44.5,-40.0,5;
-383.5,9.7,-44.9,4;
-362.7,64.0,-49.5,4;
-335.0,115.4,-55.1,4;
-309.5,165.5,-58.7,4;
-284.1,219.7,-66.3,4;
-270.3,275.4,-75.1,3;
-274.9,333.8,-83.6,3;
-300.3,390.8,-93.2,3;
-335.0,443.6,-103.0,3;
-358.1,464.5,-107.6,3;
-418.1,492.3,-117.4,3;
-487.4,502.1,-127.3,3;
-591.4,499.3,-142.0,4;
-619.1,496.5,-145.6,4;
-674.5,493.7,-151.5,4;
-718.4,490.9,-155.5,4;
-727.7,490.9,-156.5,4;
-727.7,490.9,-156.5,4;
-727.7,490.9,-156.8,4;
];

figure(1);
plot(home_depot(:,1)/100,'b');hold on;
plot(home_depot(:,2)/100,'g');
plot(home_depot(:,4),'r');hold off;
legend('x coordinate / 100','y coordinate / 100','region (1-8)');
title('Drive around home depot parking lot experiment')

figure(2);
plot(signal_hill(:,1)/100,'b');hold on;
plot(signal_hill(:,2)/100,'g');
plot(signal_hill(:,3)/10,'k');
plot(signal_hill(:,4),'r');hold off;
legend('x coordinate / 100','y coordinate / 100', 'z coordinate / 100)','region (1-8)');
title('Drive down signal hill experiment')