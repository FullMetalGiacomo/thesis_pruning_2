%% Calibration Results from 14/03/2023 and 13/03/2023
clc
close all
clear al
%% %DATA
% 13/03/2023 "uncontrolled positions, mixed angles and positions" 
%checked and correct
T1_uncontrolled = [-0.0320037, -0.109751, -0.113909];
RPY1_uncontrolled = [-0.176643, 1.5364618, -1.7661877]; % order ZYX
Quat1_uncontrolled = [-0.4946 0.4884 -0.5146 0.5021]; % xyzw

%checked and correct
T2_uncontrolled = [-0.018, -0.094, -0.117];
RPY2_uncontrolled = [0.075, 1.524, -1.543];
Quat2_uncontrolled = [0.500, 0.476, -0.523, 0.500] ; % xyzw

%checked and correct
T3_uncontrolled = [-0.012, -0.101, -0.127];
RPY3_uncontrolled = [0.306, 1.533, -1.310];
Quat3_uncontrolled =  [0.505, 0.477, -0.517, 0.500] ; % xyzw

%checked and correct 
T4_uncontrolled = [-0.028, -0.090, -0.104] ;
RPY4_uncontrolled = [0.422, 1.533, -1.215];
Quat4_uncontrolled = [-0.511, -0.471, 0.521, -0.495]  ; % xyzw

%checked and correct
T5_uncontrolled = [-0.047, -0.113, -0.126] ;
RPY5_uncontrolled = [-2.671, 1.556, 1.991];
Quat5_uncontrolled =  [0.514, 0.492, -0.511, 0.482] ; % xyzw

%%%%%%%%%% checking the error of the software for a random configuration.
calibT1_uncontrolled = [-0.390509760733171, -0.02734325523949511, -0.08091369880767295]; 
calibRPY1_uncontrolled = [0.3068 0.3489 0.5472];
calibQuat1_uncontrolled = [0.09855 0.2058 0.2375 0.9442] % xyzw
calibT2_uncontrolled = [-0.4094332980192639, -0.031995695449533686, -0.07474587885799532];
calibRPY2_uncontrolled = [0.3124 0.3672 0.5491];
calibQuat2_uncontrolled = [0.09833 0.2151 0.2360 0.9425] % xyzw

%difference on 3d rotation
q1=quaternion(calibRPY1_uncontrolled,'euler','zyx','frame')
q2=quaternion(calibRPY2_uncontrolled,'euler','zyx','frame')
%(https://es.mathworks.com/help/fusion/ref/quaternion.dist.html)
angularDistancerad = 2*acos(abs(parts(q1*conj(q2)))) % in radiants
angularDistancedeg = rad2deg(angularDistancerad) % in degrees
%difference on displacement
diff_vector=abs(calibT1_uncontrolled-calibT2_uncontrolled)
euclidean_dist=sqrt(sum((diff_vector) .^ 2))

%% errors on displacement and rotations 13/03
T_matrix=[T1_uncontrolled;
          T2_uncontrolled;
          T3_uncontrolled;
          T4_uncontrolled;
          T5_uncontrolled]

T_vector_mean=[mean(T_matrix(:,1)),mean(T_matrix(:,2)),mean(T_matrix(:,3))]
T_vector_std=[std(T_matrix(:,1)),std(T_matrix(:,2)),std(T_matrix(:,3))]

RPY_matrix=[RPY1_uncontrolled;
            RPY2_uncontrolled;
            RPY3_uncontrolled;
            RPY4_uncontrolled;
            RPY5_uncontrolled]
Deg_matrix=rad2deg(RPY_matrix)
Q_matrix=quaternion(RPY_matrix,'euler','ZYX','frame')

quatAverage = meanrot(Q_matrix)
eulerAverage = euler(quatAverage,'ZYX','frame')
eulerAveraged = eulerd(quatAverage,'ZYX','frame')
angularDistancerad_matrix = 2*acos(abs(parts(quatAverage*conj(Q_matrix)))) % in radiants
angularDistancedeg_matrix = rad2deg(angularDistancerad_matrix)
std_rad=std(angularDistancerad_matrix)
std_deg=rad2deg(std_rad)









