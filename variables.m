% --- Variables pour les Actionneurs (Prismatic Joints) ---
%q0 = [0 0 0];   % Position initiale des 3 glissières (en mètres)
%q0 = [0.1, 0.1, 0.1]; % Décalage de 10cm pour chaque glissière
q0 = [0.4447, 0.4447, 0.4447]; % celle de 0 0 -0.35
qd0 = [0 0 0];  % Vitesse initiale des 3 glissières (en m/s)

% --- Variables pour l'Effecteur Final (End Effector) ---
%eeL = 0.05;     % Longueur du cylindre de l'effecteur (CylinderLength)
%eeR = 0.02;     % Rayon du cylindre de l'effecteur (CylinderRadius)
eeL = 0.03;
eeR = 0.02;
% --- Variables de structure (Base et Plateau) ---
based_plate_thick = 0.02; 
bottom_plate_thick = 0.01;

% ---
%L2 = 0.35; %%%%%%%%%% 9siiiira %%%
L2 = 0.6;

%L3 = 0.25; % Longueur pour Upper Link torbit bin zouz bas parallel 
L3 = 0.09;
R1 = 0.02; % Rayon pour Joint Cover   
R2 = 0.015; % Rayon pour Upper Link    

%Rtop = 0.45; % non plus --> 0.364
%Rbot = 0.04; % non plus --> 0.036
Rtop = 0.364; 
Rbot = 0.036;
%b=0;
b = pi/6;
%offset_z = 0.1; % Exemple de décalage si utilisé
