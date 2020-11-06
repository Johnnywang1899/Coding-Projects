% set(0,'DefaultFigureWindowStyle','docked') 

%%%% Instructions:
% pts: Information of particles [x y z groupNum]
% GROUPMUN: group number of particles
% groupCount: total number of groups generated during calculation
% Total_group_number: total number of groups after caculation
% Centres: number of gathering centres. Simulating nonuniformly distributed carbon particle seeds
% CentreRad: general radii of gathering centres
% This program does not guarantee that all particles locate within the boundry


rng;  % set reset random seed to always repeat on same set of points

clc
clear all;
close all;


%%%%%%%%%%%%%

% Automatically set values
Dimension = 10;            % 1 micrometer = 10 unit. Dimension for the cube
WeightPercentage = 5;    % Weight Percentage (%) of Carbon Black
Density_Carbon = 1.8;      % Density of carbon black. Unit: g/cm^3
Density_Mixture = 1.05;    % Density of the mixture. Unit: g/cm^3
DiameterCarbon = 56;       % Average diameter of carbon black. Unit: nm
DiameterDeviation = 4;     % Deviation of diameter of carbon black. Unit: nm
GauDis = 0.3;              % percentage of particles not well dispersed, but concentrated around the centers
Centres = 0;               % Number of Centres
CentreRad = 10;            % Radii of Centres?95% Particle locates in this range, standard deviation 5
Filename = '5_weight_carbon_particles.xlsx';  % Name of the spreadsheet where data will be saved


boundsX = [0, 10*Dimension];        % X axis limits,unit 10 um
boundsY = [0, 10*Dimension];        % Y axis limits
boundsZ = [0, 10*Dimension];        % Z axis limits

Mass_Mixture = Density_Mixture * (Dimension^3)*1000*(10^(-6))^3;
Mass_Carbon = WeightPercentage * 0.01 * Mass_Mixture;
Volume_Carbon = Mass_Carbon/(Density_Carbon*1000);
Volume_Sphere = (4/3)*((DiameterCarbon/2)*10^(-9))^3*pi;
Num_Carbon = round(Volume_Carbon/Volume_Sphere);
ParNum = Num_Carbon;
AvgParRad = DiameterCarbon*0.01;
ParRadDev = DiameterDeviation*0.01;
ProgramInfo = {'Dimension', Dimension;'WeightPercentage', WeightPercentage;'Density_Carbon', Density_Carbon;'Density_Mixture', Density_Mixture;'DiameterCarbon', DiameterCarbon;'DiameterDeviation', DiameterDeviation;'GauDis', GauDis;'Centres', Centres;'Centres', Centres;'CentreRad', CentreRad};


%%%%%%%%%%%%%
% Manually set values
%ParNum = 42292;          % Number of particles, 0.5vol%
%AvgParRad = 0.56;        % Radii of particles,56nm, 1 unit=0.1 um
%ParRadDev = 0.04;        % 95% Particle radius within this range, 4nm. standard deviation 0.02
%GauDis = 0.3;            % percentage of particles not well dispersed, but concentrated around the centers  
%Centres = 0;             % Number of Centres
%CentreRad = 10;          % Radii of Centres?95% Particle locates in this range, standard deviation 5

%boundsX = [0, 100];        % X axis limits,unit 10 um
%boundsY = [0, 100];        % Y axis limits
%boundsZ = [0, 100];        % Z axis limits


%%%%%%%%%%%%%   
Tolerance = 3 * AvgParRad;    % Tolerance for the first selection of particles when reducing calculation time

%% DO NOT MODIFY %%
%pts = [x y z groupNum]
X = 1;              %pts(X)=pts(1)
Y = 2;              %pts(Y)=pts(2)
Z = 3;              %pts(Z)=pts(3)
GROUPNUM = 4;       %pts(GROUPNUM)=pts(4)
ParRad = 5;         %pts(ParRad)=pts(5)


GRPNUM = 1;         %initial value


filename = ['xyz.xlsx'];

%DO NOT MODIFY 
%%%%%%%%%%%%%%%%%%%%%%%
%%
for iteration=1:1                              % Number of calculations.
    tic                                        % Start a stopwatch timer
    exceed = [0; 0; 0;];
    pts = zeros(ParNum, GROUPNUM);             % Initialize matrix, record the information of every particles
    
    if Centres~=0
    %%%%%%%
    %Assign locations of particles (Normally randomly distributed)
    Boundary = [boundsX(2) boundsY(2) boundsZ(2)];

    NormDistCentres = zeros(Centres,Z);
    NormDistCentres(:,X:Z)= rand([Centres Z]) .* repmat([(boundsX(2)-4*CentreRad) (boundsY(2)-4*CentreRad) (boundsZ(2)-4*CentreRad)],[Centres,1]) + 2*CentreRad;%change CentreRad position
    NumCentreGroup = round(ParNum*GauDis/Centres);
    
    for loop=1:Centres
        pts((loop-1)*NumCentreGroup+1:loop*NumCentreGroup,X:Z) = randn([NumCentreGroup Z]) .* (CentreRad/2) + repmat(NormDistCentres(loop,X:Z),[NumCentreGroup, 1]);%95% of the points lie within CentreRad, standard deviation is CentreRad/2, that is 68% 
    end
    

    %%%%%%%
    %Assign locations of particles (Uniformly randomly distributed)
    RestNum = ParNum - NumCentreGroup*Centres;
    pts(NumCentreGroup*Centres+1:ParNum ,X:Z) = rand([RestNum Z]) .* repmat([boundsX(2) boundsY(2) boundsZ(2)],[RestNum, 1]); 
    
    else
     pts(:,X:Z) = rand([ParNum Z]) .* repmat([boundsX(2) boundsY(2) boundsZ(2)],[ParNum, 1]);
    end
    
    %%%%%%%%% Assign particle radius to each particle
    
    pts(:,ParRad)= randn([ParNum 1]) .* (ParRadDev/2) + repmat(AvgParRad, [ParNum, 1]);
    
    
    %%%%%%%%% Track limits of groups
    groupCount = 0;
    groupLimits = zeros(ParNum, 1); 

    %%
    tic

    
    for i=1:ParNum
        solution = 0;
        Count = 0;
        ParConnect = [];

        inRange = find(...
            and(pts(1:i,X) < pts(i,X) + Tolerance, pts(1:i,X) > pts(i,X) - Tolerance ) & ...
            and(pts(1:i,Y) < pts(i,Y) + Tolerance, pts(1:i,Y) > pts(i,Y) - Tolerance ) & ...
            and(pts(1:i,Z) < pts(i,Z) + Tolerance, pts(1:i,Z) > pts(i,Z) - Tolerance ));

        if inRange == i             % tested point lies outside all groups, add another group 
            groupCount = groupCount + 1;
            pts(i, GROUPNUM) = groupCount;

            groupLimits(groupCount,GRPNUM) = groupCount;


        else                        % possible points found within range, calculate distance between particles using DistBetween2Particles function
            inRange(inRange == i) = [];                             % remove the current point (the particle being compared - i) from inRange
            
            p1 = repmat([pts(i,X) pts(i,Y) pts(i,Z) pts(i, ParRad)],1,1);          % The coordinate of the particle being compared
            p2 = pts(inRange,[X, Y, Z, ParRad]);                            % The coordinate of other particles in range
 
            
            distance = DistBetween2Particles(p1,p2);                % Calculating the distance using DistBetween2Particles function

            %ParConnect = inRange(distance < (2*AvgParRad)); 
            for z = 1:size(inRange)
                if distance(z,1) < p1(1, 4) + p2(z, 4)
                    Count = Count + 1;
                    ParConnect(Count,1) = inRange(z);
                end
            end
            
            % When the distance between two particles is smaller than the diameter of a particle, the pair is deemed as connnected
           
            
            [m n] = size(ParConnect);                               % If distance > 2*particle's radius, ParConnect is [], m=0. 
            
       
            
            if m == 0               % If the particle is not connected to any other particles, it is assigned to a new group. The same step as in the first selection
                groupCount = groupCount + 1;
                pts(i, GROUPNUM) = groupCount;

                groupLimits(groupCount,GRPNUM) = groupCount;

            
            
            else                    % If the particle is connected to a particle, add particles to the corresponding group.
                for j = 1:m
                    if j == 1                                       % Add the first particle in the ParConnect to its corresponding group
                        groupNumber = pts(ParConnect(1), GROUPNUM);
                        pts(i, GROUPNUM) = groupNumber;



                    else            % Bridge groups. Merging the groups. Add the rest of particles that are connected to group that includes the first particle, therefore merging the groups.
                        bridgeGroup = pts(ParConnect(j), GROUPNUM);

                        pts(pts(:,GROUPNUM) == bridgeGroup,GROUPNUM) = pts(ParConnect(1), GROUPNUM);

                        if (bridgeGroup ~= pts(ParConnect(1), GROUPNUM))
                            groupLimits(bridgeGroup,:) = 0;          % Erase old group in groupLimits
                        end
                    end
                end
            end        
        end

        %condition for formation of percolation network 
%        if exceed(X) == 0
%            exceed(X) = ~isempty(find((groupLimits(1:groupCount,GRPX1) > boundsX(2) & groupLimits(1:groupCount,GRPX2) < boundsX(1) ), 1));
%        end
%        if exceed(Y) == 0
%            exceed(Y) = ~isempty(find((groupLimits(1:groupCount,GRPY1) > boundsY(2) & groupLimits(1:groupCount,GRPY2) < boundsY(1) ), 1));
%        end
%        if exceed(Z)== 0
%            exceed(Z) = ~isempty(find((groupLimits(1:groupCount,GRPZ1) > boundsZ(2) & groupLimits(1:groupCount,GRPZ2) < boundsZ(1) ), 1));
%        end
%
%        if exceed(X) > 0
%            solution = X;
%        elseif exceed(Y) > 0
%            solution = Y;
%        elseif exceed(Z) > 0
%            solution = Z;
%        end
%
%        if solution
%            disp('*************');
%            display([char(87 + solution) ' ' num2str(i)]);
%            xyzRow = [char(64 + solution),num2str(iteration)];
%            xlswrite(filename,i, 'Sheet1',xyzRow);
%            
%            exceed(solution) = -1;
%            solution = 0;
%        end
%
%        if sum(exceed) == -3
%            break;
%        end

    end

    % toc

nonzeros = find(groupLimits(:,1)~=0);
Total_group_number = size(nonzeros);           %Find the total number of groups


%%     3D draw
 
     tic
     figure
     hold on
 
     
     [m n] = size(pts);
     groups = unique(pts(:,GROUPNUM));
     
     cMap = colormap(hsv);
     
     if groups(1) == 0
        groups(1) = []; 
     end
     
     for j = 1:size(groups)      % display groups with same color
         
         if pts(i,GROUPNUM) ~= groups(j)
        groupMembers = pts(:,GROUPNUM) == groups(j);
        %scatter3(pts(:, 1), pts(:, 2), pts(:, 3),0.5,cMap(mod(j-1,64)+1,:));%too slow
        plot3(pts(groupMembers, 1), pts(groupMembers, 2), pts(groupMembers, 3),'o', 'MarkerFaceColor', cMap(mod(j-1,64)+1,:) ,'MarkerEdgeColor','none','MarkerSize', 2 );    
     
         end
     end
     
     %plot3(pts(:, 1), pts(:, 2), pts(:, 3), '.'); %Plot the particles as dots in 3D
     
     if solution 
         groupMembers = pts(:,GROUPNUM) == pts(i,GROUPNUM);
         plot3(pts(groupMembers, 1), pts(groupMembers, 2), pts(groupMembers, 3) ,'o', 'MarkerFaceColor', 'k', 'MarkerEdgeColor','none','MarkerSize', 1 );
        %scatter3(pts(:, 1), pts(:, 2), pts(:, 3), 0.5, 'k');%too slow 
     end
     
     rectangle('Position',[boundsX(1),boundsY(1),boundsX(2) - boundsX(1) ,boundsY(2) - boundsY(1)]);
 
 
 plot3( [0 0 100 100 0], [0 100 100 0 0], [0 0 0 0 0], 'Color', 'k', 'LineWidth', 1.5 );
 plot3( [0 0 100 100 0], [0 100 100 0 0], [100 100 100 100 100], 'Color', 'k', 'LineWidth', 1.5 );
 
 plot3( [0 0 100 100 0], [0 0 0 0 0], [0 100 100 0 0], 'Color', 'k', 'LineWidth', 1.5 );
 plot3( [0 0 100 100 0], [100 100 100 100 100], [0 100 100 0 0], 'Color', 'k', 'LineWidth', 1.5 );
 
 plot3( [0 0 0 0 0], [0 0 100 100 0], [0 100 100 0 0], 'Color', 'k', 'LineWidth', 1.5 );
 plot3( [100 100 100 100 100], [0 0 100 100 0], [0 100 100 0 0], 'Color', 'k', 'LineWidth', 1.5 );
 
     xlim([boundsX(1) , boundsX(2) ])
     ylim([boundsY(1) , boundsY(2) ])
     zlim([boundsZ(1) , boundsZ(2) ])
     axis equal
     toc
 xlabel('X')
 ylabel('Y')
 zlabel('Z')
  
 
 %% draw the size for each group
 cloudStore = [];
 for f = 1:size(groups)
     
     cloudMems = sum(pts(:,GROUPNUM) == groups(f)) ;
     if cloudMems > 100 %if cloudMems is large than 10, draw the area

         cloud = pts(pts(:,GROUPNUM) == groups(f),:);
         cloudCentroid = (mean([cloud(:,1),cloud(:,2 ), cloud(:,3)]));
         cloudRad = (max(cloud(:,X:Z)) - min(cloud(:,X:Z)))/2;
         [ellipsX, ellipsY, ellipsZ] = ellipsoid(cloudCentroid(1),cloudCentroid(2), cloudCentroid(3), cloudRad(1), cloudRad(2),cloudRad(3));

         cloudStore = [cloudStore; [groups(f) cloudMems cloudCentroid cloudRad]];
         surf(ellipsX, ellipsY, ellipsZ);
     end 
 end
 
    
    %% 2D draw  
    %% draw only large groups
    % 
    % figure
    % hold on

    % xlim([boundsX(1) - 2*fibreLen, boundsX(2) + 2*fibreLen ])
    % ylim([boundsY(1) - 2*fibreLen, boundsY(2) + 2*fibreLen ])
    % axis equal
    % [m n] = size(pts);
    % groups = unique(pts(:,GROUPNUM));
    % 
    % cMap = colormap(lines);
    % % scatter([pts(:,X) ], [pts(:,Y)], 'Marker', 'o', 'MarkerEdgeColor', cMap(1,:));
    % 
    % if groups(1) == 0
    %    groups(1) = []; 
    % end
    % 
    % colourCount = 0;
    % for i = 1:size(groups)      % display groups with same color
    %    groupMembers = pts(:,GROUPNUM) == groups(i);
    %    if sum(groupMembers) > 100
    %        colourCount = colourCount +1;
    %        line([pts(groupMembers,X1) pts(groupMembers,X2)]', [pts(groupMembers,Y1), pts(groupMembers,Y2)]', 'Marker', 'none', 'Color', cMap(mod(colourCount-1,64)+1,:));
    %    end
    % %    pause(1)
    % end
    % 
    % rectangle('Position',[boundsX(1),boundsY(1),boundsX(2) - boundsX(1) ,boundsY(2) - boundsY(1)]);

    %% double check the answer
    % tic
    
    
% 
%     groups = unique(pts(:,GROUPNUM));
%     if groups(1) == 0
%        groups(1) = []; 
%     end
%     lims = zeros(size(groups,1),4);
%     for i = 1:size(groups)      % display groups with same color
%        groupMembers = pts(:,GROUPNUM) == groups(i);
%        lims(i,1) = max(max(pts(groupMembers, [X1 X2])));
%        lims(i,2) = min(min(pts(groupMembers, [X1 X2])));
%        lims(i,3) = max(max(pts(groupMembers, [Y1 Y2])));
%        lims(i,4) = min(min(pts(groupMembers, [Y1 Y2])));
% 
%        if lims(i,1) > 100 && lims(i,2) < 0
%            display(['X ' num2str(i) ' ' num2str(groups(i)) ' ' num2str(lims(i,1)) ' ' num2str(lims(i,2))])
% 
%            numRow = ['A',num2str(iteration)];
%            xyRow = ['B',num2str(iteration)];
%            xlswrite(filename,fibre,'Sheet1'numRow);
%            xlswrite(filename,'X', 'Sheet1', xyRow);
%        elseif lims(i,3) > 100 && lims(i,4) < 0
%            display(['Y ' num2str(i) ' ' num2str(groups(i)) ' ' num2str(lims(i,3)) ' ' num2str(lims(i,4))])
% 
%            numRow = ['A',num2str(iteration)];
%            xyRow = ['B',num2str(iteration)];
%            xlswrite(filename,fibre,'Sheet1',numRow);
%            xlswrite(filename,'Y','Sheet1',xyRow);
%        end
%     end

  toc
end

%%%%% Statistics
figure
 GroupChart = [];
 Membs = zeros(size(groups,1),2);
 
 for f = 1:size(groups)
     
     Membs(f,1) = f;
     Membs(f, 2) = sum(pts(:,GROUPNUM) == groups(f)) ;
     
 end
 bar (Membs(:,1),Membs(:,2));

 xlswrite(Filename, Membs);
 xlswrite(Filename, pts,2);
 xlswrite(Filename, ProgramInfo,3);

 

 