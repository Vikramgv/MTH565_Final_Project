%% MTH 565 - Final Project
%% Random graph for studying cascading failures in Internet of Things
%% G(n, p), range = 30
%% Vikram Vijayakumar
%% References to Code:
%% 1. Hossein (2023). Wireless Sensor Networks (WSNs) (https://www.mathworks.com/matlabcentral/fileexchange/51262-wireless-sensor-networks-wsns), MATLAB Central File Exchange. Retrieved December 1, 2023.
%% 2. Abhishek Gupta (2023). WSN-Nodes-Location-Optimiation (https://github.com/earthat/WSN-Nodes-Location-Optimiation), GitHub. Retrieved December 1, 2023.
%% 3. cesar silva (2023). matlab wsn code (https://github.com/cesarfgs/matlab-wsn-code), GitHub. Retrieved December 1, 2023.

clear all
close all
clc

%Main 
sensor.nodes = 100; %Number of nodes
sensor.nodePosition(1,:) = [1 0 0]; %Control unit node (Base satation) position
sensor.nodePosition(2,:) = [2 100 100]; %Farthest node position
sensor.range = 30; %Transmission range of each node
p = 0.3; 

%Energy factors: Important for sensor node performance 
sensor.minEnergy = 80;
sensor.maxEnergy = 100; 
sensor.energyconsumptionperCicle = 0.8;
sensor.energyrecoveryperCicle = 0.2;
packets=0;
iteration=1;

%Sensor node position sorting
for a = 3 : sensor.nodes
   sensor.nodeId = a; 
   route.x = randi([1 100]); %Position of sensor with respect to x-axis
   route.y = randi([1 100]); %Position of sensor with respect to y-axis
   sensor.nodePosition(a,:) = [sensor.nodeId route.x route.y]; %Sensor Node ID
end

% Euclidean Distance calculation from one node to all others
for i = 1 : sensor.nodes
    for j = 1: sensor.nodes
        route.x1 = sensor.nodePosition(i,2); 
        route.x2 = sensor.nodePosition(j,2); 
        route.y1 = sensor.nodePosition(i,3); 
        route.y2 = sensor.nodePosition(j,3);
        sensor.euclidean(i,j) = sqrt(  (route.x1 - route.x2) ^2 + (route.y1 - route.y2)^2  );         
    end
end

%Sensor node position according to their range(euclidean distance)
% sensor.weights = lt(sensor.euclidean,sensor.range);

% Initialize weights matrix
sensor.weights = zeros(sensor.nodes);

% Create edges only if within range and with probability p
for i = 1:sensor.nodes
    for j = i+1:sensor.nodes % Avoid self-loops and duplicate edges
        if sensor.euclidean(i, j) <= sensor.range % Check range condition
            if rand < p % Check connection probability
                sensor.weights(i, j) = 1;
                sensor.weights(j, i) = 1; % Ensure symmetry for undirected graph
            end
        end
    end
end



G=graph(sensor.weights,'omitselfloops'); %Avoids self looping
for a = 1 : height(G.Edges)
    route.x = G.Edges.EndNodes(a,1);
    route.y = G.Edges.EndNodes(a,2);
    route.Z(a,:) = sensor.euclidean(route.x,route.y);
 end
G.Edges.Euclidean = route.Z(:,1);

%Energy sorting:
[sensor.nodePosition(:,4)] = sensor.maxEnergy -(sensor.maxEnergy-sensor.minEnergy)*rand(sensor.nodes,1);
sensor.nodePosition(1:2,4)=1000;
for a = 1: length(sensor.nodePosition(:,1))
    sensor.nodePosition(a,5) = degree(G,sensor.nodePosition(a,1));    
end

%Position and Energy coordination for even distribuition
for a = 1 : height(G.Edges)
	route.Sourcenode = G.Edges.EndNodes(a,1);
	route.Targetnode = G.Edges.EndNodes(a,2);
	G.Edges.SourcenodeXpos(a) = sensor.nodePosition(route.Sourcenode,2);
	G.Edges.SourcenodeYpos(a) = sensor.nodePosition(route.Sourcenode,3);
	G.Edges.TargetnodeXpos(a) = sensor.nodePosition(route.Targetnode,2);
	G.Edges.TargetnodeYpos(a) = sensor.nodePosition(route.Targetnode,3);
    G.Edges.ActiveEdges(a) = 1;
end

% Initial metrics calculations
initialDiameter = calculateDiameter(G);
disp(['Initial Diameter: ', num2str(initialDiameter)]);

initialDensity = calculateDensity(G);
disp(['Initial Density: ', num2str(initialDensity)]);

initialClustering = calculateClusteringCoefficient(G);
disp(['Initial Clustering Coefficient: ', num2str(initialClustering)]);

%Plot
figure('units','normalized','innerposition',[0 0 1 1],'MenuBar','none')
subplot(1,1,1);
route.Xmax = 1500;
route.Xmin = 0;
route.Ymax = 1500;
route.Ymin = 0;
p = plot(G,'XData',(sensor.nodePosition(:,2)),'YData',(sensor.nodePosition(:,3))); 
line(sensor.nodePosition(1:2,2),sensor.nodePosition(1:2,3),'color','black','marker','o','linestyle','none','markersize',50)
route.ax = gca;
route.ax.XAxis.TickValues = 0:10:100;
route.ax.YAxis.TickValues = 0:10:100;
grid on
hold on
title(['IOT System (WSN) | ','Number of Sensor Nodes: ',num2str(sensor.nodes),' | Range of each Sensor Node: ', num2str(sensor.range)])
pause(2)

route.deadNodeList=[]; %No dead nodes initially
G2 = shortestpathtree(G,1,2); %Hop takes the shortest path using edges
while ~isempty(G2.Edges)
    G2 = shortestpathtree(G,1,2);
    iteration = iteration+1;
    if isempty(G2.Edges) %Terminate when there is no connection between node'1' and node'2'
        break
    end
    
    %Nodes involved in routing using the shortest path(edges)
    route.nodes = unique(G2.Edges.EndNodes);
    route.routepath = shortestpath(G,1,2);
            
    %Construct localization dataset to nodes involved in routing event for plot effects
    for a = 1 : length(route.nodes)
        route.b=route.nodes(a,1);
        sensor.nodesPosition(a,:)=sensor.nodePosition(route.b,:);
    end
        
    %Energy usage (decrease) and packet transmit count
   while min(sensor.nodePosition(:,4))>0 
       for a = 1 : length(route.nodes)
           node=route.nodes(a,1);
            sensor.nodePosition(node,4)=sensor.nodePosition(node,4)-sensor.energyconsumptionperCicle^rand()+sensor.energyrecoveryperCicle^rand();
           packets=packets+1;       
       end        
    end
    
    %Find the dead node
    [route.deadNodes] = find(sensor.nodePosition(:,4)<=0);
    for a = 1 : length(route.deadNodes)
        deadnode=route.deadNodes(a,1);
        for b = 1 : height(G.Edges)
            if ismember(G.Edges.EndNodes(b,1),deadnode) == 1 || ismember(G.Edges.EndNodes(b,2),deadnode) == 1
                G.Edges.ActiveEdges(b)=0;
                deadnode;
                pause(2)
            end
        end
    end
    route.deadNodeList(length(route.deadNodeList)+1,1)=deadnode;
    [route.deadEdge]=find(G.Edges.ActiveEdges==0);

    %Set dead node energy as null (indicate node failure)
    [route.deadNodeEnergy]=find(sensor.nodePosition(:,4)<=0);
    for a = 1 : length(route.deadNodeEnergy)
       b=route.deadNodeEnergy(a,1);
       sensor.nodePosition(b,4)=NaN;
    end
    
    %Get number of hops between base station and farthest node for a particular routing session
    NoOfHops=length(route.nodes);
   
    %Contents display in plot and command window
    msg=['Routing Cycle: ',num2str(iteration-1),' | Number of Hops: ',num2str(NoOfHops),' | Number of Packets Transmitted: ', num2str(packets),' | Next Dead Node: ', num2str(deadnode),' | Routing Direction(Nodes used for hop): ', num2str(route.routepath)];
    disp(msg)
    
    % Metrics after each iteration
    currentDiameter = calculateDiameter(G);
    disp(['Iteration ', num2str(iteration), ' - Diameter: ', num2str(currentDiameter)]);
    
    currentDensity = calculateDensity(G);
    disp(['Iteration ', num2str(iteration), ' - Density: ', num2str(currentDensity)]);
    
    currentClustering = calculateClusteringCoefficient(G);
    disp(['Iteration ', num2str(iteration), ' - Clustering Coefficient: ', num2str(currentClustering)]);

    %plot after every dead node iteration's result
    figure('units','normalized','innerposition',[0 0 1 1],'MenuBar','none')
    p = plot(G,'XData',(sensor.nodePosition(:,2)),'YData',(sensor.nodePosition(:,3))); 
    line(sensor.nodePosition(1:2,2),sensor.nodePosition(1:2,3),'color','black','marker','o','linestyle','none','markersize',50)
    route.ax = gca;
    route.ax.XAxis.TickValues = 0:10:100;
    route.ax.YAxis.TickValues = 0:10:100;
    hold on
    
    %Plot all dead nodes in red
    for a = 1 : length(route.deadNodeList)
        route.b=route.deadNodeList(a,1);
        scatter(sensor.nodePosition(route.b,2),sensor.nodePosition(route.b,3),'MarkerFaceColor','red');
    end
    
    %Display after every iteration's result
    title(['Routing Cycle: ',num2str(iteration-1),' | Number of Hops: ',num2str(NoOfHops),' | Number of Packets Transmitted: ', num2str(packets),' | Next Dead Node: ', num2str(deadnode),' | Routing Direction(Nodes used for hop): ', num2str(route.routepath)]);
    grid on
    pause(2)
   
    %Remove dead edges from graph
    G = rmedge(G,route.deadEdge(:,1));

    %Mark nodes used for routing with green color
    scatter(sensor.nodesPosition(:,2),sensor.nodesPosition(:,3),'MarkerFaceColor','green');
    pause(0.2)
    clear dataset.nodesPosition %Clear router nodes position to avoid continuous plot
end

%Mark all dead nodes in red
    for a = 1 : length(route.deadNodeList)
        route.b=route.deadNodeList(a,1);
        scatter(sensor.nodePosition(route.b,2),sensor.nodePosition(route.b,3),'MarkerFaceColor','red');
    end

    % Final graph display
figure('units', 'normalized', 'innerposition', [0 0 1 1], 'MenuBar', 'none');
finalPlot = plot(G, 'XData', sensor.nodePosition(:, 2), 'YData', sensor.nodePosition(:, 3));

% Highlight dead nodes in red
hold on;
for i = 1:length(route.deadNodeList)
    deadNode = route.deadNodeList(i);
    scatter(sensor.nodePosition(deadNode, 2), sensor.nodePosition(deadNode, 3), 80, 'MarkerFaceColor', 'red', 'DisplayName', 'Dead Nodes');
end

    % % Highlight base station and farthest node
    % scatter(sensor.nodePosition(1, 2), sensor.nodePosition(1, 3), 150, 'black', 'filled', 'DisplayName', 'Base Station');
    % scatter(sensor.nodePosition(2, 2), sensor.nodePosition(2, 3), 150, 'black', 'filled', 'DisplayName', 'Farthest Node');
    % 
    % % Title and legend for final graph
    % title(['Final Graph After Node Failures: Remaining Nodes = ', num2str(numnodes(G)), ...
    %        ', Remaining Edges = ', num2str(numedges(G))]);
    %  legend show;
    % grid on;

%Title for plot after there is no path for routing
title(['Routing Cycle: ',num2str(iteration-1),' | Number of Hops: ',num2str(NoOfHops),' | Number of Packets Transmitted: ', num2str(packets),' | Last Dead Node: ', num2str(deadnode),' | Routing Direction(Nodes used for hop): ', num2str(route.routepath),'{\color{red} - NO ROUTES AVAILABLE (SYSTEM FAILURE)}'])
disp('NO ROUTES BETWEEN CONTROL UNIT (Node1) AND FARTHEST NODE (Node2)')

% Final metrics calculations
finalDiameter = calculateDiameter(G);
disp(['Final Diameter: ', num2str(finalDiameter)]);

finalDensity = calculateDensity(G);
disp(['Final Density: ', num2str(finalDensity)]);

finalClustering = calculateClusteringCoefficient(G);
disp(['Final Clustering Coefficient: ', num2str(finalClustering)]);


function diameter = calculateDiameter(graph)
    % Function to calculate the diameter of the graph
    if isempty(graph.Edges)
        diameter = Inf; % No connection between nodes
    else
        distancesMatrix = distances(graph);
        distancesMatrix(isinf(distancesMatrix)) = NaN; % Ignore disconnected nodes
        diameter = max(distancesMatrix(:), [], 'omitnan'); % Longest shortest path
    end
end

function density = calculateDensity(graph)
    % Function to calculate the density of the graph
    numNodes = numnodes(graph);
    numEdges = numedges(graph);
    if numNodes > 1
        density = (2 * numEdges) / (numNodes * (numNodes - 1));
    else
        density = 0; % No edges possible with single node
    end
end

function avgClustering = calculateClusteringCoefficient(graph)
    % Function to calculate the clustering coefficient of the graph
    adjMatrix = adjacency(graph); % Adjacency matrix
    triangles = trace(adjMatrix^3) / 6; % Number of triangles
    degrees = degree(graph); % Node degrees
    triplets = sum(degrees .* (degrees - 1)) / 2; % Total number of triplets
    if triplets > 0
        avgClustering = triangles / triplets; % Average clustering coefficient
    else
        avgClustering = 0; % Avoid division by zero
    end
end