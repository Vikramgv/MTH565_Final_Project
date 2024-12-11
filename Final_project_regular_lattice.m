%% Vikram Vijayakumar (02068559)
%% MTH 565 Final Project
%% Regular lattice, n = 100, k= 30
%% https://github.com/ivanbrugere/matlab-networks-toolbox.
%%I had reviewed the above github link to obtain examples for clustering coefficients

n = 100; %Number of vertices, change n = 20 or 100 based on the problem
k = 30; 
% for k = 2:2:10 % Loop over different values of k
%     if k == 8 %avoids 8 since it is not asked in the problem
%         continue
%     end

    %Create the adjacency matrix for a circular lattice to represent the 
    %connections between edges and vertices and help with
    %calculating clustering coefficients

    A = zeros(n); %resets for every value of k
    for i = 1:n
        for j = 1:k/2 %to prevent double edges between same vertices
            %Connect vertex i to i+j and i-j 
            A(i, mod(i+j-1, n)+1) = 1; %Right neighbor
            A(i, mod(i-j-1, n)+1) = 1; %Left neighbor
        end
    end
    
%plot the graph
G = graph(A); 
figure;
plot(G, 'Layout', 'circle');
title(['Lattice with n=', num2str(n), ', k=', num2str(k)]);

disp(' ')
disp(['For k = ', num2str(k)])

%Calculate the diameter of the graph
D = distances(G); %path between two vertices
diameter = max(D(:)); %longest path between two vertices
disp(['Diameter = ', num2str(diameter)]);

%Calculate the density
e = numedges(G); %number of edges
density = 2 * e / (n * (n-1));
disp(['Number of Edges: ', num2str(e)]);
disp(['Density: ', num2str(density)]);

%Calculate local clustering coefficient for each vertex
local_clustering = zeros(n, 1);
for v = 1:n
    neighbors = find(A(v, :)); %Find neighbors of vertex v
    neighbors_v = length(neighbors);  %number of neighbors
    subgraph = A(neighbors, neighbors); %Subgraph induced by neighbors of v
    e_v = sum(subgraph(:)) / 2; %Count edges and divide it by 2 to avoid double-counting
    local_clustering(v) = (2 * e_v) / (neighbors_v * (neighbors_v - 1));
end
disp(['Number of edges between neighbors: ', num2str(e_v)]);
disp(['Number of neighbors: ', num2str(neighbors_v)]);
disp(['Local Clustering Coefficient of each vertex is: ', num2str(local_clustering(v))]);

% Calculate mean clustering coefficient
mean_clustering = mean(local_clustering);
disp(['Mean Clustering Coefficient: ', num2str(mean_clustering)]);

% Calculate global clustering coefficient 
%Trace function is used to calculate diagonal elements in matrix. 
triangles = trace(A*A*A) / 6; % Each triangle is counted 6 times in the cube of the adjacency matrix
triples = sum(sum(A*A)) - trace(A*A); % Total number of connected triples
global_clustering = 3 * triangles / triples;
disp(['Number of triangles: ', num2str(triangles)]);
disp(['Number of connected triples: ', num2str(triples)]);
disp(['Global Clustering Coefficient: ', num2str(global_clustering)]);  
