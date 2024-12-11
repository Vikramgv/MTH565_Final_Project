%% Vikram Vijayakumar (02068559)
%% MTH 565 Project 2_1a
%% Regular lattice, n = 100, p = 0.3
%% https://github.com/ivanbrugere/matlab-networks-toolbox.
%% I had reviewed the above github link to obtain examples for clustering coefficients

v = 100;  %Number of vertices
p = 0.3;  %edge probability

%for i = 1:3
    A = rand(v) < p;  %Generate random adjacency matrix
    A = triu(A, 1);   %Ensures that the matrix is symmetric by rating the upper traingular part
    A = A + A';       %Symmetric adjacency matrix
    
    G = graph(A);     %Creating a graph from the adjacency matrix
    figure;
    plot(G, 'Layout', 'force');
    title(['Random Graph ', num2str(i), ' with n=', num2str(v), ' and p=', num2str(p)]);

    %Calculate density of the graph
    num_edges = numedges(G);  %Number of edges in the graph
    density = 2 * num_edges / (v * (v - 1));
    disp(['Density of Graph ', num2str(i), ': ', num2str(density)]);

    %Calculate mean clustering coefficient
    A = adjacency(G);  % Adjacency matrix of your graph
    local_clustering = local_clustering_coeff(A);  % Compute local clustering coefficient
    mean_clustering = mean(local_clustering);  % Mean clustering coefficient
    disp(['Mean Clustering Coefficient: ', num2str(mean_clustering)]);

    %Calculate global clustering coefficient
    global_clustering = global_clustering_coeff(A);  % Compute global clustering coefficient
    disp(['Global Clustering Coefficient: ', num2str(global_clustering)]);

    %Calculate the diameter of the graph
    D = distances(G); %path between two vertices
    diameter = max(D(:)); %longest path between two vertices
    disp(['Diameter = ', num2str(diameter)]);
%end

%Mean and global clustering fucntions
function global_C = global_clustering_coeff(A)
    triangles = trace(A^3) / 6;  % Number of triangles in the graph
    degree_sum = sum(sum(A));  % Sum of degrees of all vertices
    triples = 0.5 * sum(sum(A * A));  % Total number of connected triplets
    global_C = (6 * triangles) / triples;  % Global clustering coefficient (transitivity)
end

function C = local_clustering_coeff(A)
    n = size(A, 1);  %Number of vertices
    C = zeros(n, 1);  %Initialize local clustering coefficients
    for i = 1:n
        neighbors = find(A(i, :));  %Find the neighbors of a vertex i
        k_i = length(neighbors);  %Degree of a vertex i
        if k_i >= 2
            subgraph = A(neighbors, neighbors);  %Subgraph of neighbors of v
            num_triangles = sum(subgraph(:)) / 2;  % Number of triangles involving vertex i
            C(i) = (2 * num_triangles) / (k_i * (k_i - 1));  % Local clustering coefficient
        end
    end
end