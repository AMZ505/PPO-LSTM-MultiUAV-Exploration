function [final, fScore] = a_star(map, costs, start, goal)

    

        
    if ~islogical(map)
        error('MAP must be logical')
    end
    if ~isa(costs, 'double')
        error('COSTS must be double')
    end
    
    % Avoid div by zero
    costs(costs == 0) = eps;
    % Normalize such that smallest cost is 1.
    costs = costs / min(costs(:));

    % default return - empty for failure case
    final = [];
    
    mapSize = size(map);
    mapNumEl = numel(mapSize);

    % Initialize the open set, with START
    openSet = false(mapSize);
    openSet(start) = true;

    % Initialize closed set. Closed set consists of visited locations on 
    %  the map
    closedSet = false(mapSize);
    
    cameFrom = zeros(1, mapNumEl); 

    gScore = inf(mapSize);
    gScore(start) = 0;

    % Linear index -> row, col subscripts for the goal
    [gr, gc] = ind2sub(mapSize, goal);
    
    fScore = inf(mapSize);
    fScore(start) = compute_cost(mapSize, start, gr, gc);
     
    S2 = sqrt(2);

    % While the open set is not empty
    while any(openSet(:) > 0)


        % Find the minimum fScore within the open set
        [~, current] = min(fScore(:));

        % If we've reached the goal
        if current == goal
            % Get the full path and return it
            final = get_path(cameFrom, current);
            return
        end

        % Linear index -> row, col subscripts
        rc = rem(current - 1, mapSize(1)) + 1;
        cc = (current - rc) / mapSize(1) + 1;        
        
        % Remove CURRENT from openSet
        openSet(rc, cc) = false;
        % Place CURRENT in closedSet
        closedSet(rc, cc) = true;
        
        fScore(rc, cc) = inf;
        gScoreCurrent = gScore(rc, cc) + costs(rc, cc);
        
        % Get all neighbors of CURRENT. Neighbors are adjacent indices on 
        %   the map, including diagonals.
        % Col 1 = Row, Col 2 = Col, Col 3 = Distance to the neighbor
        n_ss = [ ...
            rc + 1, cc + 1, S2 ; ...
            rc + 1, cc + 0, 1 ; ...
            rc + 1, cc - 1, S2 ; ...
            rc + 0, cc - 1, 1 ; ...
            rc - 1, cc - 1, S2 ; ...
            rc - 1, cc - 0, 1 ; ... 
            rc - 1, cc + 1, S2 ; ...
            rc - 0, cc + 1, 1 ; ...
        ];

        % keep valid indices only
        valid_row = n_ss(:,1) >= 1 & n_ss(:,1) <= mapSize(1);
        valid_col = n_ss(:,2) >= 1 & n_ss(:,2) <= mapSize(2);
        n_ss = n_ss(valid_row & valid_col, :);
        % subscripts -> linear indices
        neighbors = n_ss(:,1) + (n_ss(:,2) - 1) .* mapSize(1);
        % only keep neighbors in the map and not in the closed set
        ixInMap = map(neighbors) & ~closedSet(neighbors);
        neighbors = neighbors(ixInMap);
        % distance to each kept neighbor
        dists = n_ss(ixInMap, 3);

        % Add each neighbor to the open set
        openSet(neighbors) = true;

        % TENTATIVE_GSCORE is the score from START to NEIGHBOR.
        tentative_gscores = gScoreCurrent + costs(neighbors) .* dists;
        
        % IXBETTER indicates where a better path was found
        ixBetter = tentative_gscores < gScore(neighbors);
        bestNeighbors = neighbors(ixBetter);        

        % For the better paths, update scores
        cameFrom(bestNeighbors) = current;
        gScore(bestNeighbors) = tentative_gscores(ixBetter);
        fScore(bestNeighbors) = gScore(bestNeighbors) + compute_cost(mapSize, bestNeighbors, gr, gc);


    end % while
        if all(openSet(:) ==0)
              hh=98;
        end
    hh=98;
end

function p = get_path(cameFrom, current)
    % Returns the path. This function is only called once and therefore
    %   does not need to be extraordinarily efficient
    inds = find(cameFrom);
    p = nan(1, length(inds));
    p(1) = current;
    next = 1;
    while any(current == inds)
        current = cameFrom(current);
        next = next + 1;
        p(next) = current; 
    end
    p(isnan(p)) = [];
end

function cost = compute_cost(mapSize, from, rTo, cTo)
    % Returns COST, an estimated cost to travel the map, starting FROM and
    %   ending at TO.
    [rFrom,cFrom] = ind2sub(mapSize, from);
    cost = sqrt((rFrom - rTo).^2 + (cFrom - cTo).^2);
end
