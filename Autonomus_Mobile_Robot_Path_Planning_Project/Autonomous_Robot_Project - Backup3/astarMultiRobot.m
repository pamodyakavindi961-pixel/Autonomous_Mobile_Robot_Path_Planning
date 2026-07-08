function path = astarMultiRobot(map,start,goal,reserved,maxT)

[row,col,hgt] = size(map);

moves = [];
costs = [];
for dx = -1:1
    for dy = -1:1
        for dz = -1:1
            if dx==0 && dy==0 && dz==0
                continue;
            end
            moves = [moves; dx dy dz];
            costs = [costs; sqrt(dx^2+dy^2+dz^2)];
        end
    end
end
moves = [moves; 0 0 0];
costs = [costs; 1];

gScore = containers.Map('KeyType','char','ValueType','double');
fScore = containers.Map('KeyType','char','ValueType','double');
cameFrom = containers.Map('KeyType','char','ValueType','char');
stateOf = containers.Map('KeyType','char','ValueType','any');

startKey = sprintf('%d_%d_%d_%d',start(1),start(2),start(3),0);
h0 = abs(start(1)-goal(1)) + abs(start(2)-goal(2)) + abs(start(3)-goal(3));
gScore(startKey) = 0;
fScore(startKey) = h0;
stateOf(startKey) = [start 0];

openKeys = {startKey};

while ~isempty(openKeys)

    bestF = inf; bestIdx = 1;
    for k = 1:numel(openKeys)
        fk = fScore(openKeys{k});
        if fk < bestF
            bestF = fk;
            bestIdx = k;
        end
    end
    curKey = openKeys{bestIdx};
    openKeys(bestIdx) = [];

    curState = stateOf(curKey);
    cr = curState(1); cc = curState(2); ch = curState(3); ct = curState(4);

    if cr==goal(1) && cc==goal(2) && ch==goal(3)
        path = [cr cc ch];
        k = curKey;
        while isKey(cameFrom,k)
            k = cameFrom(k);
            s = stateOf(k);
            path = [s(1:3); path];
        end
        return;
    end

    if ct >= maxT
        continue;
    end

    for i = 1:size(moves,1)

        neighbour = [cr cc ch] + moves(i,1:3);
        nt = ct + 1;

        if neighbour(1)<1 || neighbour(1)>row || ...
           neighbour(2)<1 || neighbour(2)>col || ...
           neighbour(3)<1 || neighbour(3)>hgt
            continue;
        end
        if map(neighbour(1),neighbour(2),neighbour(3))==1
            continue;
        end

        cellK = sprintf('%d_%d_%d_%d',neighbour(1),neighbour(2),neighbour(3),nt);
        if isKey(reserved,cellK)
            continue;
        end

        swapK = sprintf('%d_%d_%d_%d',neighbour(1),neighbour(2),neighbour(3),ct);
        curNextK = sprintf('%d_%d_%d_%d',cr,cc,ch,nt);
        if isKey(reserved,swapK) && isKey(reserved,curNextK)
            continue;
        end

        nKey = sprintf('%d_%d_%d_%d',neighbour(1),neighbour(2),neighbour(3),nt);
        tentativeG = gScore(curKey) + costs(i);

        if ~isKey(gScore,nKey) || tentativeG < gScore(nKey)
            cameFrom(nKey) = curKey;
            gScore(nKey) = tentativeG;
            hn = abs(neighbour(1)-goal(1)) + abs(neighbour(2)-goal(2)) + abs(neighbour(3)-goal(3));
            fScore(nKey) = tentativeG + hn;
            stateOf(nKey) = [neighbour nt];
            if ~any(strcmp(openKeys,nKey))
                openKeys{end+1} = nKey;
            end
        end

    end

end

path = [];

end