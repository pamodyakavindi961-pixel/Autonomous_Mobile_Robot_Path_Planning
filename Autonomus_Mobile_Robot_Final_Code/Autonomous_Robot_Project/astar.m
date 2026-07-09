function path = astar(map,start,goal)

[row,col,hgt] = size(map);

closed = false(row,col,hgt);
parent = cell(row,col,hgt);

g = inf(row,col,hgt);
f = inf(row,col,hgt);

g(start(1),start(2),start(3)) = 0;
h = abs(start(1)-goal(1)) + abs(start(2)-goal(2)) + abs(start(3)-goal(3));
f(start(1),start(2),start(3)) = h;

openList = [start f(start(1),start(2),start(3))];

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

while ~isempty(openList)

    [~,index] = min(openList(:,4));
    current = openList(index,1:3);
    openList(index,:) = [];

    if isequal(current,goal)

        path = goal;

        while ~isequal(current,start)

            current = parent{current(1),current(2),current(3)};
            path = [current; path];

        end

        path = smoothPath(map,path);

        return;

    end

    closed(current(1),current(2),current(3)) = true;

    for i = 1:size(moves,1)
        neighbour = current + moves(i,:);

        if neighbour(1)<1 || neighbour(1)>row || ...
           neighbour(2)<1 || neighbour(2)>col || ...
           neighbour(3)<1 || neighbour(3)>hgt
            continue;
        end
        if map(neighbour(1),neighbour(2),neighbour(3))==1
            continue;
        end
        if closed(neighbour(1),neighbour(2),neighbour(3))
            continue;
        end

        newG = g(current(1),current(2),current(3)) + costs(i);

        if newG < g(neighbour(1),neighbour(2),neighbour(3))
            parent{neighbour(1),neighbour(2),neighbour(3)} = current;
            g(neighbour(1),neighbour(2),neighbour(3)) = newG;
            h = abs(neighbour(1)-goal(1)) + abs(neighbour(2)-goal(2)) + abs(neighbour(3)-goal(3));
            f(neighbour(1),neighbour(2),neighbour(3)) = newG + h;

            idx = find(openList(:,1)==neighbour(1) & openList(:,2)==neighbour(2) & openList(:,3)==neighbour(3));
            if isempty(idx)
                openList = [openList; neighbour f(neighbour(1),neighbour(2),neighbour(3))];
            else
                openList(idx,4) = f(neighbour(1),neighbour(2),neighbour(3));
            end
        end
    end
end

path = [];
end

function smoothed = smoothPath(map,path)

smoothed = path(1,:);
current = 1;
n = size(path,1);

while current < n

    next = n;

    while next > current+1 && ~lineOfSight(map,path(current,:),path(next,:))
        next = next - 1;
    end

    smoothed = [smoothed; path(next,:)];
    current = next;

end

end


function clearLine = lineOfSight(map,p1,p2)

steps = max(abs(p2-p1)) * 2;

if steps == 0
    clearLine = true;
    return;
end

clearLine = true;

for t = 0:1/steps:1
    pt = round(p1 + t*(p2-p1));
    if map(pt(1),pt(2),pt(3)) == 1
        clearLine = false;
        return;
    end
end

end