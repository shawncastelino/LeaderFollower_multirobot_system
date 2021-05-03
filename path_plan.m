function [path] = path_plan(map,robotradius,startLocation,endLocation)


mapinflated = copy(map);
inflate(mapinflated,robotradius);

show(mapinflated)

prm = mobileRobotPRM;
prm.Map = mapinflated;

prm.NumNodes = 2500;
prm.ConnectionDistance = 5;

path = findpath(prm, startLocation, endLocation);
while isempty(path)
        
    % No feasible path found yet, increase the number of nodes
    prm.NumNodes = prm.NumNodes + 100;
    
    % Use the |update| function to re-create the PRM roadmap with the changed
    % attribute
    update(prm);
    
    % Search for a feasible path with the updated PRM
    path = findpath(prm, startLocation, endLocation);
end
end

