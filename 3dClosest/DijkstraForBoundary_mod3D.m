function [pathL,goalf] = DijkstraForBoundary_mod3D(Graph, source, goal)
%DijkstraForBoundary_mod is a shortest distance function which returns pathL ` 
%Graph is a binary array, with 0 for every white, red, or blue square and
% 1 for everything else
% source is a list of the coordinates on Graph that are red (robots)
% Goal is a list of the coordinates of Graph that are blue (frontiers)
%  Authors: Arun Mahadev & Aaron T. Becker, based on
% https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
Graph=flipud(Graph);
Q = find(Graph==0); %  create vertex set Q
dist = Inf*ones(size(Graph));
prevL = repmat('?',size(Graph));
prev = -1*ones(size(Graph));
path = []; %#ok<NASGU>
pathL = [];       
dist(source) = 0;  % Distance from source to source is zero
dist=flipud(dist);
goalmat=zeros(size(Graph));
goalmat(goal)=1;
goalmat=flipud(goalmat);
goal=find(goalmat);
% while Q is not empty:
while numel(Q)>0
    
    [~,ind] = min(dist(Q)); % Source nodes will be selected first
    dirs = [ 0,-1,0;-1,0,0; 0,1,0; 1,0,0; 0,0,-1; 0,0,1 ]; % left, up, right, down
    u = Q(ind);
    Q(ind) = []; % remove u from Q
    dirLetter = ['l','u','r','d','n','s'];
    [ui,uj,uk] = ind2sub(size(Graph),u);
    for i = 1:size(dirs,1) % for each neighbor v of u: where v is still in Q.
        v = sub2ind( size(Graph), dirs(i,1) + ui,  dirs(i,2) +uj, dirs(i,3)+uk); 
        if Graph(v) == 0 %only try to move if the vertex is 0
            alt = 1+dist(u); %distance to node v if we come from node u
            if alt < dist(v)
                dist(v) = alt;
                prevL(v) = dirLetter(i);
                prev(v) = u;
                % find if we have reached a goal node is v in goal.If so,
                % return path
               if any(goal==v)
                   goalf=v;
                   path = zeros(1,dist(v));
                   pathL = repmat(' ',1,dist(v));
                   path(dist(v)) = prev(v);
                   pathL(dist(v)) = prevL(v);
                   for k = dist(v)-1:-1:1
                       path(k) = prev(path(k+1));
                       pathL(k) =prevL(path(k+1));
                   end
                   Q = [];
                   
               end
            end
        end
    end  
end
goalbackmat=zeros(size(goalmat));
    goalbackmat(goalf)=1;
    goalbackmat=flipud(goalbackmat);
    goalf=find(goalbackmat); %finding location of goal frontier cell to be returned.

end