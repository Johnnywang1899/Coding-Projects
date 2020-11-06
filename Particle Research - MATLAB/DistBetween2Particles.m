function distance = DistBetween2Particles(p1,p2)


%set1=[1 1 1];                                %test
%set2=[1 1 2; 4 2 2; 8 9 9; 10 10 10];

%distance = zeros(size(inRange, 1), 3);

%test = pdist2( set1, set2, 'euclidean');     %test

%allDist = squareform( pdist2( set1, set2,'euclidean') );
%[minDist nni] = min( allDist, [], 2 ); 

for n = 1 : size(p2,1);
    distance(n,:) = pdist2(p1, p2(n,:));
end

%for n = 1 : 4;                                    %test
%    test(n,:) = pdist2( set1, set2(n,:));
%end
    