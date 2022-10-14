   function [indices, dists] = KDtreeFindnearest(this, point, K, varargin)
            %findNearestNeighbors Find the nearest neighbors of a point.
            %
            %  [indices, dists] = findNearestNeighbors(ptCloud, point, K)
            %  returns the K nearest neighbors of a query point. The point
            %  is an [X, Y, Z] vector. indices is a column vector that 
            %  contains K linear indices to the stored points in the point 
            %  cloud. dists is a column vector that contains K Euclidean 
            %  distances to the point. 
            %
            %  [...] = findNearestNeighbors(... , Name, Value)
            %  specifies additional name-value pairs described below:
            %  
            %  'Sort'          True or false. When set to true,
            %                  the returned indices are sorted in the
            %                  ascending order based on the distance
            %                  from a query point.
            %
            %                  Default: false
            %
            %  'MaxLeafChecks' An integer specifying the number of leaf
            %                  nodes that are searched in Kdtree. If the tree
            %                  is searched partially, the returned result may
            %                  not be exact. A larger number may increase the
            %                  search accuracy but reduce efficiency. When this
            %                  number is set to inf, the whole tree will be
            %                  searched to return exact search result. 
            %
            %                  Default: inf
            %
            %  Example : Find K-nearest neighbors
            %  ---------
            %  % Create a point cloud object with randomly generated points
            %  ptCloud = pointCloud(1000*rand(100,3,'single'));
            %
            %  % Define a query point and number of neighbors
            %  point = [50, 50, 50];
            %  K = 10;
            %
            %  % Get the indices and distances of 10 nearest points
            %  [indices, dists] = findNearestNeighbors(ptCloud, point, K);

            narginchk(3, 7);

            validateattributes(point, {'single', 'double'}, ...
                {'real', 'nonsparse', 'finite', 'size', [1, 3]}, mfilename, 'point');
            
            if isa(this.Location, 'single')
                point = single(point);
            else
                point = double(point);
            end

            validateattributes(K, {'single', 'double'}, ...
                {'nonsparse', 'scalar', 'positive', 'integer'}, mfilename, 'K');

            K = min(double(K), numel(this.Location)/3);

            [doSort, MaxLeafChecks] = validateAndParseSearchOption(varargin{:});

            % Use bruteforce if there are fewer than 500 points
            if numel(this.Location)/3 < 500
                if ~this.IsOrganized
                    allDists = visionSSDMetric(point', this.Location');
                else
                    allDists = visionSSDMetric(point', reshape(this.Location, [], 3)');
                end
                % This function will ensure returning actual number of neighbors
                % The result is already sorted
                [dists, indices] = vision.internal.partialSort(allDists, K);
                tf = isfinite(dists);
                indices = indices(tf);
                dists = dists(tf);
            else
                this.buildKdtree();

                searchOpts.checks = MaxLeafChecks;
                searchOpts.eps = 0;
                [indices, dists, valid] = this.Kdtree.knnSearch(point, K, searchOpts);
                
                % This step will ensure returning actual number of neighbors
                indices = indices(1:valid);
                dists = dists(1:valid);

                % Sort the result if specified
                if doSort
                    [dists, IND] = sort(dists);
                    indices = indices(IND);
                end
            end

            if nargout > 1
                dists = sqrt(dists);
            end
        end
