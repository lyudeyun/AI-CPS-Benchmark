classdef StaliroRobustnessAuxData < handle
    % StaliroRobustnessAuxData : Auxiliary data and methods that the
    % optimizer may need to pass to the simulation and robustness 
    % computation engines.
    %
    % This is optional and internal to the optimizer, but it enables 
    % tighter integration between the optimizer and the robustness
    % computation for custom functionality and support.
    %
    % See also: STaliroTimeStats
    
    % (C) G. Fainekos - S-TaLiRo
    
    properties
        % idx_phi : A vector of unique integers, or empty.
        %
        % If a cell array of STL/MTL specifications is provided as
        % input to staliro, then the property idx_phi can be used to
        % request which requirement should be evaluated on the test output 
        % trajectory. If the vector is empty, then all requirements will be 
        % evaluated on the test vector.
        %
        % For example, if 
        %   idx_phi = [2, 3]; 
        % and 
        %   phi = {'!<>_[3.5,4.0] b)', 'a U_[0,5] b', '[] c'}
        % then Compute_Robustness should return a vector of two robustness
        % values corresponding to the specifications 2 and 3 from phi.
        %
        idx_phi = {};
        
        % timeStats : An oject of class STaliroTimeStats for collecting
        % time statistics. 
        %
        % The property is initialized with an empty array (defualt value) 
        % to indicate that time statistics will not be collected.
        %
        % The property must be initialized before with instantiating 
        %
        timeStats = STaliroTimeStats;
                
    end
    
    methods
        function obj = StaliroRobustnessAuxData
            % Object constructor
        end
        
        function set.idx_phi(obj, inp)
            if ~isempty(inp)
                assert(isvector(inp) && isnumeric(inp) && min(inp>0), ' The input must be a vector of positive integers.');
                assert(length(inp)==length(unique(inp)), ' The input must be a vector of unique positive integers.');
                for i = 1:length(inp)
                    assert(inp(i)==floor(inp(i)), ' The input must be a vector of positive integers.')
                end
            end
            obj.idx_phi = inp;
        end
        
        function set.timeStats(obj, inp)
            assert(isa(inp, 'STaliroTimeStats'), ' The input must be a STaliroTimeStats object.');
            obj.timeStats = inp;
        end
        
    end
    
end

