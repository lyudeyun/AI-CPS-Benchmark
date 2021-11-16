classdef export_ARCH_options < handle
    % export_ARCH_options : Options for exporting ARCH CSV files
    
    % (C) G. Fainekos - S-TaLiRo
    
    properties
        % output : Export or not in the CSV file the output trajectories.
        % You may want to set it to false in case of very large output
        % trajectories.
        %
        %   default value: true
        output = true;
        
        % validateRobustness : validate the robustness value after running
        % the simulation. This may be useful in the case of old saved
        % results.
        %
        %   default value: false
        validateRobustness = false;
        
        % phi : the formula for the robustness validation.
        phi = [];
        
        % preds : the predicate mapping for the robustness validation.
        preds = [];
        
        % tolerance : the tolerance for checking robustness equivalence.
        % It can be any positive value delta such as the absolute value of  
        % the difference between the saved and computed robustness values 
        % is bounded be delta. If the value -1 is set, then bitwise
        % equality is requested.
        %   
        %   default value: -1 (check bitwise equivalence)
        tolerance = -1;
        
    end
    
    methods
        function obj = export_ARCH_options
            % Object constructor
        end
        
        function set.tolerance(obj, inp)
            if inp==-1 || inp>0
                obj.tolerance = inp;
            else
                error(' The tolerance property can either be -1 or a positive value.')
            end
        end
    end
    
end

