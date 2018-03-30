function [rmse0,rmse1,X] = optimize_reprojection_error(X, data, camera)

if nargout == 1
    % Only return reprojection error
    
    % Question 2b:
    [~,rmse0] = compute_reprojection_error(X);
else
    % Full bundle adjustment
    
    % Question 2c
    % * Compute initial rmse error
    % * Run full bundle adustment
    % * Compute final rmse error
    
    % ...
    % ...
end

    function [e,rmse] = compute_reprojection_error(X)
        % Note that you can use the parameters data and camera in here:
        data;
        camera;
        
        % ...
        % ...

        % Compute total reprojection error e:
%         e = ...
        if nargout > 1 % Only output rmse if 2 output arguments are requested
            % Compute rmse error
            % rmse = ...
        end
    end
end


