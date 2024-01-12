function [data_new] = set_value(data)
    
    data_new = zeros(1,length(data));
    for i = 2:length(data)
        if(~isnan(data(i)))
            data_new(i) = data(i);
        % elseif(~isnan(data(i)))
        %     data_new(i) = 0;
        else
            data_new(i) = data_new(i-1);
        end
    end

end