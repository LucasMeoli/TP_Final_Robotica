function [orientation_angle]=calculate_orientation(ranges,blocked_path,left)
%Funciona para determinar cuanto queremos que rote una vez que las
%mediciones determinó que la distancia minima medida es menor al umbral
%establecido.

%     if (left == false)
        const = Constants;
        ranges = fillmissing(ranges,'constant',5); 
        ranges(ranges<0) = 0;

        first_sample_index = 1;
        first_sample_angle = const.lidar_angle_start;
        lidar_angle_delta = (const.lidar_angle_end - const.lidar_angle_start)/length(ranges);
        orientation_angle = -pi/2;
        if (blocked_path == true)
            first_sample_index = int32(length(ranges)/2 - const.samples_angle/(2*lidar_angle_delta));
            first_sample_angle = -const.samples_angle/2;
            orientation_angle = 0;
        end
        
        last_sample_index = first_sample_index + int32(const.samples_angle/(lidar_angle_delta));
        last_sample_angle = first_sample_angle + const.samples_angle;      
       
%     else
%         const = Constants;
%         ranges = fillmissing(ranges,'constant',5); 
%         ranges(ranges<0) = 0;
% 
%         last_sample_index=length(ranges);
%         last_sample_angle=const.lidar_angle_end;
%         lidar_angle_delta=(const.lidar_angle_end-const.lidar_angle_start)/length(ranges);
%         orientation_angle=-pi/2;
%         
%         if blocked_path
%             last_sample_index=int32(length(ranges)/2-const.samples_angle/(2*lidar_angle_delta));
%             last_sample_angle=-const.samples_angle/2;
%             orientation_angle=0;
%         end
%         
%         first_sample_index = last_sample_index - int32(const.samples_angle/(lidar_angle_delta));
%         first_sample_angle = last_sample_angle - const.samples_angle;
%     end
    
    [first_sample_x,first_sample_y] = pol2cart(first_sample_angle,ranges(first_sample_index));
    [last_sample_x,last_sample_y] = pol2cart(last_sample_angle,ranges(last_sample_index));
    
    first_sample = [first_sample_x, first_sample_y];
    last_sample = [last_sample_x, last_sample_y];
    %Orientation angle: Lo que queremos que rote 
    orientation_angle = orientation_angle + atan2(first_sample(2) - last_sample(2), first_sample(1) - last_sample(1));
end
