function [orientation_angle]=calculate_orientation(ranges,blocked_path,left)
    if ~left
        const=Constants;
        ranges=fillmissing(ranges,'linear'); %Trato nan interpolando
        ranges(ranges<0)=0;

        first_sample_index=1;
        first_sample_angle=const.lidar_angle_start;
        lidar_angle_delta=(const.lidar_angle_end-const.lidar_angle_start)/length(ranges);
        orientation_angle=-pi/2;
        if blocked_path
            first_sample_index=int32(length(ranges)/2-const.samples_angle/(2*lidar_angle_delta));
            first_sample_angle=-const.samples_angle/2;
            orientation_angle=0;

        end
        last_sample_index=first_sample_index+int32(const.samples_angle/(lidar_angle_delta));
        last_sample_angle=first_sample_angle+const.samples_angle;

        [first_sample_x,first_sample_y]=pol2cart(first_sample_angle,ranges(first_sample_index));
        [last_sample_x,last_sample_y]=pol2cart(last_sample_angle,ranges(last_sample_index));
        first_sample=[first_sample_x,first_sample_y];
        last_sample=[last_sample_x,last_sample_y];


        orientation_angle=orientation_angle+atan2(first_sample(2)-last_sample(2),first_sample(1)-last_sample(1));
        obstacles=[first_sample;last_sample];
    else
        %Orientation angle: Lo que queremos que rote 
        const=Constants;
        ranges=fillmissing(ranges,'linear'); %Trato nan interpolando
        ranges(ranges<0)=0;

        last_sample_index=length(ranges);
        last_sample_angle=const.lidar_angle_end;
        lidar_angle_delta=(const.lidar_angle_end-const.lidar_angle_start)/length(ranges);
        orientation_angle=-pi/2;
        if blocked_path
            last_sample_index=int32(length(ranges)/2-const.samples_angle/(2*lidar_angle_delta));
            last_sample_angle=-const.samples_angle/2;
            orientation_angle=0;

        end
        first_sample_index=last_sample_index-int32(const.samples_angle/(lidar_angle_delta));
        first_sample_angle=last_sample_angle-const.samples_angle;
        [first_sample_x,first_sample_y]=pol2cart(first_sample_angle,ranges(first_sample_index));
        [last_sample_x,last_sample_y]=pol2cart(last_sample_angle,ranges(last_sample_index));
        first_sample=[first_sample_x,first_sample_y];
        last_sample=[last_sample_x,last_sample_y];
        orientation_angle=orientation_angle+atan2(first_sample(2)-last_sample(2),first_sample(1)-last_sample(1));
        %obstacles=[first_sample;last_sample];
    end
    end
% function [orientation_angle,wall_normal_vector]=calculate_orientation(ranges,blocked_path)
%     const=Constants;
%     meas_index=(1:length(ranges))';
%     ranges=fillmissing(ranges,'linear'); %Trato nan interpolando
%     ranges(ranges<0)=0;
%     %first_sample_index=1;
%     %first_sample_angle=const.lidar_angle_start;
%     lidar_angle_delta=(const.lidar_angle_end-const.lidar_angle_start)/length(ranges);
%     %if blocked_path
%     first_sample_index=int32(length(ranges)/2-const.samples_angle/(2*lidar_angle_delta));
%     first_sample_angle=-const.samples_angle/2;
%     %end
%     last_sample_index=first_sample_index+int32(const.samples_angle/(lidar_angle_delta));
%     last_sample_angle=first_sample_angle+const.samples_angle;
%     [first_sample_x,first_sample_y]=pol2cart(first_sample_angle,ranges(first_sample_index));
%     [last_sample_x,last_sample_y]=pol2cart(last_sample_angle,ranges(last_sample_index));
%     first_sample=[first_sample_x,first_sample_y];
%     last_sample=[last_sample_x,last_sample_y];
%     
%     %orientation_angle=atan2(first_sample(2)-last_sample(2),first_sample(1)-last_sample(1));
%     wall_normal_vector=find_normal_vector(first_sample,last_sample);
%     orientation_angle=atan2(wall_normal_vector(2),wall_normal_vector(1));
%     %%Ver cuando estemos m�s frescos
%     if abs(angdiff(orientation_angle,0))>pi/2
%         orientation_angle=angdiff(orientation_angle,pi);
%     end
% end