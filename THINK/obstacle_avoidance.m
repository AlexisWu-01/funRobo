function p_obstacle = obstacle_avoidance(lidar,angle_list,scan,scale)
%     [ranges, angles] = lidar(robo_pose, mapOfTrack);                         
%     scan = lidarScan(ranges, angles);
    distance = zeros(1,length(scale));
    for i = 1:length(angle_list)
        if scan.Ranges(i) < lidar.Range(2)
            distance(i) = scan.Ranges(i);
        end
    end
   total = sum(distance);
   p_obstacle = -distance./total;
end