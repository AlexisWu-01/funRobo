function [plist, angle_list] =  SENSE(robo_pose, mapOfTrack, blankLidarMap,fig_lidarMap,fig_localLidarPlot,lidar,goal_point)

% SENSE scans the reference map using the range sensor and the current
% pose.
% This simulates normal range readings for driving in an unknown
% environment
% Update the lidar map with the range readings
% inpus are rover position, mapOfTrack, lidarMap, figure to plot global
% lidar, figure to plot local lidar, sensor name outputs are lidar ranges
% and angles in local coordinate system

    [ranges, angles] = lidar(robo_pose, mapOfTrack);                         
    scan = lidarScan(ranges, angles);
    validScan = removeInvalidData(scan, "rangeLimits",[0,lidar.Range(2)]);
    angle_list = linspace(deg2rad(-90), deg2rad(90), 6);
    p_obstacle = obstacle_avoidance(lidar, angle_list, scan,angle_list);
    p_goal = goal_tracking(robo_pose, goal_point);
    plist = p_goal+p_obstacle;
    insertRay(blankLidarMap, robo_pose, validScan, lidar.Range(2));

    figure(fig_lidarMap);
        hold on;
        show(blankLidarMap);
        grid on;
        grid minor;
        hold off;
   

    figure(fig_localLidarPlot);
    plot(scan);
    hold on;
    plot(0,0,'r*');
    hold off;
    workspace;
end