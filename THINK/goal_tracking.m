function p_goal = goal_tracking(robo_pose, goal_point)
%     curr_orientation = robo_pose(3);
    target_orientation = tan((goal_point(1)-robo_pose(1))/ (goal_point(2)-robo_pose(2)));
    disp(target_orientation);
    scale = linspace(deg2rad(-90), deg2rad(90), 6);
    disp(scale);
    p_goal = normpdf(scale, target_orientation, 1);
    
end