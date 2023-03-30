function world_coord = transformCoord(robotPose, robot_coord)
    theta = robotPose(3);
    rotation_mat = [cos(theta), -sin(theta); sin(theta) cos(theta)];
    translation_mat = [1 0 robotPose(1); 0 1 robotPose(2); 0 0 1];
    rotated_pose = rotation_mat*robot_coord;
    rotated_pose(3) = 1;
    disp(rotated_pose);
    world_coord = translation_mat * rotated_pose;
    world_coord = world_coord([1 2]);
end