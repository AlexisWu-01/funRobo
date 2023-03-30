function [roverX, roverY, poses] = THINK(poses, diffDrive, ppControl, sampleTime, loopIndex,plist)
    % Run the Pure Pursuit controller and convert output to wheel speeds
    turn_speed = [-3 -2 -1 1 2 3];

    [wref,vref] = ppControl(poses(:,loopIndex));
    [pmax, idx] = max(plist);
    wRef = turn_speed(idx);
    vRef = 2;
    disp([wref, vref, wRef, vRef]);
    % Perform forward discrete integration step
    vel = derivative(diffDrive, poses(:,loopIndex), [vRef+vref, wRef+wref]);
    poses(:,loopIndex+1) = poses(:,loopIndex) + vel*sampleTime;

    %update rover location and pose
    roverX = poses(1,loopIndex+1);
    roverY = poses(2,loopIndex+1);

end
