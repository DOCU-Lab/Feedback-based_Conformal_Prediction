function step(obj)
    % one step
    obj.x(1,1) = obj.x(1,1) + (cos(obj.x(3,1))*obj.x(4,1))*obj.ts;
    obj.x(2,1) = obj.x(2,1) + (sin(obj.x(3,1))*obj.x(4,1))*obj.ts;
    obj.x(3,1) = obj.x(3,1) + ((tan(obj.u(1,1))/obj.rr)*obj.x(4,1))*obj.ts;
    obj.x(4,1) = obj.x(4,1) + (obj.u(2,1))*obj.ts;
    obj.x_trajectory = [obj.x_trajectory, obj.x];       %record trajectory
end

