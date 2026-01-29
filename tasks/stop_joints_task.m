classdef stop_joints_task < Task
    properties

    end

    methods
        function obj=stop_joints_task(robot_ID,taskID, smooth)
            obj.ID=robot_ID;
            obj.task_name=taskID;
            obj.smooth=smooth;
        end

        function updateReference(obj, robot_system)
            obj.xdotbar = zeros(14,1);
        end

        function updateJacobian(obj,robot_system)
            obj.J = eye(14);
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(14);
        end
    end
end