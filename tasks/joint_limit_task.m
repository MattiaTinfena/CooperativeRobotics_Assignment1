classdef joint_limit_task < Task
    properties
        delta = deg2rad(10);
    end

    methods
        function obj=joint_limit_task(robot_ID,taskID, smooth)
            obj.ID=robot_ID;
            obj.task_name=taskID;
            obj.smooth=smooth;
        end

        function updateReference(obj, robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;
            end

            err_min = robot.q - (robot.jlmin + obj.delta);
            err_max = robot.q - (robot.jlmax - obj.delta);

            err = min(err_min,0) + max(err_max,0);

            obj.xdotbar = -1 * err;
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.3);
        end

        function updateJacobian(obj,robot_system)

            if obj.ID=='L'
                obj.J=[eye(7), zeros(7, 7)];
            elseif obj.ID=='R'
                obj.J=[zeros(7, 7), eye(7)];
            end
        end

        function updateActivation(obj, robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;
            end

            lim_inf = [0 0 0 0 0 0 0]';
            lim_sup = [1 1 1 1 1 1 1]';
            A_max = arrayfun(@IncreasingBellShapedFunction,(robot.jlmax - obj.delta), robot.jlmax, lim_inf, lim_sup, robot.q);
            A_min = arrayfun(@DecreasingBellShapedFunction,robot.jlmin,(robot.jlmin + obj.delta), lim_inf, lim_sup, robot.q);

            obj.A = diag(A_max + A_min);
        end
    end
end