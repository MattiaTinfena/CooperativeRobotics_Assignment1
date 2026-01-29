classdef minimum_altitude_task < Task
    properties

    end

    methods
        function obj=minimum_altitude_task(robot_ID,taskID, smooth)
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

            robot.alt = robot.wTt(3,4);

            if size(robot.alt) == 1
                obj.xdotbar = 0.2 * max(0, (0.20 - robot.alt));
            else
                obj.xdotbar = 0;
            end
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj,robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;
            end
            tool_jacobian=robot.wJt;

            n = [0 0 0 0 0 1];
            if obj.ID=='L'
                obj.J = n * [tool_jacobian, zeros(6, 7)];
            elseif obj.ID=='R'
                obj.J = n * [zeros(6, 7), tool_jacobian];
            end

        end

        function updateActivation(obj, robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;
            end

            if size(robot.alt) == 1
                obj.A = DecreasingBellShapedFunction(0.15,0.20,0,1,robot.alt);
            else
                obj.A = 0;
            end
        end
    end
end