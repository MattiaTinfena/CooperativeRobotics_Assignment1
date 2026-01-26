classdef move_object_task < Task
    %Tool position control for a single arm
    properties

    end

    methods
        function obj=move_object_task(robot_ID,taskID, smooth)
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


            r_toc = robot.wTo(1:3, 4) - robot.wTg(1:3,4);
            tToc = [eye(3), robot.wTt(1:3, 1:3)' * r_toc; 0 0 0 1];
            wToc = robot.wTt * tToc;
            wTog = [robot.wTt(1:3, 1:3) * robot.wTog(1:3, 1:3), robot.wTog(1:3, 4); 0 0 0 1];

            [v_ang, v_lin] = CartError(wTog ,wToc);
            robot.dist_to_goal=v_lin;
            robot.rot_to_goal=v_ang;

            obj.xdotbar =  0.7 * [v_ang; v_lin];
            % limit the requested velocities...
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.3);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.3);
        end

        function updateJacobian(obj,robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;
            end
            tool_jacobian=robot.wJt;

            r_toc = robot.wTo(1:3, 4) - robot.wTg(1:3,4);
            wS_toc = [eye(3) zeros(3); skew(-robot.wTt(1:3, 1:3)' * r_toc) eye(3)];

            if obj.ID=='L'
                obj.J=[wS_toc * tool_jacobian, zeros(6, 7)];
            elseif obj.ID=='R'
                obj.J=[zeros(6, 7), wS_toc * tool_jacobian];
            end
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end