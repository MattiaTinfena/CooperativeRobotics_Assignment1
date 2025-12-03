classdef SimulationLogger < handle
    properties
        t            % time vector
        ql            % joint positions
        qdotl        % joint velocities
        qr            % joint positions
        qdotr        % joint velocities
        a            % task activations (diagonal only)
        xdotbar_task % reference velocities for tasks (cell array)
        robot        % robot model
        action_set     % set of actions
        n
    end

    methods
        function obj = SimulationLogger(maxLoops, robotModel, action_set)
            obj.robot = robotModel;
            obj.action_set = action_set;

            obj.t = zeros(1, maxLoops);
            obj.ql = zeros(7, maxLoops);
            obj.qdotl = zeros(7, maxLoops);
            obj.qr = zeros(7, maxLoops);
            obj.qdotr = zeros(7, maxLoops);
            obj.n=length(action_set.actions);
            l=zeros(1,obj.n);
            for i=1:obj.n
                l(i)=length(action_set.actions{i});
            end

            obj.xdotbar_task=cell(length(action_set.actions), max(l), maxLoops);
            
        end

        function update(obj, t, loop)
            % Store robot state
            obj.t(loop) = t;
            obj.ql(:, loop) = obj.robot.left_arm.q;
            obj.qdotl(:, loop) = obj.robot.left_arm.qdot;
            obj.qr(:, loop) = obj.robot.right_arm.q;
            obj.qdotr(:, loop) = obj.robot.right_arm.qdot;
            %Store task reference velocities
            for i=1:obj.n
                for j=1:length(obj.action_set.actions{i})
                    obj.xdotbar_task{i,j,loop}=obj.action_set.actions{i}{j}.xdotbar;
                end
            end


        end
        function plotAll(obj,action,task)
                % Example plotting for robot state
                figure(1);
                subplot(2,1,1);
                title('Left arm motion');
                plot(obj.t, obj.ql, 'LineWidth', 2);
                legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
                subplot(2,1,2);
                plot(obj.t, obj.qdotl, 'LineWidth', 2);
                legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7');
                figure(2);
                title('Right arm motion');    
                subplot(2,1,1);
                plot(obj.t, obj.qr, 'LineWidth', 2);
                legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
                subplot(2,1,2);
                plot(obj.t, obj.qdotr, 'LineWidth', 2);
                legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7');

                % Optional: plot a number of tasks from an specific action set
                figure(3);
                s=squeeze(obj.xdotbar_task(action,:,:));
                nt=length(task);
                title(strcat('Action Set'," ",num2str(action)));  
                for i=1:nt
                    subplot(1,nt,i)
                    data=cell2mat(s(task(i),:));
                    plot(obj.t(1,1:end-1),data','.-')
                    grid on
                    if(obj.action_set.actions{action}{task(i)}.task_name=="T")
                        legend('wx','wy','wz','vx','vy','vz');
                    end
                    title([strcat('Robot'," ",obj.action_set.actions{action}{task(i)}.ID," ",'Task'," ",num2str(task(i))," ",obj.action_set.actions{action}{task(i)}.task_name)]);
                end
            end

    end
end