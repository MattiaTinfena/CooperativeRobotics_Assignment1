classdef ActionManager < handle
    properties
        actions = {}      % cell array of actions (each action = stack of tasks)
        actionsTest = {}      % cell array of actions (each action = stack of tasks)
        actions_tag = {}
        actions_names = {}
        current_action = 1 % index of currently active action
        previous_action = 1
        action_changes = 0
        action_switch_time = {}
        all_task_list = {}
        all_task_names = []
        initial_time = 0
        tasks = {}
        ap_instructions = {}
    end

    methods
        % function addAction(obj, taskStack, name)
        %     % taskStack: cell array of tasks that define an action
        %     obj.actions_tag{end+1} = taskStack;
        %     for i=1:length(taskStack)
        %         for j=1:length(obj.all_task_names)
        %             if taskStack(i) == obj.all_task_names(j)
        %                 obj.actionsTest{end+1} = obj.all_task_list{j};
        %             end
        %         end
        %     end
        %     obj.actions{end+1} = obj.actionsTest;
        %     obj.actionsTest = {};
        %     disp(obj.actions_tag)
        %     disp(obj.actions)
        %     disp(obj.actionsTest)
        %     obj.actions_names{end+1} = name;
        % end

        function addAction(obj, taskStack, name)
            % taskStack: cell array di nomi task (string/char)

            obj.actions_tag{end+1} = taskStack;

            % Trova per ogni elemento di taskStack l'indice in obj.all_task_names
            [tf, idx] = ismember(taskStack, obj.all_task_names);

            % Se vuoi ignorare quelli non trovati:
            idx = idx(tf);

            % Costruisci direttamente l'azione (cell array di task)
            actionTasks = obj.all_task_list(idx);

            % Salva
            obj.actions{end+1} = actionTasks;
            obj.actions_names{end+1} = name;

        end


        function setTaskList(obj, task_list, task_names)
            obj.all_task_list = task_list;
            obj.all_task_names = task_names;
        end

        function [ydotbar] = computeICAT(obj, robot, time)
            % Get current action

            ap = {};
            if (obj.action_changes == 0)
                for i = 1:length(obj.tasks)
                    ap{end + 1} = 1;
                end
            else

                if(length(obj.ap_instructions) == length(obj.tasks))
                    for i = 1:length(obj.tasks)
                        if(obj.ap_instructions(i) == 1)
                            ap{end + 1} = IncreasingBellShapedFunction(obj.initial_time, obj.initial_time +2, 0, 1, time);
                        elseif(obj.ap_instructions(i) == -1)
                            ap{end + 1} = DecreasingBellShapedFunction(obj.initial_time, obj.initial_time +2, 0, 1, time);
                        else
                            ap{end + 1} = 1;
                        end
                    end
                else
                    disp("Error in computing ap_instructions")
                end

                % when gaussian transitory is ended
                if (time > obj.initial_time + 2)
                    % disp("Transition ended")
                    % disp("initial_time:");
                    % disp(obj.initial_time);
                    % disp("time:");
                    % disp(time);
                    obj.tasks = obj.actions{obj.current_action};
                    disp(obj.tasks);
                    obj.action_changes = 0;
                    ap = {};
                    for i = 1:length(obj.tasks)
                        ap{end + 1} = 1;
                    end
                end
            end

            % 1. Update references, Jacobians, activations
            for i = 1:length(obj.tasks)
                obj.tasks{i}.updateReference(robot);
                obj.tasks{i}.updateJacobian(robot);
                obj.tasks{i}.updateActivation(robot);
            end

            % 2. Perform ICAT (task-priority inverse kinematics)
            ydotbar = zeros(14,1);
            Qp = eye(14);
            for i = 1:length(obj.tasks)
                [Qp, ydotbar] = iCAT_task(obj.tasks{i}.A * ap{i}, obj.tasks{i}.J, ...
                                           Qp, ydotbar, obj.tasks{i}.xdotbar, ...
                                           1e-4, 0.01, 10);
            end

            % 3. Last task: residual damping
            [~, ydotbar] = iCAT_task(eye(14), eye(14), Qp, ydotbar, zeros(14,1), 1e-4, 0.01, 10);
        end

    function setCurrentAction(obj, actionName, time)

        found = false;

        for i = 1:length(obj.actions_names)
            if strcmp(obj.actions_names{i}, actionName)
                obj.previous_action = obj.current_action;
                obj.current_action = i;
                obj.action_changes = 1;
                obj.initial_time = time;
                found = true;
                break; % esci dal ciclo
            end
        end

        if ~found
            error('Action not found');
        end

        % act_tags  = obj.actions_tag{obj.current_action};
        % prev_tags = obj.actions_tag{obj.previous_action};

        % all_tags = unique([prev_tags, act_tags], 'stable');

        % [tf, idx] = ismember(all_tags, obj.all_task_names);
        % obj.tasks = obj.all_task_list(idx(tf));

        act_tags  = obj.actions_tag{obj.current_action};
        prev_tags = obj.actions_tag{obj.previous_action};

        % Unione stabile dei tag
        all_tags = unique([prev_tags, act_tags], 'stable');

        % Mappa tag → task object
        [tf_all, idx_all] = ismember(all_tags, obj.all_task_names);
        obj.tasks = obj.all_task_list(idx_all(tf_all));

        % Membership
        in_act  = ismember(all_tags, act_tags);
        in_prev = ismember(all_tags, prev_tags);

        % Preallocazione
        obj.ap_instructions = zeros(1, numel(all_tags));

        % Regole
        obj.ap_instructions( in_act &  in_prev) =  0;
        obj.ap_instructions( in_act & ~in_prev) = +1;
        obj.ap_instructions(~in_act &  in_prev) = -1;

        % Se vuoi mantenere allineamento perfetto con "tasks"
        obj.ap_instructions = obj.ap_instructions(tf_all);
        
        disp(act_tags)
        disp(prev_tags)
        disp(all_tags)
        disp(obj.ap_instructions)



    end

    end
end