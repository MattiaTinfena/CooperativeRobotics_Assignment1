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

        function addAction(obj, taskStack, name)

            obj.actions_tag{end+1} = taskStack;
            [tf, idx] = ismember(taskStack, obj.all_task_names);

            idx = idx(tf);
            actionTasks = obj.all_task_list(idx);

            obj.actions{end+1} = actionTasks;
            obj.actions_names{end+1} = name;

        end


        function setTaskList(obj, task_list, task_names)
            obj.all_task_list = task_list;
            obj.all_task_names = task_names;
        end

        function [ydotbar] = computeICAT(obj, robot, time)


            if (obj.action_changes == 0)
                for i = 1:length(obj.tasks)
                    obj.tasks{i}.ap = 1;
                end
            else
                if(length(obj.ap_instructions) == length(obj.tasks))
                    for i = 1:length(obj.tasks)
                        if(obj.ap_instructions(i) == 1)
                            if(obj.tasks{i}.smooth)
                                obj.tasks{i}.ap = IncreasingBellShapedFunction(obj.initial_time, obj.initial_time +2, 0, 1, time);
                            else
                                obj.tasks{i}.ap = 1;
                            end
                        elseif(obj.ap_instructions(i) == -1)
                            obj.tasks{i}.ap = DecreasingBellShapedFunction(obj.initial_time, obj.initial_time +2, 0, 1, time);
                        else
                            obj.tasks{i}.ap = 1;
                        end
                    end
                else
                    disp("Error in computing ap_instructions")
                end

                % when gaussian transitory is ended
                if (time > obj.initial_time + 2)
                    obj.tasks = obj.actions{obj.current_action};
                    % disp(obj.tasks);
                    obj.action_changes = 0;
                    for i = 1:length(obj.tasks)
                        obj.tasks{i}.ap = 1;
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
                [Qp, ydotbar] = iCAT_task(obj.tasks{i}.A * obj.tasks{i}.ap, obj.tasks{i}.J, ...
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
                    break;
                end
            end

            if ~found
                error('Action not found');
            end


            % ... (trova l'indice dell'azione come già fai) ...

            act_tags  = obj.actions_tag{obj.current_action};
            prev_tags = obj.actions_tag{obj.previous_action};

            % 1. Identifica tutti i tag coinvolti nella transizione
            union_tags = union(act_tags, prev_tags);

            % 2. FILTRO CRUCIALE: Crea all_tags seguendo l'ordine di 'all_task_names'
            % ismember(A, B) con 'stable' non basta qui, vogliamo l'ordine di obj.all_task_names
            mask_involved = ismember(obj.all_task_names, union_tags);
            all_tags = obj.all_task_names(mask_involved); % Questi sono i tag ordinati per priorità

            % 3. Ora recupera gli oggetti task corrispondenti
            [tf_all, idx_all] = ismember(all_tags, obj.all_task_names);
            obj.tasks = obj.all_task_list(idx_all(tf_all));

            % 4. Calcola le istruzioni di attivazione basandoti sul nuovo all_tags
            in_act  = ismember(all_tags, act_tags);
            in_prev = ismember(all_tags, prev_tags);

            obj.ap_instructions = zeros(1, numel(all_tags));
            obj.ap_instructions( in_act &  in_prev) =  0;  % Resta attivo
            obj.ap_instructions( in_act & ~in_prev) = +1;  % Deve attivarsi
            obj.ap_instructions(~in_act &  in_prev) = -1;  % Deve spegnersi

            % 1. Identifica tutti i tag coinvolti nella transizione
            union_tags = union(act_tags, prev_tags);

            % 2. FILTRO CRUCIALE: Crea all_tags seguendo l'ordine di 'all_task_names'
            % ismember(A, B) con 'stable' non basta qui, vogliamo l'ordine di obj.all_task_names
            mask_involved = ismember(obj.all_task_names, union_tags);
            all_tags = obj.all_task_names(mask_involved); % Questi sono i tag ordinati per priorità

            % 3. Ora recupera gli oggetti task corrispondenti
            [tf_all, idx_all] = ismember(all_tags, obj.all_task_names);
            obj.tasks = obj.all_task_list(idx_all(tf_all));

            % 4. Calcola le istruzioni di attivazione basandoti sul nuovo all_tags
            in_act  = ismember(all_tags, act_tags);
            in_prev = ismember(all_tags, prev_tags);

            obj.ap_instructions = zeros(1, numel(all_tags));
            obj.ap_instructions( in_act &  in_prev) =  0;  % Resta attivo
            obj.ap_instructions( in_act & ~in_prev) = +1;  % Deve attivarsi
            obj.ap_instructions(~in_act &  in_prev) = -1;  % Deve spegnersi end
        end
    end
end