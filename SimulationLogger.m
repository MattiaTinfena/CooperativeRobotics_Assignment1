classdef SimulationLogger < handle
    properties
        t

        ql
        qdotl
        qr
        qdotr

        alt_l
        alt_r
        dist_l
        dist_r
        rot_l
        rot_r

        tools_distance
        tools_orientation

        % --- NUOVE PROPRIETA' PER LE VELOCITA' ---
        % Left Arm
        v_lin_tool_l
        v_ang_tool_l
        v_lin_obj_l
        v_ang_obj_l

        % Right Arm
        v_lin_tool_r
        v_ang_tool_r
        v_lin_obj_r
        v_ang_obj_r
        % -----------------------------------------

        xdotbar_task
        activation_task
        priority_task
        product_task

        global_activations
        total_tasks

        robot
        action_mng
        n
        curr_loop

        switch_times = []
        switch_names = {}
        last_action_idx = 0
    end

    methods
        function obj = SimulationLogger(maxLoops, robotModel, actionManager)
            obj.robot = robotModel;
            obj.action_mng = actionManager;
            obj.curr_loop = 0;

            obj.t = zeros(1, maxLoops);

            obj.ql = zeros(7, maxLoops);
            obj.qdotl = zeros(7, maxLoops);
            obj.qr = zeros(7, maxLoops);
            obj.qdotr = zeros(7, maxLoops);

            obj.alt_l = zeros(1, maxLoops);
            obj.alt_r = zeros(1, maxLoops);
            obj.dist_l = zeros(1, maxLoops);
            obj.dist_r = zeros(1, maxLoops);
            obj.rot_l = zeros(3, maxLoops);
            obj.rot_r = zeros(3, maxLoops);

            obj.tools_distance = zeros(1, maxLoops);
            obj.tools_orientation = zeros(1, maxLoops);

            % --- INIZIALIZZAZIONE NUOVE MATRICI ---
            obj.v_lin_tool_l = zeros(3, maxLoops);
            obj.v_ang_tool_l = zeros(3, maxLoops);
            obj.v_lin_obj_l  = zeros(3, maxLoops);
            obj.v_ang_obj_l  = zeros(3, maxLoops);

            obj.v_lin_tool_r = zeros(3, maxLoops);
            obj.v_ang_tool_r = zeros(3, maxLoops);
            obj.v_lin_obj_r  = zeros(3, maxLoops);
            obj.v_ang_obj_r  = zeros(3, maxLoops);
            % --------------------------------------

            obj.n = length(actionManager.actions);
            l = zeros(1, obj.n);
            for i = 1:obj.n
                l(i) = length(actionManager.actions{i});
            end
            max_tasks = max(l);

            obj.xdotbar_task = cell(obj.n, max_tasks, maxLoops);
            obj.activation_task = cell(obj.n, max_tasks, maxLoops);
            obj.priority_task = cell(obj.n, max_tasks, maxLoops);
            obj.product_task = cell(obj.n, max_tasks, maxLoops);

            obj.total_tasks = length(actionManager.all_task_list);
            obj.global_activations = zeros(obj.total_tasks, maxLoops);

            obj.last_action_idx = 0;
        end

        function update(obj, t, loop)
            obj.curr_loop = loop;
            obj.t(loop) = t;

            current_act = obj.action_mng.current_action;
            if current_act ~= obj.last_action_idx
                obj.switch_times(end+1) = t;
                if ~isempty(obj.action_mng.actions_names)
                    obj.switch_names{end+1} = obj.action_mng.actions_names{current_act};
                else
                    obj.switch_names{end+1} = sprintf("Act %d", current_act);
                end
                obj.last_action_idx = current_act;
            end

            obj.ql(:, loop) = obj.robot.left_arm.q;
            obj.qdotl(:, loop) = obj.robot.left_arm.qdot;
            obj.qr(:, loop) = obj.robot.right_arm.q;
            obj.qdotr(:, loop) = obj.robot.right_arm.qdot;

            if ~isempty(obj.robot.left_arm.alt), obj.alt_l(loop) = obj.robot.left_arm.alt; end
            if ~isempty(obj.robot.right_arm.alt), obj.alt_r(loop) = obj.robot.right_arm.alt; end

            d_l = obj.robot.left_arm.dist_to_goal;
            if ~isempty(d_l), if isscalar(d_l), obj.dist_l(loop) = d_l; else, obj.dist_l(loop) = norm(d_l); end, end

            d_r = obj.robot.right_arm.dist_to_goal;
            if ~isempty(d_r), if isscalar(d_r), obj.dist_r(loop) = d_r; else, obj.dist_r(loop) = norm(d_r); end, end

            r_l = obj.robot.left_arm.rot_to_goal;
            if ~isempty(r_l), n_rows = min(3, length(r_l)); obj.rot_l(1:n_rows, loop) = r_l(1:n_rows); end

            r_r = obj.robot.right_arm.rot_to_goal;
            if ~isempty(r_r), n_rows = min(3, length(r_r)); obj.rot_r(1:n_rows, loop) = r_r(1:n_rows); end

            [~, lin_ang] = CartError(obj.robot.left_arm.wTt ,obj.robot.right_arm.wTt);
            obj.tools_distance(loop) = norm(lin_ang);

            R_L = obj.robot.left_arm.wTt(1:3, 1:3);
            % R_L = diag([-1 -1 1])*R_L;
            R_R = obj.robot.right_arm.wTt(1:3, 1:3);

            R_diff = R_L' * R_R;

            obj.tools_orientation(loop) = norm(VersorLemma(R_diff, eye(3)));

            % =========================================================
            % --- USER TO FILL: VELOCITIES (Tool & Object) ---
            % =========================================================
            twist_l = obj.robot.left_arm.wJt * obj.robot.left_arm.qdot;
            twist_r = obj.robot.right_arm.wJt* obj.robot.right_arm.qdot;

            % --- LEFT ARM ---
            % Velocità Lineare Tool (3x1)
            temp_v_lin_tool_l = twist_l(4:6);

            % Velocità Angolare Tool (3x1)
            temp_v_ang_tool_l = twist_l(1:3);

            % Velocità Lineare Oggetto (3x1)
            temp_v_lin_obj_l  = obj.robot.left_arm.object_des_vel(4:6);

            % Velocità Angolare Oggetto (3x1)
            temp_v_ang_obj_l  = obj.robot.left_arm.object_des_vel(1:3);

            % --- RIGHT ARM ---
            % Velocità Lineare Tool (3x1)
            temp_v_lin_tool_r = twist_r(4:6);

            % Velocità Angolare Tool (3x1)
            temp_v_ang_tool_r = twist_r(1:3);

            % Velocità Lineare Oggetto (3x1)
            temp_v_lin_obj_r  = obj.robot.right_arm.object_des_vel(4:6);

            % Velocità Angolare Oggetto (3x1)
            temp_v_ang_obj_r  = obj.robot.right_arm.object_des_vel(1:3);

            % --- Salvataggio nella history ---
            obj.v_lin_tool_l(:, loop) = temp_v_lin_tool_l;
            obj.v_ang_tool_l(:, loop) = temp_v_ang_tool_l;
            obj.v_lin_obj_l(:, loop)  = temp_v_lin_obj_l;
            obj.v_ang_obj_l(:, loop)  = temp_v_ang_obj_l;

            obj.v_lin_tool_r(:, loop) = temp_v_lin_tool_r;
            obj.v_ang_tool_r(:, loop) = temp_v_ang_tool_r;
            obj.v_lin_obj_r(:, loop)  = temp_v_lin_obj_r;
            obj.v_ang_obj_r(:, loop)  = temp_v_ang_obj_r;
            % =========================================================

            for i = 1:obj.n
                tasks = obj.action_mng.actions{i};
                for j = 1:length(tasks)
                    task = tasks{j};

                    if isempty(task.xdotbar), obj.xdotbar_task{i,j,loop} = 0; else, obj.xdotbar_task{i,j,loop} = task.xdotbar; end

                    val_A = task.A;
                    val_A_vec = 0;
                    if isempty(val_A)
                        obj.activation_task{i,j,loop} = 0;
                    else
                        if size(val_A, 1) > 1 && size(val_A, 2) > 1, val_A_vec = diag(val_A); else, val_A_vec = val_A; end
                        obj.activation_task{i,j,loop} = val_A_vec;
                    end

                    val_ap = 0;
                    if isempty(task.ap), obj.priority_task{i,j,loop} = 0; else, val_ap = task.ap; obj.priority_task{i,j,loop} = task.ap; end

                    if isempty(task.A) || isempty(task.ap), obj.product_task{i,j,loop} = 0; else, obj.product_task{i,j,loop} = val_A_vec * val_ap; end
                end
            end

            for k = 1:obj.total_tasks
                t_ptr = obj.action_mng.all_task_list{k};
                if isempty(t_ptr.A)
                    val_A_scalar = 0;
                elseif ismatrix(t_ptr.A) && ~isscalar(t_ptr.A)
                    val_A_scalar = max(diag(t_ptr.A));
                else
                    val_A_scalar = t_ptr.A;
                end

                if isempty(t_ptr.ap), val_ap = 0; else, val_ap = t_ptr.ap; end
                obj.global_activations(k, loop) = val_A_scalar * val_ap;
            end
        end

        function plotAll(obj)

            LW = 3;
            LW_GRID = 4.0;
            FS = 20;
            FS_Title = 23;

            t_plot = obj.t(1:obj.curr_loop);

            % --- DEFINIZIONE LIMITE ASSE X ---
            T_MAX = 20;
            % ---------------------------------

            figure(1);
            subplot(2,1,1);
            plot(t_plot, obj.ql(:, 1:obj.curr_loop), 'LineWidth', LW);
            title('Left Arm Joints (ql)', 'FontSize', FS_Title);
            grid on; set(gca, 'FontSize', FS);
            xlim([0, T_MAX]);

            subplot(2,1,2);
            plot(t_plot, obj.qr(:, 1:obj.curr_loop), 'LineWidth', LW);
            title('Right Arm Joints (qr)', 'FontSize', FS_Title);
            grid on; set(gca, 'FontSize', FS);
            sgtitle('Robot Joint Positions', 'FontSize', FS_Title+2, 'FontWeight', 'bold');
            xlim([0, T_MAX]);

            figure(2);
            subplot(2,1,1);
            plot(t_plot, obj.qdotl(:, 1:obj.curr_loop), 'LineWidth', LW);
            title('Left Arm Velocities (qdotl)', 'FontSize', FS_Title);
            grid on; set(gca, 'FontSize', FS);
            xlim([0, T_MAX]);

            subplot(2,1,2);
            plot(t_plot, obj.qdotr(:, 1:obj.curr_loop), 'LineWidth', LW);
            title('Right Arm Velocities (qdotr)', 'FontSize', FS_Title);
            grid on; set(gca, 'FontSize', FS);
            sgtitle('Robot Joint Velocities', 'FontSize', FS_Title+2, 'FontWeight', 'bold');
            xlim([0, T_MAX]);

            % --- NUOVA FIGURA 3: LEFT ARM TOOL VS OBJECT VELOCITIES ---
            figure(3); clf;
            sgtitle('Left Arm: Actual tool velocities vs Object desired velocities', 'FontSize', FS_Title+2, 'FontWeight', 'bold');

            % Subplot 1: Linear Velocities
            subplot(2,1,1); hold on;
            % Tool (Solid Lines)
            plot(t_plot, obj.v_lin_tool_l(1, 1:obj.curr_loop), 'r-', 'LineWidth', LW);
            plot(t_plot, obj.v_lin_tool_l(2, 1:obj.curr_loop), 'g-', 'LineWidth', LW);
            plot(t_plot, obj.v_lin_tool_l(3, 1:obj.curr_loop), 'b-', 'LineWidth', LW);
            % Object (Dashed Lines)
            plot(t_plot, obj.v_lin_obj_l(1, 1:obj.curr_loop), 'r--', 'LineWidth', LW);
            plot(t_plot, obj.v_lin_obj_l(2, 1:obj.curr_loop), 'g--', 'LineWidth', LW);
            plot(t_plot, obj.v_lin_obj_l(3, 1:obj.curr_loop), 'b--', 'LineWidth', LW);

            % --- Switch Lines ---
            for s = 1:length(obj.switch_times)
                xline(obj.switch_times(s), '--k', 'LineWidth', LW_GRID, 'Alpha', 0.6, 'HandleVisibility', 'off');
            end

            hold off; grid on; set(gca, 'FontSize', FS);
            ylabel('Linear Vel [m/s]', 'FontSize', FS);
            title('Linear velocities', 'FontSize', FS_Title);
            legend('Tx','Ty','Tz','Ox','Oy','Oz', 'Location', 'eastoutside');
            xlim([0, T_MAX]);

            % Subplot 2: Angular Velocities
            subplot(2,1,2); hold on;
            % Tool (Solid Lines)
            plot(t_plot, obj.v_ang_tool_l(1, 1:obj.curr_loop), 'r-', 'LineWidth', LW);
            plot(t_plot, obj.v_ang_tool_l(2, 1:obj.curr_loop), 'g-', 'LineWidth', LW);
            plot(t_plot, obj.v_ang_tool_l(3, 1:obj.curr_loop), 'b-', 'LineWidth', LW);
            % Object (Dashed Lines)
            plot(t_plot, obj.v_ang_obj_l(1, 1:obj.curr_loop), 'r--', 'LineWidth', LW);
            plot(t_plot, obj.v_ang_obj_l(2, 1:obj.curr_loop), 'g--', 'LineWidth', LW);
            plot(t_plot, obj.v_ang_obj_l(3, 1:obj.curr_loop), 'b--', 'LineWidth', LW);

            % --- Switch Lines ---
            for s = 1:length(obj.switch_times)
                xline(obj.switch_times(s), '--k', 'LineWidth', LW_GRID, 'Alpha', 0.6, 'HandleVisibility', 'off');
            end

            hold off; grid on; set(gca, 'FontSize', FS);
            ylabel('Angular Vel [rad/s]', 'FontSize', FS);
            xlabel('Time [s]', 'FontSize', FS);
            title('Angular velocities', 'FontSize', FS_Title);
            legend('Tx','Ty','Tz','Ox','Oy','Oz', 'Location', 'eastoutside');
            xlim([0, T_MAX]);

            % --- NUOVA FIGURA 4: RIGHT ARM TOOL VS OBJECT VELOCITIES ---
            figure(4); clf;
            sgtitle('Right Arm: Actual tool velocities vs Object desired velocities', 'FontSize', FS_Title+2, 'FontWeight', 'bold');

            % Subplot 1: Linear Velocities
            subplot(2,1,1); hold on;
            % Tool (Solid Lines)
            plot(t_plot, obj.v_lin_tool_r(1, 1:obj.curr_loop), 'r-', 'LineWidth', LW);
            plot(t_plot, obj.v_lin_tool_r(2, 1:obj.curr_loop), 'g-', 'LineWidth', LW);
            plot(t_plot, obj.v_lin_tool_r(3, 1:obj.curr_loop), 'b-', 'LineWidth', LW);
            % Object (Dashed Lines)
            plot(t_plot, obj.v_lin_obj_r(1, 1:obj.curr_loop), 'r--', 'LineWidth', LW);
            plot(t_plot, obj.v_lin_obj_r(2, 1:obj.curr_loop), 'g--', 'LineWidth', LW);
            plot(t_plot, obj.v_lin_obj_r(3, 1:obj.curr_loop), 'b--', 'LineWidth', LW);

            % --- Switch Lines ---
            for s = 1:length(obj.switch_times)
                xline(obj.switch_times(s), '--k', 'LineWidth', LW_GRID, 'Alpha', 0.6, 'HandleVisibility', 'off');
            end

            hold off; grid on; set(gca, 'FontSize', FS);
            ylabel('Linear Vel [m/s]', 'FontSize', FS);
            title('Linear velocities', 'FontSize', FS_Title);
            legend('Tx','Ty','Tz','Ox','Oy','Oz', 'Location', 'eastoutside');
            xlim([0, T_MAX]);

            % Subplot 2: Angular Velocities
            subplot(2,1,2); hold on;
            % Tool (Solid Lines)
            plot(t_plot, obj.v_ang_tool_r(1, 1:obj.curr_loop), 'r-', 'LineWidth', LW);
            plot(t_plot, obj.v_ang_tool_r(2, 1:obj.curr_loop), 'g-', 'LineWidth', LW);
            plot(t_plot, obj.v_ang_tool_r(3, 1:obj.curr_loop), 'b-', 'LineWidth', LW);
            % Object (Dashed Lines)
            plot(t_plot, obj.v_ang_obj_r(1, 1:obj.curr_loop), 'r--', 'LineWidth', LW);
            plot(t_plot, obj.v_ang_obj_r(2, 1:obj.curr_loop), 'g--', 'LineWidth', LW);
            plot(t_plot, obj.v_ang_obj_r(3, 1:obj.curr_loop), 'b--', 'LineWidth', LW);

            % --- Switch Lines ---
            for s = 1:length(obj.switch_times)
                xline(obj.switch_times(s), '--k', 'LineWidth', LW_GRID, 'Alpha', 0.6, 'HandleVisibility', 'off');
            end

            hold off; grid on; set(gca, 'FontSize', FS);
            ylabel('Angular Vel [rad/s]', 'FontSize', FS);
            xlabel('Time [s]', 'FontSize', FS);
            title('Angular velocities', 'FontSize', FS_Title);
            legend('Tx','Ty','Tz','Ox','Oy','Oz', 'Location', 'eastoutside');
            xlim([0, T_MAX]);

            figure(20); clf; hold on;
            for k = 1:obj.total_tasks
                plot(t_plot, obj.global_activations(k, 1:obj.curr_loop), 'LineWidth', LW);
            end

            y_limits = ylim;
            if y_limits(2) < 1.1, y_limits(2) = 1.1; end
            ylim([-0.1, y_limits(2)]);
            xlim([0, T_MAX]);

            for s = 1:length(obj.switch_times)
                xline(obj.switch_times(s), '--k', 'LineWidth', LW_GRID, 'Alpha', 0.6);
                text(obj.switch_times(s), y_limits(1) + 0.05, ['  ' obj.switch_names{s}], ...
                    'Rotation', 90, 'VerticalAlignment', 'bottom', ...
                    'HorizontalAlignment', 'left', 'FontSize', FS-4, ...
                    'FontWeight', 'bold', 'Interpreter', 'none');
            end

            hold off; grid on;
            xlabel('Time [s]', 'FontSize', FS);
            ylabel('Activation (A * ap)', 'FontSize', FS);
            title('Global Task Activations Timeline', 'FontSize', FS_Title);
            set(gca, 'FontSize', FS);

            if ~isempty(obj.action_mng.all_task_names)
                legend(obj.action_mng.all_task_names, 'Interpreter', 'none', 'Location', 'eastoutside', 'FontSize', FS);
            end

            for k = 1:obj.total_tasks

                task_name_str = string(obj.action_mng.all_task_names(k));
                target_task = obj.action_mng.all_task_list{k};

                act_idx = -1; tsk_idx = -1; found = false;
                for a = 1:obj.n
                    act_tasks = obj.action_mng.actions{a};
                    for t = 1:length(act_tasks)
                        if act_tasks{t} == target_task
                            act_idx = a; tsk_idx = t; found = true; break;
                        end
                    end
                    if found, break; end
                end

                if ~found
                    fprintf('Task %d skipped (not found).\n', k);
                    continue;
                end

                q_L_data = obj.ql(:, 1:obj.curr_loop);
                q_R_data = obj.qr(:, 1:obj.curr_loop);

                if startsWith(task_name_str, "L", 'IgnoreCase', true)
                    q_R_data = [];
                elseif startsWith(task_name_str, "R", 'IgnoreCase', true)
                    q_L_data = [];
                end

                metrics_config = {
                    {q_L_data, q_R_data, 'Rad', 'Joint Config (q)'}, ...
                    {obj.alt_l(1:obj.curr_loop), obj.alt_r(1:obj.curr_loop), 'm', 'Altitude (alt)'}, ...
                    {obj.tools_distance(1:obj.curr_loop), [], 'm', 'Tools Distance'}, ...
                    {obj.tools_orientation(1:obj.curr_loop), [], 'rad', 'Tools Orientation'}, ...
                    {obj.dist_l(1:obj.curr_loop), obj.dist_r(1:obj.curr_loop), 'm', 'Dist to Goal'}, ...
                    {obj.rot_l(:,1:obj.curr_loop), obj.rot_r(:,1:obj.curr_loop), 'rad/err', 'Rot to Goal'} ...
                    };

                raw_xdot = squeeze(obj.xdotbar_task(act_idx, tsk_idx, 1:obj.curr_loop));
                raw_A    = squeeze(obj.activation_task(act_idx, tsk_idx, 1:obj.curr_loop));
                raw_ap   = squeeze(obj.priority_task(act_idx, tsk_idx, 1:obj.curr_loop));
                raw_prod = squeeze(obj.product_task(act_idx, tsk_idx, 1:obj.curr_loop));

                data_xdot = obj.fillWithZeros(raw_xdot);
                data_A    = obj.fillWithZeros(raw_A);
                data_ap   = obj.fillWithZeros(raw_ap);
                data_prod = obj.fillWithZeros(raw_prod);

                f = figure(100 + k);
                clf;
                f.Name = sprintf("Task Analysis: %s", task_name_str);
                tg = uitabgroup(f);

                for m = 1:length(metrics_config)
                    conf = metrics_config{m};
                    data_L = conf{1};
                    data_R = conf{2};
                    y_lab  = conf{3};
                    t_tit  = conf{4};

                    tab = uitab(tg, 'Title', t_tit);

                    ax1 = subplot(3,1,1, 'Parent', tab);
                    plot(ax1, t_plot, data_xdot', 'LineWidth', LW);
                    ylabel(ax1, '$\dot{\bar{x}}$', 'Interpreter', 'latex', 'FontSize', FS+4);
                    grid(ax1, 'on'); set(ax1, 'FontSize', FS);
                    title(ax1, ['Ref Velocity - ' char(task_name_str)], 'Interpreter', 'none', 'FontSize', FS_Title);
                    for s = 1:length(obj.switch_times)
                        xline(ax1, obj.switch_times(s), ':k', 'Alpha', 0.5, 'LineWidth', LW_GRID, 'HandleVisibility', 'off');
                    end
                    xlim(ax1, [0, T_MAX]);

                    n_vel = size(data_xdot, 1);
                    vel_labels = arrayfun(@(x) sprintf('$\\dot{x}_{%d}$', x), 1:n_vel, 'UniformOutput', false);
                    legend(ax1, vel_labels, 'Interpreter', 'latex', 'Location', 'eastoutside', 'FontSize', FS);

                    ax2 = subplot(3,1,2, 'Parent', tab);
                    hold(ax2, 'on');

                    h_ap = plot(ax2, t_plot, data_ap', '-b', 'LineWidth', LW);
                    h_A = plot(ax2, t_plot, data_A', '--k', 'LineWidth', LW);
                    h_prod = plot(ax2, t_plot, data_prod', '-.r', 'LineWidth', LW);

                    for s = 1:length(obj.switch_times)
                        xline(ax2, obj.switch_times(s), ':k', 'Alpha', 0.5, 'LineWidth', LW_GRID, 'HandleVisibility', 'off');
                    end
                    hold(ax2, 'off'); grid(ax2, 'on'); set(ax2, 'FontSize', FS);
                    ylabel(ax2, 'Activations', 'FontSize', FS);
                    if ~isempty(h_ap) && ~isempty(h_A) && ~isempty(h_prod)
                        legend(ax2, [h_ap(1), h_A(1), h_prod(1)], ...
                            {'$a_p$', '$A$', '$Total$'}, ...
                            'Interpreter', 'latex', 'Location', 'eastoutside', 'FontSize', FS);
                    end
                    xlim(ax2, [0, T_MAX]);

                    ax3 = subplot(3,1,3, 'Parent', tab);
                    hold(ax3, 'on');

                    has_L = ~isempty(data_L);
                    has_R = ~isempty(data_R);

                    if m == 1
                        colors = lines(7);
                        legend_entries = [];
                        legend_labels = {};

                        if has_L
                            for j = 1:size(data_L, 1)
                                h = plot(ax3, t_plot, data_L(j,:), 'Color', colors(j,:), 'LineWidth', LW);
                                legend_entries(end+1) = h;
                                legend_labels{end+1} = sprintf('$q_{L,%d}$', j);
                            end
                        end

                        if has_R
                            line_style = '-';
                            if has_L, line_style = '--'; end
                            for j = 1:size(data_R, 1)
                                color_idx = mod(j-1, 7) + 1;
                                h = plot(ax3, t_plot, data_R(j,:), 'Color', colors(color_idx,:), ...
                                    'LineStyle', line_style, 'LineWidth', LW);
                                legend_entries(end+1) = h;
                                legend_labels{end+1} = sprintf('$q_{R,%d}$', j);
                            end
                        end

                        if ~isempty(legend_entries)
                            legend(ax3, legend_entries, legend_labels, 'Interpreter', 'latex', 'Location', 'eastoutside', 'FontSize', FS);
                        end
                    else

                        if has_L
                            if size(data_L,1) > 1, plot(ax3, t_plot, data_L', 'b', 'LineWidth', LW-1);
                            else, plot(ax3, t_plot, data_L, 'b', 'LineWidth', LW); end
                        end

                        if has_R
                            if size(data_R,1) > 1, plot(ax3, t_plot, data_R', 'r', 'LineWidth', LW-1);
                            else, plot(ax3, t_plot, data_R, 'r--', 'LineWidth', LW); end
                        end

                        if contains(t_tit, 'Tools Distance')
                            legend(ax3, 'Distance [m]', 'Location', 'eastoutside', 'FontSize', FS);
                        elseif contains(t_tit, 'Tools Orientation')
                            legend(ax3, 'Orientation [rad]', 'Location', 'eastoutside', 'FontSize', FS);
                        elseif has_L && has_R
                            legend(ax3, 'Left', 'Right', 'Location', 'eastoutside', 'FontSize', FS);
                        elseif has_L
                            legend(ax3, 'Left', 'Location', 'eastoutside', 'FontSize', FS);
                        elseif has_R
                            legend(ax3, 'Right', 'Location', 'eastoutside', 'FontSize', FS);
                        end
                    end

                    for s = 1:length(obj.switch_times)
                        xline(ax3, obj.switch_times(s), ':k', 'Alpha', 0.5, 'LineWidth', LW_GRID, 'HandleVisibility', 'off');
                    end

                    hold(ax3, 'off'); grid(ax3, 'on'); set(ax3, 'FontSize', FS);
                    ylabel(ax3, y_lab, 'FontSize', FS);
                    xlabel(ax3, 'Time [s]', 'FontSize', FS);
                    title(ax3, t_tit, 'FontSize', FS_Title);
                    xlim(ax3, [0, T_MAX]);

                    if strcmp(t_tit, 'Tools Orientation')
                        ylim(ax3, [0, 6.28]);
                    end
                end
            end
        end

        function data_mat = fillWithZeros(~, cell_data)
            dim = 1;
            for k = 1:length(cell_data)
                val = cell_data{k};
                if ~isempty(val) && ~isscalar(val)
                    dim = size(val, 1);
                    break;
                end
            end
            for k = 1:length(cell_data)
                if isempty(cell_data{k}) || (isscalar(cell_data{k}) && dim > 1)
                    cell_data{k} = zeros(dim, 1);
                end
            end
            data_mat = cell2mat(cell_data');
            if size(data_mat, 2) ~= length(cell_data)
                data_mat = data_mat';
            end
        end
    end
end