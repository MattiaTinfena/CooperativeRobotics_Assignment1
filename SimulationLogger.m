classdef SimulationLogger < handle
    properties
        t               % time vector
        ql              % joint positions
        qdotl           % joint velocities
        qr              % joint positions
        qdotr           % joint velocities

        % Strutture dati: (n_actions x n_tasks x n_loops)
        % Salviamo tutto in celle per gestire dimensioni variabili
        xdotbar_task
        activation_task
        priority_task
        product_task

        robot           % robot model
        action_set      % set of actions
        n               % number of actions
        curr_loop       % contatore loop corrente
    end

    methods
        function obj = SimulationLogger(maxLoops, robotModel, action_set)
            obj.robot = robotModel;
            obj.action_set = action_set;
            obj.curr_loop = 0;

            % Inizializzazione vettori robot
            obj.t = zeros(1, maxLoops);
            obj.ql = zeros(7, maxLoops);
            obj.qdotl = zeros(7, maxLoops);
            obj.qr = zeros(7, maxLoops);
            obj.qdotr = zeros(7, maxLoops);

            obj.n = length(action_set.actions);

            % Trova il numero massimo di task per dimensionare le celle
            l = zeros(1, obj.n);
            for i = 1:obj.n
                l(i) = length(action_set.actions{i});
            end
            max_tasks = max(l);

            % Prealloca celle (inizialmente vuote)
            obj.xdotbar_task = cell(obj.n, max_tasks, maxLoops);
            obj.activation_task = cell(obj.n, max_tasks, maxLoops);
            obj.priority_task = cell(obj.n, max_tasks, maxLoops);
            obj.product_task = cell(obj.n, max_tasks, maxLoops);
        end

        function update(obj, t, loop)
            obj.curr_loop = loop;
            obj.t(loop) = t;

            % 1. Log Stato Robot
            obj.ql(:, loop) = obj.robot.left_arm.q;
            obj.qdotl(:, loop) = obj.robot.left_arm.qdot;
            obj.qr(:, loop) = obj.robot.right_arm.q;
            obj.qdotr(:, loop) = obj.robot.right_arm.qdot;

            % 2. Log Tasks - ITERA SU TUTTO SEMPRE
            % Poiché i Task sono handle, leggiamo il loro stato attuale.
            % Se l'ActionManager sta sfumando un task vecchio, noi lo vediamo qui.
            for i = 1:obj.n
                for j = 1:length(obj.action_set.actions{i})
                    task = obj.action_set.actions{i}{j};

                    % --- Reference Velocity (xdotbar) ---
                    if isempty(task.xdotbar)
                        obj.xdotbar_task{i,j,loop} = 0; % Valore placeholder
                    else
                        obj.xdotbar_task{i,j,loop} = task.xdotbar;
                    end

                    % --- Internal Activation (A) ---
                    val_A = task.A;
                    if isempty(val_A)
                        obj.activation_task{i,j,loop} = 0;
                        val_A_calc = 0;
                    else
                        % Se è una matrice (es. 6x6), prendi la diagonale
                        if size(val_A, 1) > 1 && size(val_A, 2) > 1
                            val_A_calc = diag(val_A);
                        else
                            val_A_calc = val_A;
                        end
                        obj.activation_task{i,j,loop} = val_A_calc;
                    end

                    % --- Priority Activation (ap) ---
                    if isempty(task.ap)
                        obj.priority_task{i,j,loop} = 0;
                        val_ap_calc = 0;
                    else
                        obj.priority_task{i,j,loop} = task.ap;
                        val_ap_calc = task.ap;
                    end

                    % --- Product (A * ap) ---
                    % Calcoliamo il prodotto per il plot combinato
                    obj.product_task{i,j,loop} = val_A_calc * val_ap_calc;
                end
            end
        end

        function plotAll(obj, action, task_indices)
            % Usa il tempo fino al loop corrente
            t_plot = obj.t(1:obj.curr_loop);

            % --- Plot Robot ---
            figure(1);
            subplot(2,1,1); plot(t_plot, obj.ql(:, 1:obj.curr_loop), 'LineWidth', 1.5);
            title("Left Arm Pos"); grid on;
            subplot(2,1,2); plot(t_plot, obj.qdotl(:, 1:obj.curr_loop), 'LineWidth', 1.5);
            title("Left Arm Vel"); grid on;
            sgtitle('Robot Motion');

            % --- Plot Tasks ---
            nt = length(task_indices);

            for i = 1:nt
                task_idx = task_indices(i);
                current_task = obj.action_set.actions{action}{task_idx};

                figure(10 + i);

                % Titolo
                titleString = sprintf('Act:%d | Tsk:%d | ID:%s | %s', ...
                    action, task_idx, string(current_task.ID), string(current_task.task_name));
                sgtitle(titleString, 'Interpreter', 'none', 'FontSize', 11, 'FontWeight', 'bold');

                % --- Recupero dati grezzi (Celle) ---
                raw_xdot = squeeze(obj.xdotbar_task(action, task_idx, 1:obj.curr_loop));
                raw_A    = squeeze(obj.activation_task(action, task_idx, 1:obj.curr_loop));
                raw_ap   = squeeze(obj.priority_task(action, task_idx, 1:obj.curr_loop));
                raw_prod = squeeze(obj.product_task(action, task_idx, 1:obj.curr_loop));

                % --- Conversione intelligente (riempie i buchi con vettori di zeri) ---
                % Questo è fondamentale: se un task è inattivo all'inizio, avremo degli '0' scalari.
                % Quando diventa attivo, avremo vettori 6x1. La funzione fillWithZeros uniforma tutto.
                data_xdot = obj.fillWithZeros(raw_xdot);
                data_A    = obj.fillWithZeros(raw_A);
                data_ap   = obj.fillWithZeros(raw_ap);
                data_prod = obj.fillWithZeros(raw_prod);

                % --- Plotting ---
                subplot(4,1,1);
                plot(t_plot, data_xdot', 'LineWidth', 1.5);
                ylabel('$\dot{\bar{x}}$', 'Interpreter', 'latex', 'FontSize', 12);
                grid on;
                % Legenda dinamica
                labels = arrayfun(@(x) sprintf('x_{%d}', x), 1:size(data_xdot,1), 'UniformOutput', false);
                legend(labels, 'Location', 'eastoutside', 'Box', 'off');

                subplot(4,1,2);
                plot(t_plot, data_A', 'LineWidth', 1.5);
                ylabel('$A$', 'Interpreter', 'latex', 'FontSize', 12);
                grid on; set(gca, 'XTickLabel', []);

                subplot(4,1,3);
                plot(t_plot, data_ap', 'LineWidth', 1.5);
                ylabel('$a_p$', 'Interpreter', 'latex', 'FontSize', 12);
                grid on; set(gca, 'XTickLabel', []);

                subplot(4,1,4);
                plot(t_plot, data_prod', 'LineWidth', 1.5);
                ylabel('$A \cdot a_p$', 'Interpreter', 'latex', 'FontSize', 12);
                xlabel('Time [s]');
                grid on;
            end
        end

        function data_mat = fillWithZeros(~, cell_data)
            % Funzione helper per rendere cell2mat robusto a dati misti (scalari 0 e vettori)

            % 1. Cerca la dimensione reale dei dati (se il task è stato attivo almeno una volta)
            dim = 1;
            for k = 1:length(cell_data)
                val = cell_data{k};
                if ~isempty(val) && ~isscalar(val)
                    dim = size(val, 1);
                    break;
                end
            end

            % 2. Sostituisce gli scalari (0 o []) con vettori di zeri della dimensione corretta
            % Esempio: trasforma 0 in [0;0;0;0;0;0] se il task è 6D
            for k = 1:length(cell_data)
                if isempty(cell_data{k}) || (isscalar(cell_data{k}) && dim > 1)
                    cell_data{k} = zeros(dim, 1);
                end
            end

            % 3. Converte la cell array in matrice
            data_mat = cell2mat(cell_data');

            % Check orientamento (deve essere Dimensione x Tempo)
            if size(data_mat, 2) ~= length(cell_data)
                data_mat = data_mat';
            end
        end
    end
end