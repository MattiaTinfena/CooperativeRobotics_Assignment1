function main()
%Add path
addpath('./simulation_scripts');
addpath('./tools')
addpath('./icat')
addpath('./tasks')
clc;clear;close all;
%Simulation Parameters
dt = 0.005;
end_time = 15;

% Initialize Franka Emika Panda Model
model = load("panda.mat");

%Simulation Setup
real_robot = false;

%Initiliaze panda_arm() Class, specifying the base offset w.r.t World Frame
arm1=panda_arm(model,eye(4));
wTb2 = [-1 0 0 1.06;
    0 -1 0 -0.01;
    0 0 1 0;
    0 0 0 1];
arm2=panda_arm(model,wTb2);

%Initialize Bimanual Simulator Class
bm_sim=bimanual_sim(dt,arm1,arm2,end_time);

%Define Object Shape and origin Frame
obj_length = 0.10;
w_obj_pos = [0.5 0 0.59]';
w_obj_ori = rotation(0,0,0);

%Set goal frames for left and right arm, based on object frame
arm1.setGoal(w_obj_pos, w_obj_ori, w_obj_pos - [obj_length/2; 0; 0],arm1.wTt(1:3, 1:3) * rotation(0, deg2rad(30), 0));
arm2.setGoal(w_obj_pos, w_obj_ori, w_obj_pos + [obj_length/2; 0; 0],arm2.wTt(1:3, 1:3) * rotation(0, deg2rad(30), 0));

%Define Object goal frame (Cooperative Motion)
wTog=[rotation(0.0, 0.0, 0.0) [0.65, -0.35, 0.28]'; 0 0 0 1];
arm1.set_obj_goal(wTog);
arm2.set_obj_goal(wTog);

%Define Tasks, input values(Robot type(L,R,BM), Task Name)
left_tool_task=tool_task("L","LT",true);
right_tool_task=tool_task("R","RT",true);
left_minimun_altitude_task=minimum_altitude_task("L","LMA",true);
right_minimun_altitude_task=minimum_altitude_task("R","RMA",true);
left_joint_limit_task=joint_limit_task("L","LJL",true);
right_joint_limit_task=joint_limit_task("R","RJL",true);
bim_rigid_constraint_task = bimanual_rigid_constraint_task("R","BRC",false);
left_move_object_task = move_object_task("L", "LMO", true);
right_move_object_task = move_object_task("R", "RMO", true);
stp_joints_task = stop_joints_task("R","SJ",true);

task_list = {left_tool_task, right_tool_task, left_minimun_altitude_task, right_minimun_altitude_task, left_joint_limit_task, right_joint_limit_task, bim_rigid_constraint_task, left_move_object_task, right_move_object_task, stp_joints_task};
task_list_name = ["LTT", "RTT", "LMAT", "RMAT", "LJLT", "RJLT", "BRCT", "LMOT", "RMOT", "SJT"];

%Actions for each phase: go to phase, coop_motion phase, end_motion phase
move_to = ["LJLT", "RJLT", "LMAT", "RMAT", "LTT", "RTT"];
move_obj = ["BRCT", "LJLT", "RJLT", "LMAT", "RMAT", "LMOT", "RMOT"];
stop = ["LMAT", "RMAT", "SJT"];

%Load Action Manager Class and load actions
actionManager = ActionManager();
actionManager.setTaskList(task_list, task_list_name);
actionManager.addAction(move_to, "MT");
actionManager.addAction(move_obj, "MO");
actionManager.addAction(stop, "ST");
actionManager.setCurrentAction("MT", bm_sim.time);

initial_time = bm_sim.time;

%Initiliaze robot interface
robot_udp=UDP_interface(real_robot);

%Initialize logger
logger=SimulationLogger(ceil(end_time/dt)+1,bm_sim,actionManager);

%Main simulation Loop
for t = 0:dt:end_time
    % 1. Receive UDP packets - DO NOT EDIT
    [ql,qr]=robot_udp.udp_receive(t);
    if real_robot==true %Only in real setup, assign current robot configuration as initial configuratio
        bm_sim.left_arm.q=ql;
        bm_sim.right_arm.q=qr;
    end

    % 2. Update Full kinematics of the bimanual system
    bm_sim.update_full_kinematics();

    % 3. Compute control commands for current action
    [q_dot]=actionManager.computeICAT(bm_sim,bm_sim.time);

    % 4. Step the simulator (integrate velocities)
    bm_sim.sim(q_dot);

    % 5. Send updated state to Pybullet
    robot_udp.send(t,bm_sim)

    % 6. Lggging
    logger.update(bm_sim.time,bm_sim.loopCounter)

    bm_sim.time;
    % 7. Optional real-time slowdown
    SlowdownToRealtime(dt);

    % 8. Action switching
    goal_reached = norm(bm_sim.left_arm.rot_to_goal) < 0.01 && norm(bm_sim.left_arm.dist_to_goal) < 0.01;
    delta_time = bm_sim.time - initial_time;

    if actionManager.current_action == 1 && goal_reached
        actionManager.setCurrentAction("MO",  bm_sim.time);
        initial_time = bm_sim.time;

    elseif (actionManager.current_action == 2 && goal_reached) || (delta_time > 10)
        actionManager.setCurrentAction("ST",  bm_sim.time);
    end
end
% Display joint position and velocity, Display for a given action, a number of tasks
action=1;
tasks=[1, 2, 3, 4, 5, 6];
logger.plotAll(action,tasks);
