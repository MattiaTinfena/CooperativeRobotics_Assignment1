function main()
%Add path
addpath('./simulation_scripts');
addpath('./tools')
addpath('./icat')
addpath('./tasks')
clc;clear;close all; 
%Simulation Parameters
dt = 0.005;
end_time = 20;

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
obj_length = 0.12;
w_obj_pos = [0.5 0 0.59]';
w_obj_ori = rotation(0,0,0);

%Set goal frames for left and right arm, based on object frame
%TODO: Set arm goal frame based on object frame.
arm1.setGoal(w_obj_pos, w_obj_ori, w_obj_pos - [obj_length/2; 0; 0],arm1.wTt(1:3, 1:3) * rotation(0, deg2rad(30), 0));
arm2.setGoal(w_obj_pos, w_obj_ori, w_obj_pos + [obj_length/2; 0; 0],arm2.wTt(1:3, 1:3) * rotation(0, deg2rad(30), 0));

%Define Object goal frame (Cooperative Motion)
wTog=[rotation(0,0,0) [0.65, -0.35, 0.28]'; 0 0 0 1];
arm1.set_obj_goal(wTog)
arm2.set_obj_goal(wTog)

%Define Tasks, input values(Robot type(L,R,BM), Task Name)
left_tool_task=tool_task("L","LT");
right_tool_task=tool_task("R","RT");

task_list = {left_tool_task, right_tool_task};
task_list_name = ["LTT", "RTT"];


%Actions for each phase: go to phase, coop_motion phase, end_motion phase
go_to = ["LTT", "RTT"];

%Load Action Manager Class and load actions
actionManager = ActionManager();
actionManager.setTaskList(task_list, task_list_name);
actionManager.addAction(go_to, "GT");
actionManager.setCurrentAction("GT", bm_sim.time);

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
end
% Display joint position and velocity, Display for a given action, a number
% of tasks
action=1;
tasks=[1];
logger.plotAll(action,tasks);
end
