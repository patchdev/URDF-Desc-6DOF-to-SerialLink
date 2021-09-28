%------------------------------
%   NAME: Diego Pacheco Torres
%   INDEX: 9685
%   DATE: 2020-2021
%------------------------------


%****************************************
%   GAUSS 6 DOF ROBOT ARM
%****************************************
%   Solution based on DHfactor function
%****************************************
% Reminder to start the robotics toolbox
% Using RBT10
% Note: Distances are converted to mm
% Note: remember turnings has order Rz Ry Rx

%startup_rvc
clc;
clear all


%------------------------------
% Ground Joint URDF description
%------------------------------
% <joint name="ground_joint" type="fixed">
%         <parent link="ground_link" />
%         <child link="base_link" />
%         <origin xyz="0 0 0" rpy="0 0 0" />
%     </joint>
%------------------------------
% Type: Fixed, is base



%------------------------------
% First Joint URDF description
%------------------------------
%     <joint name="joint1" type="revolute">
%             <parent link="base_link" />
%             <child link="link1" />
%             <origin xyz="0 0 ${base_joint1_z_distance}" rpy="0 0 0" />
%             <axis xyz="0 0 1" />
%             <limit effort="1" velocity="1.0" lower="${joint1_lower_limit}" upper="${joint1_upper_limit}" />
%      </joint>
%     <xacro:property name="base_joint1_z_distance" value="0.070" />
%------------------------------
% Type: Revolute, has limits

  j1 = 'Tz(Lz1).Rz(q1)';
  Lz1 = 0.070;


%------------------------------
% Second Joint URDF description
%------------------------------
% <joint name="joint2" type="revolute">
%         <parent link="link1" />
%         <child link="link2" />
%         <origin xyz="0 0 ${joint1_joint2_z_distance}" rpy="${PI/2} 0 0" />
%         <axis xyz="0 0 1" />
%         <limit effort="1" velocity="1.0" lower="${joint2_lower_limit}" upper="${joint2_upper_limit}" />
%     </joint>
%     <xacro:property name="joint1_joint2_z_distance" value="0.1805" />
%------------------------------
% Type: Revolute, has limits

  j2 = 'Tz(Lz2).Rz(q2).Rx(90)';
  Lz2 = 0.1805;


%------------------------------
% Third Joint URDF description
%------------------------------
% <joint name="joint3" type="revolute">
%         <parent link="link2" />
%         <child link="link3" />
%         <origin xyz="0 ${joint2_joint3_y_distance} 0" rpy="0 0 0" />
%         <axis xyz="0 0 1" />
%         <limit effort="1" velocity="1.0" lower="${joint3_lower_limit}" upper="${joint3_upper_limit}" />
%     </joint>
%     <xacro:property name="joint2_joint3_y_distance" value="0.185" />
%------------------------------
% Type: Revolute, has limits

  j3 = 'Ty(Ly3).Rz(q3)';
  Ly3 = 0.185;


%------------------------------
% Fourth Joint URDF description
%------------------------------
% <joint name="joint4" type="revolute">
%         <parent link="link3" />
%         <child link="link4" />
%         <origin xyz="${joint3_joint4_x_distance} 0 0" rpy="0 ${PI/2} 0" />
%         <axis xyz="0 0 1" />
%         <limit effort="1" velocity="1.0" lower="${joint4_lower_limit}" upper="${joint4_upper_limit}" />
%     </joint>
%     <xacro:property name="joint3_joint4_x_distance" value="0.149" />
%------------------------------
% Type: Revolute, has limits

  j4 = 'Tx(Lx4).Rz(q4).Ry(90)';
  Lx4 = 0.149;


%------------------------------
% Fifth Joint URDF description
%------------------------------
% <joint name="joint5" type="revolute">
%         <parent link="link4" />
%         <child link="link5" />
%         <origin xyz="0 0 ${joint4_joint5_z_distance}" rpy="0 ${-PI/2} 0" />
%         <axis xyz="0 0 1" />
%         <limit effort="1" velocity="1.0" lower="${joint5_lower_limit}" upper="${joint5_upper_limit}" />
%     </joint>
%     <xacro:property name="joint4_joint5_z_distance" value="0.101" />
%------------------------------
% Type: Revolute, has limits

  j5 = 'Tz(Lz5).Rz(q5).Ry(-90)';
  Lz5 = 0.101;


%------------------------------
% Sixth Joint URDF description
%------------------------------
%     <joint name="joint6" type="revolute">
%         <parent link="link5" />
%         <child link="link6" />
%         <origin xyz="${joint5_joint6_x_distance} 0 0" rpy="0 ${PI/2} 0" />
%         <axis xyz="0 0 1" />
%         <limit effort="1" velocity="1.0" lower="${joint6_lower_limit}" upper="${joint6_upper_limit}" />
%     </joint>
%         <xacro:property name="joint5_joint6_x_distance" value="0.026" />
%------------------------------
% Type: Revolute, has limits

  j6 = 'Tx(Lx6).Rz(q6).Ry(90)';
  Lx6 = 0.026;


%------------------------------
% Flange Joint URDF description
%------------------------------
% <joint name="joint_flange" type="fixed">
%         <parent link="link6" />
%         <child link="flange" />
%         <origin xyz="0 0 0" rpy="0 ${-PI/2} ${-PI/2}" />
%     </joint>
%------------------------------
% Type: Fixed

  jflange = 'Rz(-90).Ry(-90)';


%------------------------------
% Tool Joint URDF description
%------------------------------
% <joint name="joint_tool" type="fixed">
%         <parent link="flange" />
%         <child link="tool0" />
%         <origin xyz="0 0 0" rpy="0 0 0" />
%     </joint>
%------------------------------
% Type: Fixed

  jtool = 'Tx(Lx)';
  Lx = 0.1;


%------------------------------
% Robot definition by DH
%------------------------------
s = strcat(j1,'.',j2,'.',j3,'.',j4,'.',j5,'.',j6,'.',jflange,'.',jtool);
dh = DHFactor(s);
RobotGauss = eval( dh.command('GAUSS') );


%------------------------------
% Set Kinematics parameters
%------------------------------
%Overwritting of Links default parameters to Kinematics limits
L = RobotGauss.links;

%   <limit effort="1" velocity="1.0" lower="${joint1_lower_limit}" upper="${joint1_upper_limit}" />
%   <xacro:property name="joint1_lower_limit" value="${-PI/2}" />
%   <xacro:property name="joint1_upper_limit" value="${PI/2}" />
  L(1).qlim = [-pi/2 pi/2];


%   <limit effort="1" velocity="1.0" lower="${joint2_lower_limit}" upper="${joint2_upper_limit}" />
%   <xacro:property name="joint2_lower_limit" value="${-PI/2}" />
%   <xacro:property name="joint2_upper_limit" value="${PI/6}" />
  L(2).qlim = [-pi/2 pi/6];

%   <limit effort="1" velocity="1.0" lower="${joint3_lower_limit}" upper="${joint3_upper_limit}" />
%   <xacro:property name="joint3_lower_limit" value="-0.29670579" />
%   <xacro:property name="joint3_upper_limit" value="${PI/2}" />
  L(3).qlim = [-0.29670579 pi/2];

%   <limit effort="1" velocity="1.0" lower="${joint4_lower_limit}" upper="${joint4_upper_limit}" />
%   <xacro:property name="joint4_lower_limit" value="${-PI*2/3}" />
%   <xacro:property name="joint4_upper_limit" value="${PI*2/3}" />
  L(4).qlim = [-pi*(2/3) pi*(2/3)];

%   <limit effort="1" velocity="1.0" lower="${joint5_lower_limit}" upper="${joint5_upper_limit}" />
%   <xacro:property name="joint5_lower_limit" value="-1.74532922" />
%   <xacro:property name="joint5_upper_limit" value="1.74532922" />
  L(5).qlim = [-1.74532922 1.74532922];

%   <limit effort="1" velocity="1.0" lower="${joint6_lower_limit}" upper="${joint6_upper_limit}" />
%   <xacro:property name="joint6_lower_limit" value="-2.53072737" />
%   <xacro:property name="joint6_upper_limit" value="2.53072737" />
  L(6).qlim = [-2.53072737 2.53072737];


%------------------------------
% Robot Plot
%------------------------------
RobotGauss
RobotGauss.teach();
%Plotting with a fixed position
%RobotGauss.plot([0 0 0 0 0 0]);
