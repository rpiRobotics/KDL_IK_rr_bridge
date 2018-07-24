clear;
%% Robot Parameters
[ex,ey,ez,n,P,~,H,type,dq_bounds,q_bounds] = robotParams();

%R_EE can be eye(3) for other robots
R_EE = [1.0000    0.0000    0.0000
        0.0000    0.984815757178173    0.173602777666664
        0.0000   -0.173602777666664    0.984815757178173];
    
%% Forward Kinematics
% q = [2,3,0,0,0,1,1]';
% q = ones(7,1);
q = rand(7,1);%assign a random joint angle set
%Fixed joints
jnt_idx = floor(8*rand(1));
flag_fixed = [0,0,0,0,0,0,0]';
if jnt_idx ~=0
    flag_fixed(jnt_idx) = 1; %assign a random fixed joint
end
q_fixed = zeros(size(H,2),1);
for i = 1:size(H,2)
    if(flag_fixed(i) == 1)
       q_fixed(i) = q(i); 
    end
end
[pos,R] = fwdKin_alljoints_sawyer(q,type,H,P,n,R_EE);
R0 = R(:,:,end);
pos0 = pos(:,end);
ikTgt = [R0;pos0'];
ikTgt0 = ikTgt;

%Initial guess
q_init_guess = zeros(size(H,2)-sum(flag_fixed==1),1)+0.02*rand(size(H,2)-sum(flag_fixed==1),1);

%% Call IK function
[ret, q_rslt] = getIK( P,H,R_EE,ikTgt,flag_fixed,q_fixed,q_init_guess );



%% Compare result to target
if(ret == 0)
    display('failed');
else
    display(q_rslt);
    [pos,R] = fwdKin_alljoints_sawyer(q_rslt,type,H,P,n,R_EE);
    ikRslt = [R(:,:,end);pos(:,end)'];
    Diff_Tgt_Rslt = ikTgt0 - ikRslt;
    display(Diff_Tgt_Rslt);
end