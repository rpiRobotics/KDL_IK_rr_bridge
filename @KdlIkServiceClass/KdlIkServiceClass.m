%This is an example of an object that can be used as a service object in
%MATLAB.  Note that MATLAB service objects are limited to functions,
%properties, and events.  The class MUST inherent from "handle".  This
%class is a stub.  See the MATLAB help documentation for more information
%on how to use the "classdef" command.

classdef KdlIkServiceClass < handle
    properties
       
    end
    
    methods
        function [rslt] = getIK(c,P_v,H_v,R_EE_v,ikTgt_v,flag_fixed,q_fixed,q_init_guess )
            %% Convert Inputs
            sz1 = 3;
            sz2 = numel(P_v)/sz1;            
            P = reshape(P_v,sz1,sz2);
            sz1 = 3;
            sz2 = numel(H_v)/sz1;            
            H = reshape(H_v,sz1,sz2);       
            R_EE = reshape(R_EE_v,3,3);
            ikTgt = reshape(ikTgt_v,4,3);           
            %% Compute IK
            %ikTgt is the target orientation and position of the end effector.
            %P contains joint positions in zero configuration, base position is included.
            %H contains rotation axis assignments.
            %R_EE it a special end-effector rotation.
            %Fixed joints
            n = size(H,2);
            type = zeros(n,1);
            [pos,R] = fwdKin_alljoints_sawyer(q_fixed,type,H,P,n,eye(3));
            P_solve = [[0,0,0]',pos];
            P_solve = pos-P_solve(:,1:end-1);
            H_solve = H;
            for i = 1:size(H,2)
                H_solve(:,i) = R(:,:,i)*H(:,i);
            end

            if(sum(flag_fixed)==1)
                R_modify = R(:,:,flag_fixed==1)';
                ikTgt(1:3,1:3) = ikTgt(1:3,1:3)*R_EE'*R_modify*R_EE;%ikTgt(1:3,1:3)*R_EE'*(R(:,:,6)'*R(:,:,7))'*R_EE;
            end
            q_fixed = [flag_fixed,q_fixed];
            % q_init_guess = [0,0,0,0,0,0]';

            %Solve IK
            % tic
            [ret, q_rslt] = ik_solver_kdl(ikTgt,P_solve,H_solve,R_EE,q_fixed,q_init_guess);
            % toc
            for i = 1:size(H,2)
                if(flag_fixed(i) == 1)
                    if i == 1
                        q_rslt = [q_fixed(i,2);q_rslt];
                    else
                        q_rslt = [q_rslt(1:i-1);q_fixed(i,2);q_rslt(i:end)];
                    end
                end
            end

            rslt = [ret; q_rslt];

        end
    end
    
    events
       
    end
    
end