function [ P_v,H_v,R_EE_v,ikTgt_v ] = ReshapeData( P,H,R_EE,ikTgt )
%RESHAPEDATA Summary of this function goes here
%   Detailed explanation goes here
len = numel(P);
P_v = reshape(P,len,1);
len = numel(H);
H_v = reshape(H,len,1);
R_EE_v = reshape(R_EE,9,1);
ikTgt_v = reshape(ikTgt,12,1);
end

