RobotRaconteur.StartLocalServer('KdlIkService');
RobotRaconteur.StartTcpServer(4343);

%Define a service definition
servicedef1=['service KdlIkService\n\n' ...
    'object KDLIK\n' ...    
    'function double[] getIK(double[] P_v, double[] H_v, double[] R_EE_v, double[] ikTgt_v,double[] flag_fixed, double[] q_fixed, double[] q_init_guess)\n'...
    'end object\n'];
%Use sprintf to convert the '\n' to newlines
servicedef=sprintf(servicedef1);

%Register the service definition
RobotRaconteur.RegisterServiceType(servicedef);

%Create the object
o=KdlIkServiceClass();

%Register the object as a service
RobotRaconteur.RegisterService('KdlIkService','KdlIkService.KDLIK',o);

%Use the following instead if you want authentication. "authdata"
%has the same format as the "PasswordFileUserAuthenticator"
%authdata=sprintf(['myusername 34819d7beeabb9260a5c854bc85b3e44 objectlock\n' ...
%'anotherusername 1910ea24600608b01b5efd7d4ea6a840 objectlock\n' ...
%'superuser f1bc69265be1165f64296bcb7ca180d5 objectlock,objectlockoverride\n']);
%RobotRaconteur.RegisterService('MatlabTestService','example.MatlabTestService.MatlabTestObject',o,authdata);

%%Runtime section.  Run this section repeatedly to execute requests
%to the service.

%Run the service.  Use Ctrl-C to exit
i=0;
while(1)
  %We need to process the incoming requests manually because MATLAB
  %has poor threading capabilities.  It will timeout in 1 second.
  RobotRaconteur.ProcessServerRequests(1); 
  i=i+1;
end