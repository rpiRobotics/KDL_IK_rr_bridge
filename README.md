# KDL_IK_rr_bridge
A MATLAB RR bridge for KDL's Inverse Kinematics Solver

Run KDL_IK_RR.m to host the service.
Run testIK_RR.m in another MATLAB instance to verify the connection.

KDL library built with vs2015
MEX built with TDM-GCC-64
In another environment you may need to rebuild KDL and the mex function from source.
KDL: http://www.orocos.org/orocos/source-code-0

After building KDL:
1) Place orocos-kdld.lib in the RR bridge root folder
2) Place KDL *.cpp files and header files in the RR bridge root folder.
3) Place utilities folder and Eigen folder in the RR bridge root folder as well.

4)Run the following:
mex ik_solver_kdl.cpp stdafx.cpp articulatedbodyinertia.cpp chain.cpp chaindynparam.cpp chainfksolverpos_recursive.cpp chainfksolvervel_recursive.cpp chainidsolver_recursive_newton_euler.cpp chainidsolver_vereshchagin.cpp chainiksolverpos_lma.cpp chainiksolverpos_nr.cpp chainiksolverpos_nr_jl.cpp chainiksolvervel_pinv.cpp chainiksolvervel_pinv_givens.cpp chainiksolvervel_pinv_nso.cpp chainiksolvervel_wdls.cpp chainjnttojacdotsolver.cpp chainjnttojacsolver.cpp frameacc.cpp frames.cpp frames_io.cpp framevel.cpp jacobian.cpp jntarray.cpp jntarrayacc.cpp jntarrayvel.cpp jntspaceinertiamatrix.cpp joint.cpp kinfam_io.cpp path.cpp path_circle.cpp path_composite.cpp path_cyclic_closed.cpp path_line.cpp path_point.cpp path_roundedcomposite.cpp rigidbodyinertia.cpp rotational_interpolation.cpp rotational_interpolation_sa.cpp rotationalinertia.cpp segment.cpp trajectory.cpp trajectory_composite.cpp trajectory_segment.cpp trajectory_stationary.cpp tree.cpp treefksolverpos_recursive.cpp treeiksolverpos_nr_jl.cpp treeiksolverpos_online.cpp treeiksolvervel_wdls.cpp treejnttojacsolver.cpp velocityprofile.cpp velocityprofile_dirac.cpp velocityprofile_rect.cpp velocityprofile_spline.cpp velocityprofile_trap.cpp velocityprofile_traphalf.cpp ./utilities/utility.cxx ./utilities/utility_io.cxx ./utilities/error_stack.cxx ./utilities/svd_eigen_HH.cpp ./utilities/svd_HH.cpp -I. orocos-kdld.lib

If no mex compiler is found, download TDM-GCC-64 and run the following in MATLAB:

setenv('MW_MINGW64_LOC','Your Compiler Path')

'Your Compiler Path': e.g. 'C:\TDM-GCC-64'



