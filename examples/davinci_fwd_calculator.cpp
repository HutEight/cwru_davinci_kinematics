#include <ctime>
#include <iostream>
#include <tf/transform_listener.h>
#include <tf/LinearMath/Quaternion.h>
#include <Eigen/Geometry>
#include <sensor_msgs/JointState.h>

#include <cwru_davinci_kinematics/davinci_inv_kinematics.h>



void printEigenAffine(Eigen::Affine3d affine)
{
    std::cout << "Rotation: " << std::endl;
    std::cout << affine.linear() << std::endl;
    std::cout << "origin: " << affine.translation().transpose() << std::endl;
}

int main(int argc, char **argv)
{
  davinci_kinematics::Forward dvrk_forward;
  davinci_kinematics::Inverse dvrk_inverse;

  Eigen::Affine3d affine_wrist_wrt_base, affine_gripper_wrt_base;
  davinci_kinematics::Vectorq7x1 q_vec, q_vec_up, q_vec_bottom;

	q_vec(0) = -0.39041503772354585;
	q_vec(1) = -0.12879973887631638;
  q_vec(2) =  0.19864607218000002;
	q_vec(3) = -0.03837325965766796;
	q_vec(4) =  0.01871920046786534;
	q_vec(5) =  0.10986757708893162;
	q_vec(6) =  0.10976721244223565;

  affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);

  std::cout << "PSM1 RESULT: " << std::endl;
  printEigenAffine(affine_gripper_wrt_base);


	q_vec(0) = -0.1671585530625895;
	q_vec(1) = -0.1547017849364654;
  q_vec(2) =  0.19888329325;
	q_vec(3) = -0.00482286870014406;
	q_vec(4) = -0.013390376977013164;
	q_vec(5) = -0.013584667479919373;
	q_vec(6) = -0.06697106711505449;


  affine_gripper_wrt_base = dvrk_forward.fwd_kin_solve(q_vec);

  std::cout << "PSM1 RESULT: " << std::endl;
  printEigenAffine(affine_gripper_wrt_base);



  return 0;
}
