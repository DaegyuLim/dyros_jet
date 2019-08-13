#include "dyros_jet_controller/dyros_jet_model.h"
#include "dyros_jet_controller/walking_controller.h"
#include "cvxgen_6_8_0/cvxgen/solver.h"
#include <stdio.h>


namespace dyros_jet_controller
{

void WalkingController::FootstepUpdateJoystick()
{
    /*//////////////////////////////////////////////////////////////
      *
      * function for calculating foot step when using joystick
      * making 3 steps ahead from current step to make short matrix size
      * first version, only considering fowrad direction and fixed foot step length
      * foot_step_(current_step_number, 6) =0 means swing foot is left( support foot is right)
      * foot_step_(current_step_number, 0)  x foot step where the robot will step right after
      * foot_step matrix has 4 rows (previous step, current, +1, +2 steps)
      * total step number and current_step number will increase while walking command is on
      *///////////////////////////////////////////////////////////////////////////////////

       double dlength = 0.1;// step_length_x_;

       if(walking_tick_ == 0){
           walking_cmd_ = step_length_x_;
           foot_step_joy_.resize(5,7);
           foot_step_joy_support_frame_.resize(5,7);
           foot_step_joy_support_frame_offset_.resize(5,7);
           foot_step_joy_.setZero();
           foot_step_joy_support_frame_.setZero();
           foot_step_joy_support_frame_offset_.setZero();

       }

       if(current_step_num_==5){
           walking_cmd_=0;
           total_step_num_joy_ = current_step_num_ +4;
           cout<<"hello"<<endl;
       }
       unsigned int number_of_foot_step;
       number_of_foot_step = 5;
       int temp;
       temp = -1;
       int index =0;

       Eigen::Matrix<double, 5, 7> p_foot_step;
       p_foot_step.setZero();

       if(walking_tick_ ==0 || walking_tick_ == t_start_){
           if(walking_cmd_ == 1)
           {
               if(current_step_num_ <2){
                   for(int i=0;i<number_of_foot_step;i++){
                       temp *= -1;
                       foot_step_joy_(index,0) = dlength*(i+1);
                       foot_step_joy_(index,1) = -temp*0.127794;
                       foot_step_joy_(index,6) = 0.5+0.5*temp;

                       index++;
                   }
               }
               else {
                   p_foot_step = foot_step_joy_;
                   int current_step = (int)current_step_num_;
                   if(!(current_step%2==0.0))
                       temp *= -1;

                   for(int i=0;i<number_of_foot_step;i++){
                       temp*= -1;
    //                   foot_step_joy_(index,0) = p_foot_step(1,0) + dlength*i;
    //                   foot_step_joy_(index,1) = p_foot_step(index,0)*-1;
                       foot_step_joy_(index,0) = dlength * (current_step_num_ + i -1);
                       foot_step_joy_(index,1) = -temp*0.127794;
                       foot_step_joy_(index,6) = 0.5+0.5*temp;

                       index++;
                   }
               }
           }
           else {
                foot_step_joy_ = foot_step_joy_;
           }
           for(int i=0;i<number_of_foot_step;i++){
               file[28]<<walking_tick_ + 30<<"\t"<<current_step_num_<<"\t"<<i<<"\t"<<foot_step_joy_(i,0)<<"\t"<<foot_step_joy_(i,1)<<"\t"<<foot_step_joy_(i,2)<<"\t"<<foot_step_joy_(i,3)<<"\t"<<foot_step_joy_(i,4)<<"\t"<<foot_step_joy_(i,5)<<"\t"<<foot_step_joy_(i,6)<<endl;
           }
       }
}
void WalkingController:: getZmpTrajectory_joystick()
{

  unsigned int planning_step_number  = 3;

  unsigned int norm_size = 0;

  //if(current_step_num_ >= total_step_num_ - planning_step_number)
  if(walking_cmd_==0){
    norm_size = (t_last_-t_start_+1)*(total_step_num_joy_-current_step_num_)+20*hz_;
    cout<<"hello1"<<endl;
  }
  else
    norm_size = (t_last_-t_start_+1)*(planning_step_number);
  if(current_step_num_ == 0)
    norm_size = norm_size + t_temp_+1;
  addZmpOffset();
  zmpGenerator_joystick(norm_size, planning_step_number);
}
void WalkingController::zmpGenerator_joystick(const unsigned int norm_size, const unsigned planning_step_num)
{

//    Eigen::Vector2d zmp_final;

//    zmp_final(0) = ref_zmp_joy_((int)t_total_,0);
//    zmp_final(1) = ref_zmp_joy_((int)t_total_,1);

  //ref_zmp_.resize(norm_size, 2);
  ref_zmp_joy_.resize(norm_size,2);

  com_offset_.setZero();

  Eigen::VectorXd temp_px;
  Eigen::VectorXd temp_py;

  unsigned int index =0;


  if(current_step_num_ ==0)
  {
    for (int i=0; i<= t_temp_; i++) //200 tick
    {
      if(i <= 0.5*hz_)
      {
        ref_zmp_joy_(i,0) = com_support_init_(0)+com_offset_(0);
        ref_zmp_joy_(i,1) = com_support_init_(1)+com_offset_(1);
      }
      else if(i < 1.5*hz_)
      {
        double del_x = i-0.5*hz_;
        ref_zmp_joy_(i,0) = com_support_init_(0)+com_offset_(0)-del_x*(com_support_init_(0)+com_offset_(0))/(1.0*hz_);
        ref_zmp_joy_(i,1) = com_support_init_(1)+com_offset_(1);
      }
      else
      {
        ref_zmp_joy_(i,0) = 0.0;
        ref_zmp_joy_(i,1) = com_support_init_(1)+com_offset_(1);
      }
      index++;
    }
  }
  if(walking_cmd_==0)// walking cmd off
  {
      if(current_step_num_<total_step_num_joy_){
          cout<<"hello2"<<endl;
            for(unsigned int i = current_step_num_; i<total_step_num_joy_ ; i++)
            {
                cout<<"hello3 "<<endl;
             // zmpPattern(i,temp_px1,temp_py);
              onestepZmp(i,temp_px,temp_py);
             // onestepZmp_modified(i,temp_px,temp_py);

              for (unsigned int j=0; j<t_total_; j++)
              {
                ref_zmp_joy_(index+j,0) = temp_px(j);
                ref_zmp_joy_(index+j,1) = temp_py(j);

              }
              index = index+t_total_;
            }

            for (unsigned int j=0; j<20*hz_; j++)
            {
              ref_zmp_joy_(index+j,0) = ref_zmp_joy_(index-1,0);
              ref_zmp_joy_(index+j,1) = ref_zmp_joy_(index-1,1);
            }
            final_ref_zmp_joy_(0) = ref_zmp_joy_(index,0);
            final_ref_zmp_joy_(1) = ref_zmp_joy_(index,1);
            index = index+20*hz_;
      }
      else {
          cout<<"hello4"<<endl;
            for(unsigned int j=0;j<norm_size;j++){
              ref_zmp_joy_(j,0) = final_ref_zmp_joy_(0);
              ref_zmp_joy_(j,1) = final_ref_zmp_joy_(1);
          }
      }
  }
  else
  {
    for(unsigned int i=current_step_num_; i < current_step_num_+planning_step_num; i++)
    {
    //  zmpPattern(i,temp_px1,temp_py);
      onestepZmp(i,temp_px,temp_py);
      //onestepZmp_modified(i,temp_px,temp_py);
      for (unsigned int j=0; j<t_total_; j++)
      {
        ref_zmp_joy_(index+j,0) = temp_px(j);
        ref_zmp_joy_(index+j,1) = temp_py(j);

      }
      index = index+t_total_;
     }
    } //for if(current_step_number_>!!@!@!#~~~~~~~~!
  if(walking_tick_==t_start_){
      file[10]<<walking_tick_<<"\t"<<current_step_num_;
      //file[14]<<walking_tick_;
      for(int i=0;i<norm_size;i++){
          file[10]<<"\t"<<ref_zmp_joy_(i,1);
          //file[14]<<"\t"<<ref_zmp1(i,0);
      }
      //file[14]<<endl;
      file[10]<<endl;
  }

}
}
