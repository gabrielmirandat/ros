#include "tr3/point_kinematics.h"

// Construtor
// Inicia variaveis
PointKinematics::PointKinematics()
: iterator_(0), linear_scape_vel_(0.0), angular_scape_vel_(0.0), turn_angular_vel_(0.0),
  destiny_yaw_(0.0), destiny_x_(0.0), destiny_y_(0.0), final_x_(0.0), final_y_(0.0),
  found_last_objective_(false), found_objective_(true), need_update_orientation_(true), first_turning_(true),
  ori_x_(0.0), ori_y_(0.0), ori_yaw_(0.0),
  new_linear_vel_(0.0), new_angular_vel_(0.0)
{
}

PointKinematics::~PointKinematics()
{
}


void PointKinematics::updateResources()
{
    ori_x_ = pose_msg_.pose.pose.position.x;
    ori_y_ = pose_msg_.pose.pose.position.y;

    // recebe orientacao em graus de 0 a 360
    ori_yaw_ = tf::getYaw(pose_msg_.pose.pose.orientation);
    ori_yaw_ = RAD2DEGREE(ori_yaw_);

    float disp_x = destiny_x_ - ori_x_;
    float disp_y = destiny_y_ - ori_y_;

    destiny_yaw_ = atan2(disp_y, disp_x);
    destiny_yaw_ = RAD2DEGREE(destiny_yaw_);

    new_linear_vel_ = desired_vel_.linear.x;
    new_angular_vel_ = desired_vel_.angular.z;
}

void PointKinematics::run()
{
    updateResources();

    new_linear_vel_ = linear_scape_vel_;

    // se encontrou um destino de no
    if(found_objective_)
    {
        updateDestinyPosAngle();
        found_objective_ = false;
        need_update_orientation_ = true;
        return;
    }

    // logica para checar orientacao
    // virar para a esquerda aumenta o grau
    if(need_update_orientation_)
    {
        new_linear_vel_ = 0.0;
        new_angular_vel_ = 0.0;

        std::cout << "ori_yaw: " << ori_yaw_ << " destiny_yaw: " << destiny_yaw_ << " diff: " << fabs(ori_yaw_ - destiny_yaw_) << std::endl;

        if(first_turning_)
        {
            // decide orientação de giro pelo menor caminho
            double turn_side = fmod(360.0 - ori_yaw_ + destiny_yaw_ , 360.0);

            if( turn_side >= 180.0)
            {
                new_angular_vel_ = -1.0 * angular_scape_vel_;
                //ROS_INFO("[PointKin] Girando pra direita!");
            }
            else
            {
                new_angular_vel_ = angular_scape_vel_;
                //ROS_INFO("[PointKin] Girando pra esquerda!");
            }

            turn_angular_vel_ = new_angular_vel_;
            first_turning_ = false;
        }
        else
        {
            new_angular_vel_ = turn_angular_vel_;


            // checa se orientaçao ja esta adequada
            if((fabs(ori_yaw_ - destiny_yaw_) < 4.0) || (fabs(ori_yaw_ - destiny_yaw_) > 356.0))
            {
                new_angular_vel_ = 0.0;
                need_update_orientation_ = false;
                ROS_INFO("[PointKin] Achou orientacao desejada!");
            }
        }

    }
    // se ja girou
    // segue em frente até estar proximo do objetivo
    else // if(!need_update_orientation_)
    {
        new_angular_vel_ = 0.0;
        //ROS_INFO("[PointKin] Sem atualizacao de angulo!");

        // chegou no objetivo
        if(sqrt((ori_x_ - destiny_x_)*(ori_x_ - destiny_x_) + \
          (ori_y_ - destiny_y_)*(ori_y_ - destiny_y_)) < 1.0)
        {
            new_linear_vel_ = 0.0;
            found_objective_ = true;
            ROS_INFO("[PointKin] Achou objetivo!");
        }

        first_turning_ = true;
    }
}

// atualiza os dados de destino
// como posiçoes em x,y e angulo desejado
void PointKinematics::updateDestinyPosAngle()
{

    if(iterator_ == path_x_.size()+1)
    {
        found_last_objective_ = true;
        return; // sai
    }

    if(iterator_ == path_x_.size())
    {
        destiny_x_ = final_x_;
        destiny_y_ = final_y_;
    }
    // ainda nao eh a ultima pos
    else
    {
        destiny_x_ = path_x_[iterator_];
        destiny_y_ = path_y_[iterator_];
    }

    float disp_x = destiny_x_ - ori_x_;
    float disp_y = destiny_y_ - ori_y_;

    destiny_yaw_ = atan2(disp_y, disp_x);
    destiny_yaw_ = RAD2DEGREE(destiny_yaw_);

    iterator_++;

    ROS_INFO("[PointKin] Atualizando novo destino!");
    ROS_INFO("[PointKin] atual: %f,%f,%f", ori_x_, ori_y_, ori_yaw_);
    ROS_INFO("[PointKin] destino: %f,%f,%f", destiny_x_, destiny_y_, destiny_yaw_);
}


void PointKinematics::cleanResources()
{
    path_x_.clear();
    path_y_.clear();

    iterator_ = 0;
    linear_scape_vel_ = angular_scape_vel_ = turn_angular_vel_ = 0.0;
    destiny_yaw_ = destiny_x_ = destiny_y_ = final_x_ = final_y_ = 0.0;
    found_last_objective_ = false; found_objective_ = need_update_orientation_ = first_turning_ = true,
    ori_x_ = ori_y_ = ori_yaw_ = 0.0;
    new_linear_vel_ = new_angular_vel_ = 0.0;
}
