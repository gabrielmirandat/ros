#include "tr3/trabalho.h"

// -- public

Trabalho::Trabalho(ros::NodeHandle nh)
: nh_(nh),
  choosen_linear_vel_(0.0), choosen_angular_vel_(0.0),
  initial_pos_x_(0.0), initial_pos_y_(0.0),
  final_pos_x_(0.0), final_pos_y_(0.0),
  timer_force_corretion_counter_(0), obstacle_force_corretion_counter_(0),
  obstacle_can_count_(false), current_robot_node_(-1), last_robot_node_(-1)
{
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    dsr_sub_ = nh_.subscribe("desired_vel", 10, &Trabalho::dsrCallback, this);
    laser_sub_ = nh_.subscribe("hokuyo_scan", 10, &Trabalho::laserCallback, this);
    odom_sub_ = nh_.subscribe("pose", 10, &Trabalho::odomCallback, this);

    graph_ = new Graph();
    obs_avoid_ = new ObstacleAvoidance();
    occ_grid_ = new OccupancyGrid();
    point_kin_ = new PointKinematics();
    topo_map_ = new TopologicalMap();
}

Trabalho::~Trabalho()
{
    delete graph_;
    delete obs_avoid_;
    delete occ_grid_;
    delete point_kin_;
    delete topo_map_;
}

void Trabalho::dsrCallback(const geometry_msgs::Twist::ConstPtr& desired_vel)
{
    this->desired_vel_ = *desired_vel;
    obs_avoid_->dsrCallback(desired_vel);
    point_kin_->dsrCallback(desired_vel);
}

void Trabalho::laserCallback(const sensor_msgs::LaserScan::ConstPtr& laser_msg)
{
    obs_avoid_->laserCallback(laser_msg);
    occ_grid_->laserCallback(laser_msg);
}

void Trabalho::odomCallback(const nav_msgs::Odometry::ConstPtr& pose_msg)
{
    this->pose_msg_ = *pose_msg;
    occ_grid_->odomCallback(pose_msg);
    point_kin_->odomCallback(pose_msg);
}


void Trabalho::cleanResources()
{
    initial_pos_x_ = initial_pos_y_ = 0.0;
    final_pos_x_ = final_pos_y_ = 0.0;
    timer_force_corretion_counter_ = obstacle_force_corretion_counter_ = 0;
    obstacle_can_count_ = false;
    current_robot_node_ = last_robot_node_ = -1;
}







// rode teleop antes
void Trabalho::spinTrab1()
{
    ros::Rate loop_rate(50);
    while(ros::ok())
    {
        // atualiza callbacks
        ros::spinOnce();

        // algorithm();
        vel_pub_.publish(command_vel_);
        loop_rate.sleep();
    }

    command_vel_.linear.x = 0.0;
    command_vel_.angular.z = 0.0;
    vel_pub_.publish(command_vel_);
}







void Trabalho::spinNav()
{
    bool running; running = true;


    // cria mapa topologico do cic
    topo_map_->createCicMap();
    graph_->createCicGraph();

    while(running)
    {
        // atualiza callbacks
        ros::spinOnce();
        ros::spinOnce();
        ros::spinOnce();

        // pede por velocidade
        // e posicao de destino
        std::cout << "[double] Informe a vel linear x: ";               std::cin >> choosen_linear_vel_;
        std::cout << "[double] Informe a vel angular z: ";              std::cin >> choosen_angular_vel_;
        std::cout << "[double double] Informe a posição de destino: ";  std::cin >> final_pos_x_; std::cin >> final_pos_y_;

        // calcula no atual e final
        initial_pos_x_ = pose_msg_.pose.pose.position.x;
        initial_pos_y_ = pose_msg_.pose.pose.position.y;

        // busca e obtem o menor caminho
        graph_->dijkstra(topo_map_->nodeGivenPosition(final_pos_x_, final_pos_y_),
                         topo_map_->nodeGivenPosition(initial_pos_x_, initial_pos_y_));

        point_kin_->setDestinyVectorPositionsX( topo_map_->nodesCenterXByIdVector( graph_->getPath()));
        point_kin_->setDestinyVectorPositionsY( topo_map_->nodesCenterYByIdVector( graph_->getPath()));
        point_kin_->setFinalX(final_pos_x_);
        point_kin_->setFinalY(final_pos_y_);
        point_kin_->setLinVelScape(choosen_linear_vel_);
        point_kin_->setAngVelScape(choosen_angular_vel_);

        ros::Rate loop_rate(100);
        while(ros::ok)
        {
            // atualiza callbacks
            ros::spinOnce();

            if(point_kin_->getFoundLastObjective())
            {
                running = false;
                break; // acabou
            }

            point_kin_->run();

           command_vel_.linear.x = point_kin_->getLinearVel();
           command_vel_.angular.z = point_kin_->getAngularVel();

           vel_pub_.publish(command_vel_);


           // espera ate completar o rate esperado
           loop_rate.sleep();
        }

        // para
        command_vel_.linear.x = 0.0;
        command_vel_.angular.z = 0.0;
        vel_pub_.publish(command_vel_);
    }
    // para
    command_vel_.linear.x = 0.0;
    command_vel_.angular.z = 0.0;
    vel_pub_.publish(command_vel_);
    vel_pub_.publish(command_vel_);
    vel_pub_.publish(command_vel_);
}







void Trabalho::spinTrab2()
{
    bool running; running = true;
    char go_again; go_again = 's';

    // cria mapa topologico do cic
    topo_map_->createCicMap();
    graph_->createCicGraph();

    while(running)
    {

        point_kin_->cleanResources();
        cleanResources();

        // atualiza callbacks
        ros::spinOnce();

        // pede por velocidade
        // e posicao de destino
        std::cout << "[double] Informe a vel linear x: ";               std::cin >> choosen_linear_vel_;
        std::cout << "[double] Informe a vel angular z: ";              std::cin >> choosen_angular_vel_;
        std::cout << "[double double] Informe a posição de destino: ";  std::cin >> final_pos_x_; std::cin >> final_pos_y_;

        // calcula no atual e final
        initial_pos_x_ = pose_msg_.pose.pose.position.x;
        initial_pos_y_ = pose_msg_.pose.pose.position.y;

        // busca e obtem o menor caminho
        graph_->dijkstra(topo_map_->nodeGivenPosition(final_pos_x_, final_pos_y_),
                         topo_map_->nodeGivenPosition(initial_pos_x_, initial_pos_y_));

        point_kin_->setDestinyVectorPositionsX( topo_map_->nodesCenterXByIdVector( graph_->getPath()));
        point_kin_->setDestinyVectorPositionsY( topo_map_->nodesCenterYByIdVector( graph_->getPath()));
        point_kin_->setFinalX(final_pos_x_);
        point_kin_->setFinalY(final_pos_y_);

        point_kin_->setLinVelScape(choosen_linear_vel_);
        obs_avoid_->setLinVelScape(choosen_linear_vel_);
        point_kin_->setAngVelScape(choosen_angular_vel_);
        obs_avoid_->setAngVelScape(choosen_angular_vel_);

        ros::Rate loop_rate(50);
        while(running)
        {
            // atualiza callbacks
            ros::spinOnce();

            current_robot_node_ = topo_map_->nodeGivenPosition(pose_msg_.pose.pose.position.x, pose_msg_.pose.pose.position.y);
            if(current_robot_node_ != last_robot_node_)
            {
                ROS_INFO("[ROBOT] Estou em %c!", (char) current_robot_node_ + 65);
                last_robot_node_ = current_robot_node_;
            }

            if(point_kin_->getFoundLastObjective())
            {
                break; // acabou objetivo final
            }

            point_kin_->run();
            obs_avoid_->run();

            if((point_kin_->getNeedUpdateOrientation()))
            {
                command_vel_.linear.x = point_kin_->getLinearVel();
                command_vel_.angular.z = point_kin_->getAngularVel();

                obstacle_can_count_ = false;
                timer_force_corretion_counter_ = 500; //10 segundos


                //ROS_INFO("[Trab] Kinematics ");
            }
            else if(obs_avoid_->getCenterObstruction() || obs_avoid_->getLateralObstruction() || obs_avoid_->getSideObstruction())
            {
                command_vel_.linear.x = obs_avoid_->getLinearVel();
                command_vel_.angular.z = obs_avoid_->getAngularVel();

                obstacle_can_count_ = true;
                timer_force_corretion_counter_ = 500; //10 segundos
                obstacle_force_corretion_counter_ = 150; //3 segundos


                //ROS_INFO("[Trab] Obstacle ");
            }
            else
            {
                if(timer_force_corretion_counter_ == 0 || obstacle_force_corretion_counter_ == 0)
                {
                    point_kin_->setNeedUpdateOrientation(true);

                    ROS_INFO("[Trab] Counters ");
                    if(timer_force_corretion_counter_ == 0) ROS_INFO("[Trab] Timer de falta de evento chegou em ZERO! ");
                    if(obstacle_force_corretion_counter_ == 0) ROS_INFO("[Trab] Timer por ter havido obstaculo chegou em ZERO! ");

                    obstacle_can_count_ = false;
                    timer_force_corretion_counter_ = 500; //10 segundos
                    obstacle_force_corretion_counter_ = 150; //3 segundos
                }

                command_vel_.linear.x = choosen_linear_vel_;
                command_vel_.angular.z = 0.0;
            }


            vel_pub_.publish(command_vel_);

            // decrementa contador temporal
            timer_force_corretion_counter_--;
            if(obstacle_can_count_) obstacle_force_corretion_counter_--;

            // espera ate completar o rate esperado
            loop_rate.sleep();
        }

        // para
        command_vel_.linear.x = 0.0;
        command_vel_.angular.z = 0.0;
        vel_pub_.publish(command_vel_);

        std::cout << "\n\n[char] Gostaria de escolher um novo destino? <s/n>";
        std::cout << "\nSAIR DO PROGRAMA IMPLICA EM PERDER OS DADOS DA ODOMETRIA!\n";
        std::cin >> go_again;

        if(go_again == 's' || go_again == 'S')
            running = true;
        else
            running = false;
    }
} // fim







// RODE COM TELEOP ANTES APENAS A NAVEGAÇÃO
// MAS DEPENDE DO TOPOLOGICO
// COPIA A PRIMEIRA PARTE FORA DO LOOP
void Trabalho::spinMap()
{

    // atualiza callbacks
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();

    // cria mapa topologico do cic
    topo_map_->createCicMap();

    occ_grid_->setNodesCenterX(topo_map_->getAllNodesCenterX());
    occ_grid_->setNodesCenterY(topo_map_->getAllNodesCenterY());
    occ_grid_->setNodesDispX(topo_map_->getAllNodesDispX());
    occ_grid_->setNodesDispY(topo_map_->getAllNodesDispY());

    // cria os grids dos mapas topológicos do cic
    occ_grid_->createCicGrid();

    // cv::waitKey(0);

    ros::Rate loop_rate(10);
    while(ros::ok)
    {
        // atualiza callbacks
        ros::spinOnce();

        current_robot_node_ = topo_map_->nodeGivenPosition(pose_msg_.pose.pose.position.x, pose_msg_.pose.pose.position.y);
        if(current_robot_node_ != last_robot_node_)
        {
            ROS_INFO("[ROBOT] Estou em %c!", (char) current_robot_node_ + 65);
            last_robot_node_ = current_robot_node_;
            occ_grid_->setActualNode(current_robot_node_);
        }

        if(current_robot_node_ != -1)
        {
            occ_grid_->UpdateMap();
            occ_grid_->showImageNode();
        }

        command_vel_.linear.x = desired_vel_.linear.x;
        command_vel_.angular.z = desired_vel_.angular.z;

        vel_pub_.publish(command_vel_);

        char key = cv::waitKey(1);
        if(key==27) cv::waitKey(0);
    }

    // salva grids em arquivo
    occ_grid_->saveGridsInText();
}






void Trabalho::spinTrab3()
{
    // atualiza callbacks
    ros::spinOnce();
    ros::spinOnce();
    ros::spinOnce();

    // cria mapa topologico do cic
    topo_map_->createCicMap();

    occ_grid_->setNodesCenterX(topo_map_->getAllNodesCenterX());
    occ_grid_->setNodesCenterY(topo_map_->getAllNodesCenterY());
    occ_grid_->setNodesDispX(topo_map_->getAllNodesDispX());
    occ_grid_->setNodesDispY(topo_map_->getAllNodesDispY());

    // cria os grids dos mapas topológicos do cic
    occ_grid_->createCicGrid();

    // pede por velocidade
    // e posicao de destino
    std::cout << "[double] Informe a vel linear x: ";               std::cin >> choosen_linear_vel_;
    std::cout << "[double] Informe a vel angular z: ";              std::cin >> choosen_angular_vel_;

    // calcula no atual e final
    initial_pos_x_ = pose_msg_.pose.pose.position.x;
    initial_pos_y_ = pose_msg_.pose.pose.position.y;

    // caminho para percorrer todo o mapa
    int vv[25] = {H,E,C,B,A,D,F,G,H,J,M,L,K,I,F,G,H,J,M,O,R,Q,P,N,K};
    std::vector<int> path_map(&vv[0], &vv[0]+25);

    point_kin_->setDestinyVectorPositionsX( topo_map_->nodesCenterXByIdVector( path_map));
    point_kin_->setDestinyVectorPositionsY( topo_map_->nodesCenterYByIdVector( path_map));
    point_kin_->setFinalX(topo_map_->getNodePosX(K)-1.0);
    point_kin_->setFinalY(topo_map_->getNodePosY(K)-1.0);

    point_kin_->setLinVelScape(choosen_linear_vel_);
    obs_avoid_->setLinVelScape(choosen_linear_vel_);

    point_kin_->setAngVelScape(choosen_angular_vel_);
    obs_avoid_->setAngVelScape(choosen_angular_vel_);

    ros::Rate loop_rate(LOOP_RATE);
    while(ros::ok)
    {
        // atualiza callbacks
        ros::spinOnce();

        current_robot_node_ = topo_map_->nodeGivenPosition(pose_msg_.pose.pose.position.x, pose_msg_.pose.pose.position.y);
        if(current_robot_node_ != last_robot_node_)
        {
            ROS_INFO("[ROBOT] Estou em %c!", (char) current_robot_node_ + 65);
            last_robot_node_ = current_robot_node_;
            occ_grid_->setActualNode(current_robot_node_);
        }

        if(current_robot_node_ != -1)
        {
            occ_grid_->UpdateMap();
            occ_grid_->showImageNode();
            char key = cv::waitKey(1);
            if(key==27) cv::waitKey(0);
        }


        if(point_kin_->getFoundLastObjective())
        {
            break; // acabou objetivo final
        }

        point_kin_->run();
        obs_avoid_->run();

        if((point_kin_->getNeedUpdateOrientation()))
        {
            command_vel_.linear.x = point_kin_->getLinearVel();
            command_vel_.angular.z = point_kin_->getAngularVel();

            obstacle_can_count_ = false;
            timer_force_corretion_counter_ = TIMER_FORCE; //10 segundos


            //ROS_INFO("[Trab] Kinematics ");
        }
        else if(obs_avoid_->getCenterObstruction() || obs_avoid_->getLateralObstruction() || obs_avoid_->getSideObstruction())
        {
            command_vel_.linear.x = obs_avoid_->getLinearVel();
            command_vel_.angular.z = obs_avoid_->getAngularVel();

            obstacle_can_count_ = true;
            timer_force_corretion_counter_ = TIMER_FORCE; //10 segundos
            obstacle_force_corretion_counter_ = OBSTACLE_FORCE; //3 segundos


            //ROS_INFO("[Trab] Obstacle ");
        }
        else
        {
            if(timer_force_corretion_counter_ == 0 || obstacle_force_corretion_counter_ == 0)
            {
                point_kin_->setNeedUpdateOrientation(true);

                ROS_INFO("[Trab] Counters ");
                if(timer_force_corretion_counter_ == 0) ROS_INFO("[Trab] Timer de falta de evento chegou em ZERO! ");
                if(obstacle_force_corretion_counter_ == 0) ROS_INFO("[Trab] Timer por ter havido obstaculo chegou em ZERO! ");

                obstacle_can_count_ = false;
                timer_force_corretion_counter_ = TIMER_FORCE; //10 segundos
                obstacle_force_corretion_counter_ = OBSTACLE_FORCE; //3 segundos
            }

            command_vel_.linear.x = choosen_linear_vel_;
            command_vel_.angular.z = 0.0;
        }


        vel_pub_.publish(command_vel_);

        // decrementa contador temporal
        timer_force_corretion_counter_--;
        if(obstacle_can_count_) obstacle_force_corretion_counter_--;

        // espera ate completar o rate esperado
        loop_rate.sleep();
    }

    // para
    command_vel_.linear.x = 0.0;
    command_vel_.angular.z = 0.0;
    vel_pub_.publish(command_vel_);

    // salva grids em arquivo
    occ_grid_->saveGridsInText();


} // fim






//ADFIKNP
//B G L Q
//CEHJMOR

//H J M O R O M L K L
void Trabalho::spinTrab5()
{
    bool running; running = true;
    char go_again; go_again = 's';
    bool explore_flag = true;

    // cria mapa topologico do cic
    topo_map_->createCicMap();

    occ_grid_->setNodesCenterX(topo_map_->getAllNodesCenterX());
    occ_grid_->setNodesCenterY(topo_map_->getAllNodesCenterY());
    occ_grid_->setNodesDispX(topo_map_->getAllNodesDispX());
    occ_grid_->setNodesDispY(topo_map_->getAllNodesDispY());

    // cria os grids dos mapas topológicos do cic
    occ_grid_->createCicGrid();

    // cria grafo do cic
    graph_->createCicGraph();

    while(running)
    {

        point_kin_->cleanResources();
        cleanResources();

        // atualiza callbacks
        ros::spinOnce();

        if(explore_flag)
        {
            choosen_linear_vel_ = choosen_angular_vel_ = 0.0;

            while((choosen_linear_vel_ < 0.1 || choosen_linear_vel_ > 0.3) ||
                  (choosen_angular_vel_ < 0.1 || choosen_angular_vel_ > 0.3))
            {
                std::cout << "Deve ser maior que 0.1 e menor que 0.3" << std::endl;
                std::cout << "[double] Informe a vel linear  x: ";               std::cin >> choosen_linear_vel_;
                std::cout << "[double] Informe a vel angular z: ";               std::cin >> choosen_angular_vel_;

                // ROS_ASSERT(choosen_linear_vel_ > 0.1 && choosen_linear_vel_ < 0.5 &&
                //           choosen_angular_vel_ > 0.1 && choosen_angular_vel_ < 0.5);
            }


            final_pos_x_ = topo_map_->getNodePosX(L) - 1.0;
            final_pos_y_ = topo_map_->getNodePosY(L) - 1.0;
        }
        else
        {
            // posicao nao existente
            final_pos_x_ = final_pos_y_ = 1000.0;

            while(topo_map_->nodeGivenPosition(final_pos_x_, final_pos_y_) == -1)
            {
                std::cout << "Deve estar dentro de um no" << std::endl;
                std::cout << "[double double] Informe a posição de destino: ";  std::cin >> final_pos_x_; std::cin >> final_pos_y_;

                // ROS_ASSERT(topo_map_->nodeGivenPosition(final_pos_x_, final_pos_y_) != -1);
            }

        }

        // calcula no atual e final
        initial_pos_x_ = pose_msg_.pose.pose.position.x;
        initial_pos_y_ = pose_msg_.pose.pose.position.y;

        std::vector<int> path_map;
        if(explore_flag)
        {
            // caminho para percorrer nos desejados
            // H J M O R O M L K L
            int vv[10] = {H,J,M,O,R,O,M,L,K,L};
            for(size_t i=0; i<10; i++)
                path_map.push_back(vv[i]);
        }
        else
        {
            // busca e obtem o menor caminho
            graph_->dijkstra(topo_map_->nodeGivenPosition(final_pos_x_, final_pos_y_),
                             topo_map_->nodeGivenPosition(initial_pos_x_, initial_pos_y_));
            path_map.clear();
            path_map = graph_->getPath();
        }


        point_kin_->setDestinyVectorPositionsX( topo_map_->nodesCenterXByIdVector( path_map));
        point_kin_->setDestinyVectorPositionsY( topo_map_->nodesCenterYByIdVector( path_map));
        point_kin_->setFinalX(final_pos_x_);
        point_kin_->setFinalY(final_pos_y_);

        point_kin_->setLinVelScape(choosen_linear_vel_);
        obs_avoid_->setLinVelScape(choosen_linear_vel_);

        point_kin_->setAngVelScape(choosen_angular_vel_);
        obs_avoid_->setAngVelScape(choosen_angular_vel_);

        ros::Rate loop_rate(LOOP_RATE);
        while(running)
        {
            // atualiza callbacks
            ros::spinOnce();

            current_robot_node_ = topo_map_->nodeGivenPosition(pose_msg_.pose.pose.position.x, pose_msg_.pose.pose.position.y);
            if(current_robot_node_ != last_robot_node_)
            {
                ROS_INFO("[ROBOT] Estou em %c!", (char) current_robot_node_ + 65);
                last_robot_node_ = current_robot_node_;
                occ_grid_->setActualNode(current_robot_node_);

                // salva grids em arquivo
                ROS_INFO("[ROBOT] GRADES SALVAS!");
                occ_grid_->saveGridsInText();
            }

            if(current_robot_node_ != -1)
            {
                occ_grid_->UpdateMap();
                occ_grid_->showImageNode();
                char key = cv::waitKey(1);
                if(key==27) cv::waitKey(0);
            }

            if(point_kin_->getFoundLastObjective())
            {
                break; // acabou objetivo final
            }

            point_kin_->run();
            obs_avoid_->run();

            if( obs_avoid_->getVeryCloseFlag())
            {
                command_vel_.linear.x = obs_avoid_->getLinearVel();
                command_vel_.angular.z = obs_avoid_->getAngularVel();

                obstacle_can_count_ = true;
                timer_force_corretion_counter_ = TIMER_FORCE; //10 segundos
                obstacle_force_corretion_counter_ = OBSTACLE_FORCE; //3 segundos


                //ROS_INFO("[Trab] Very close Obstacle ");
            }
            else if((point_kin_->getNeedUpdateOrientation()))
            {
                command_vel_.linear.x = point_kin_->getLinearVel();
                command_vel_.angular.z = point_kin_->getAngularVel();

                obs_avoid_->setTurning(false);

                obstacle_can_count_ = false;
                timer_force_corretion_counter_ = TIMER_FORCE; //10 segundos


                //ROS_INFO("[Trab] Kinematics ");
            }
            else if( obs_avoid_->getCenterObstruction() || obs_avoid_->getLateralObstruction() || obs_avoid_->getSideObstruction())
            {
                command_vel_.linear.x = obs_avoid_->getLinearVel();
                command_vel_.angular.z = obs_avoid_->getAngularVel();

                obstacle_can_count_ = true;
                timer_force_corretion_counter_ = TIMER_FORCE; //10 segundos
                obstacle_force_corretion_counter_ = OBSTACLE_FORCE; //3 segundos


                //ROS_INFO("[Trab] Obstacle ");
            }
            else
            {
                if(timer_force_corretion_counter_ == 0 || obstacle_force_corretion_counter_ == 0)
                {
                    point_kin_->setNeedUpdateOrientation(true);

                    ROS_INFO("[Trab] Counters ");
                    if(timer_force_corretion_counter_ == 0) ROS_INFO("[Trab] Timer de falta de evento chegou em ZERO! ");
                    if(obstacle_force_corretion_counter_ == 0) ROS_INFO("[Trab] Timer por ter havido obstaculo chegou em ZERO! ");

                    obstacle_can_count_ = false;
                    timer_force_corretion_counter_ = TIMER_FORCE; //10 segundos
                    obstacle_force_corretion_counter_ = OBSTACLE_FORCE; //3 segundos
                }

                command_vel_.linear.x = choosen_linear_vel_;
                command_vel_.angular.z = 0.0;
            }


            vel_pub_.publish(command_vel_);

            // decrementa contador temporal
            timer_force_corretion_counter_--;
            if(obstacle_can_count_) obstacle_force_corretion_counter_--;

            // espera ate completar o rate esperado
            loop_rate.sleep();
        }

        // para
        command_vel_.linear.x = 0.0;
        command_vel_.angular.z = 0.0;
        vel_pub_.publish(command_vel_);

        if(explore_flag)
        {
            go_again = 's';
        }
        else
        {
            std::cout << "\n\n[char] Gostaria de escolher um novo destino? <s/n>";
            std::cin >> go_again;
        }

        if(go_again == 's' || go_again == 'S')
            running = true;
        else
            running = false;

        explore_flag = false;

        std::cout << "\nSAIR DO PROGRAMA IMPLICA EM PERDER OS DADOS DA ODOMETRIA!\n";
    }

    // salva grids em arquivo
    ROS_INFO("[ROBOT] GRADES SALVAS!");
    occ_grid_->saveGridsInText();
} // fim
