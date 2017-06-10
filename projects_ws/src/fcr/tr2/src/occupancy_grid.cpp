#include <tr2/occupancy_grid.h>

OccupancyGrid::OccupancyGrid()
: robot_size_x_(0.508), robot_size_y_(0.498),
  actual_node_(-1)
{
}

OccupancyGrid::~OccupancyGrid()
{
    for(size_t i=0; i < cells_grid_.size(); i++)
        for(size_t j=0; j < cells_grid_[i].size(); j++)
        delete cells_grid_[i][j];
}

void OccupancyGrid::createCicGrid()
{
    // freopen("debug1.txt","w",stdout);

    cells_grid_.resize(TOPO_MAP_NODE_SIZE);
    mat_cells_grid_.resize(TOPO_MAP_NODE_SIZE);

    for(size_t k=0; k<cells_grid_.size(); k++)
    {
        double cell_center_x;
        double cell_center_y;
        size_t img_id_x, img_id_y;

        // std::cout << "node: " << k << "[" << nodesCenterX_[k] << "," << nodesCenterY_[k] << "] [" \
        // << nodesDispX_[k] << "," << nodesDispY_[k] << "]" << std::endl;

        img_id_x = img_id_y = 0;
        for(cell_center_x = nodesCenterX_[k] - nodesDispX_[k] + robot_size_x_/2.0 + 0.01;
            cell_center_x <= nodesCenterX_[k] + nodesDispX_[k] - robot_size_x_/2.0 - 0.01;
            cell_center_x += robot_size_x_)
        {

            img_id_y = 0;
            for(cell_center_y = nodesCenterY_[k] + nodesDispY_[k] - robot_size_y_/2.0 - 0.01;
                cell_center_y >= nodesCenterY_[k] - nodesDispY_[k] + robot_size_y_/2.0 + 0.01;
                cell_center_y -= robot_size_y_)
            {

                // std::cout << "\tcell: " << "[" << cell_center_x << "," << cell_center_y << "] [" \
                // << img_id_y << "," << img_id_x << "]" << std::endl;

                // seta tudo como indefinido
                cells_grid_[k].push_back(new Cell(cell_center_x, cell_center_y, 0.5, img_id_y, img_id_x));
                img_id_y++;
            }

            img_id_x++;
        }

        // ok
        // preenche de cinza
        mat_cells_grid_[k] = cv::Mat(img_id_y, img_id_x, CV_8UC1, cv::Scalar(100));
    }

    // fclose(stdout);
}

bool OccupancyGrid::isInsideNode(double actual_pos_x, double actual_pos_y)
{
    double cx = nodesCenterX_[actual_node_];
    double cy = nodesCenterY_[actual_node_];
    double dx = nodesDispX_[actual_node_];
    double dy = nodesDispY_[actual_node_];

    double inside_x = (actual_pos_x < cx + dx - 0.02) && (actual_pos_x > cx - dx + 0.02);
    double inside_y = (actual_pos_y < cy + dy - 0.02) && (actual_pos_y > cy - dy + 0.02);

    if( inside_x && inside_y) return true;

    return false;
}

double OccupancyGrid::distFromSensor(double pos_x, double pos_y)
{
    double orix = ori_x_ + SENSOR_DISP*COSDEGREE(ori_angle_);
    double oriy = ori_y_ + SENSOR_DISP*SENDEGREE(ori_angle_);

    return sqrt((orix - pos_x)*(orix - pos_x) + \
      (oriy - pos_y)*(oriy - pos_y));
}

void OccupancyGrid::setCellByPos(double pos_x, double pos_y, double value)
{

    for(size_t i = 0; i< cells_grid_[actual_node_].size(); i++)
    {
        double cx = cells_grid_[actual_node_][i]->getCenterX();
        double cy = cells_grid_[actual_node_][i]->getCenterY();
        double dx = robot_size_x_/2.0;
        double dy = robot_size_y_/2.0;

        double inside_x = (pos_x < cx + dx - robot_size_x_/32.0) && (pos_x > cx - dx + robot_size_x_/32.0);
        double inside_y = (pos_y < cy + dy - robot_size_y_/32.0) && (pos_y > cy - dy + robot_size_y_/32.0);

        if( inside_x && inside_y)
        {
            Cell* cell_aux = cells_grid_[actual_node_][i];

            // std::cout << "\t\t\t[" << cell_aux->getIdRow() << "," << cell_aux->getIdCol() << "] " << value*200 << std::endl;

            // se vai setar caminho ou a celula nao era conhecida ainda
            if(cell_aux->getValue() == 0.5 || cell_aux->getValue() == 1.0)
            {
                cell_aux->setValue(value);
                mat_cells_grid_[actual_node_].at<uchar>(cv::Point(cell_aux->getIdCol(), cell_aux->getIdRow())) = value * 200.0;
            }

            if(value == 5.0)
            {
                cell_aux->setValue(0.0);
                mat_cells_grid_[actual_node_].at<uchar>(cv::Point(cell_aux->getIdCol(), cell_aux->getIdRow())) = 255;
            }

            break;
        }
    }
}

void OccupancyGrid::updateResources()
{
    ori_x_ = pose_msg_.pose.pose.position.x;
    ori_y_ = pose_msg_.pose.pose.position.y;
    ori_angle_ = tf::getYaw(pose_msg_.pose.pose.orientation);
    ori_angle_ = RAD2DEGREE(ori_angle_);
}

void OccupancyGrid::UpdateMap()
{
    updateResources();

    double base_angle = scan_msg_.angle_min;
    double incr_angle = scan_msg_.angle_increment;
    bool first_cell = true;

    // freopen("debug2.txt","w",stdout);

    for(size_t sensor_data = 0; sensor_data < scan_msg_.ranges.size(); sensor_data+=4)
    {
        double actual_pos_x = ori_x_ + SENSOR_DISP*COSDEGREE(ori_angle_);
        double actual_pos_y = ori_y_ + SENSOR_DISP*SENDEGREE(ori_angle_);
        double sensor_angle = base_angle + sensor_data*incr_angle;
        sensor_angle = RAD2DEGREE(sensor_angle);
        double final_angle;
        final_angle = fmod (ori_angle_ + sensor_angle, 360.0);

        // std::cout << "\n\ns_data: " << sensor_data << "  s_dist: " <<  scan_msg_.ranges[sensor_data] \
        //           << "  s_angle: " << sensor_angle \
        //           << "  s_final: " << final_angle << std::endl;

        while(isInsideNode(actual_pos_x, actual_pos_y) &&
              (distFromSensor(actual_pos_x, actual_pos_y) < (scan_msg_.ranges[sensor_data] - 0.1)) &&
              (distFromSensor(actual_pos_x, actual_pos_y) <= 6.0))
        {
            // std::cout << "\tactual_x: " << actual_pos_x << "\t actual_y: " << actual_pos_y << std::endl;

            // seta sem obstaculo
            if(first_cell)
            {
                setCellByPos(actual_pos_x, actual_pos_y, 5.0);
                first_cell = false;
            }
            else
                setCellByPos(actual_pos_x, actual_pos_y, 1.0);

            actual_pos_x += INCR*COSDEGREE(final_angle);
            actual_pos_y += INCR*SENDEGREE(final_angle);
        }

        if((fabs(distFromSensor(actual_pos_x, actual_pos_y) - scan_msg_.ranges[sensor_data]) < INCR))
        {
            setCellByPos(actual_pos_x, actual_pos_y, 0.0);
            // actual_pos_x += robot_size_x_*COSDEGREE(final_angle);
            // actual_pos_y += robot_size_y_*SENDEGREE(final_angle);
            // setCellByPos(actual_pos_x, actual_pos_y, 0.0);
        }

        first_cell = true;
    }

    // fclose(stdout);
}

void OccupancyGrid::showImageNode()
{
    cv::Mat aux_mat;

    std::stringstream sstm;
    sstm << "occ_map_" << actual_node_;
    std::string window = sstm.str();

    size_t sz_cols = mat_cells_grid_[actual_node_].cols * 10;
    size_t sz_rows = mat_cells_grid_[actual_node_].rows * 10;

    // std::cout << "DEBUG2-> " << sz_cols << "," << sz_rows << std::endl;

    cv::resize(mat_cells_grid_[actual_node_] , aux_mat, cv::Size(sz_cols, sz_rows));
    cv::imshow(window, aux_mat);
}

void OccupancyGrid::saveGridsInText()
{
    std::ofstream grid_file;
    grid_file.open ("grid_map.txt");


    for(size_t node=0; node<cells_grid_.size(); node++)
    {
        grid_file << "\nNode: " << node << std::endl;

        for(size_t cell=0; cell<cells_grid_[node].size(); cell++)
        {
            Cell* cell_aux = cells_grid_[node][cell];

            grid_file << "\n\tCell" << std::endl;
            grid_file << "\t{" << std::endl;
            grid_file << "\t\tcenter_x_: " << cell_aux->getCenterX() << std::endl;
            grid_file << "\t\tcenter_y_: " << cell_aux->getCenterY() << std::endl;
            grid_file << "\t\tvalue_: " << cell_aux->getValue() << std::endl;
            grid_file << "\t\tid_row_: " << cell_aux->getIdRow() << std::endl;
            grid_file << "\t\tid_col_: " <<cell_aux->getIdCol() << std::endl;
            grid_file << "\t}" << std::endl;
        }
    }

    grid_file.close();
}
