/**************************************************************************
 * @author z.h
 * @date 2018.12.30
 * @usage:ekf_node.cpp 
 * @brief:@params[IN]:@return:
 **************************************************************************/
#include <iostream>
#include <math>

#include <Eigen/Dense>
#include "ekf.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

int main(int argc, char *argv)
{

    kf *fusion = new ekf;

    vector<VectorXd> estimations;
    vector<VectorXd> ground_truth;

	string sensor_type;
    fusion -> ProcessFusion(sensor_type);
    fusion -> CalculateRMSE(estimations, ground_truth);

    delete kf;
	
	return 0;
}
