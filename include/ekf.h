/**************************************************************************
 * @author z.h
 * @date 2018.12.30
 * @usage:ekf.h 
 * @brief:@params[IN]:@return:
 **************************************************************************/
#ifndef EKF_H_EKF_H
#define EKF_H_EKF_H

#include <iostream>
#include "kf.h"
#include "Eigen/Dense"
#include <vector>
#include <string>
#include <fstream>


class ekf:public kf
{
public:

	/**
    * Constructor.
    */
	ekf();

	/**
	* Destructor
	*/
	~ekf();

	/**
	* Update Updates the state by using Extended Kalman Filter equations
	* @param z The measurement at k+1
	*/  
	void Update_ekf(const Eigen::VectorXd &z);

	/**
	* A helper method to calculate RMSE.
	*/
	Eigen::VectorXd CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth);

	/**
	* A helper method to calculate Jacobians.
	*/
	Eigen::MatrixXd CalculateJacobian(const VectorXd& x_state);

	/**
	* ProcessFusion the main part of ekf
	*/
	void ProcessFusion(const SensorType &sensor_type_);

private:

	// check whether the tracking toolbox was initiallized or not (first measurement)
	bool is_initialized_;

	//acceleration noise 
	float noise_ax, noise_ay;

	// previous timestamp
	long long previous_timestamp_;
    // present timestamp
	long long present_timestamp_;

	Eigen::MatrixXd R_lidar_;
	Eigen::MatrixXd R_radar_;
	Eigen::MatrixXd H_lidar_;
	Eigen::MatrixXd Hj_;

	enum SensorType{
	    LIDAR,
	    RADAR
    };

	Eigen::VectorXd raw_measurements_;

	
};


#endif /*EKF_H_EKF_H*/