/**************************************************************************
 * @author z.h
 * @date 2018.12.30
 * @usage:ekf.cpp 
 * @brief:@params[IN]:@return:
 **************************************************************************/
#include <iostream>
#include "ekf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
* Constructor.
*/
ekf::ekf()	
{
	// initializing matrices
	is_initialized_ = false;
	
	previous_timestamp_ = 0;

	// Set the noise components
	noise_ax = 9.f;
	noise_ay = 9.f;	

	//measurement covariance matrix - lidar
	R_lidar_ = MatrixXd(2, 2);
	R_lidar_ << 0.0225, 0,
	    		0, 0.0225;

	//measurement covariance matrix - radar
	R_radar_ = MatrixXd(3, 3);
	R_radar_ << 0.09, 0, 0,
	    	    0, 0.0009, 0,
	            0, 0, 0.09;

	//H matrix - lidar
	H_lidar_ = MatrixXd(2, 4);
	H_lidar_ << 1, 0, 0, 0,
	            0, 1, 0, 0;

    //Hj_ jacobi matrix - radar
	Hj_ = MatrixXd(3, 4);
	Hj_ << 1, 1, 0, 0,
	       1, 1, 0, 0,
	       1, 1, 1, 1;

	// Set the initial transition matrix F_
	F_ = MatrixXd(4,4);
	F_ << 1, 0, 1, 0,
	      0, 1, 0, 1,
	      0, 0, 1, 0,
	      0, 0, 0, 1;

	//process uncertainty matrix
	P_ = MatrixXd(4, 4);
	P_ << 1, 0, 0, 0,
	      0, 1, 0, 0,
	      0, 0, 1000, 0,
	      0, 0, 0, 1000;

	//state vector
	x_ = VectorXd(4);
	x_ << 1, 1, 1, 1;
}

/**
* Destructor.
*/
ekf::~ekf()	{}

void ekf::Update_ekf(const Eigen::VectorXd &z)
{
	/**
	TODO:
	* update the state by using Extended Kalman Filter equations
	*/
	double px = x_[0];
	double py = x_[1];
	double vx = x_[2];
	double vy = x_[3];

	double rho = sqrt(px*px + py*py );//range
	double phi  = 0;//bearing
	double rho_dot = 0;//v

	if (fabs(rho) > EPS) {
		phi = atan2(py , px);// return value between -pi and pi
		rho_dot = ((px * vx + py * vy) / rho);
	}

	VectorXd z_pred(3);
	z_pred << rho, phi, rho_dot;

	VectorXd y = z - z_pred;
	while (y(1) < -PI) {y(1) += 2 * PI;}
	while (y(1) > PI) {y(1) -= 2 * PI;}

	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	MatrixXd K = P_ * H_.transpose() * S.inverse();
	x_ = x_ + (K * y);
	long size = x_.size();
	MatrixXd I = MatrixXd::Identity(size,size);
	P_ = (I - K * H_) * P_;

}

VectorXd ekf::CalculateRMSE(const vector<VectorXd> &estimations, 
	                        const vector<VectorXd> &ground_truth)
{
	/**
	TODO:
	* Calculate the RMSE here.
	*/
	VectorXd rmse(4);
	rmse << 0,0,0,0;

	// estimation vector  len should be non-zero and equal ground truth vector len 
	if(estimations.size() != ground_truth.size()
				|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//calculate squared errors
	for(unsigned int i=0; i < estimations.size(); ++i){
			VectorXd errors = estimations[i] - ground_truth[i];
			errors = errors.array()*errors.array(); //coefficient-wise multiplication
			rmse += errors;
	}

	//calculate the mean square root
	rmse = rmse/estimations.size();
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;			

}

MatrixXd CalculateJacobian(const VectorXd& x_state)
{
	/**
	TODO:
	* Calculate a Jacobian here.
	*/

	MatrixXd Hj = MatrixXd::Zero(3, 4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	float c1 = px*px+py*py;

	//check division by zero
	if(fabs(c1) < EPS){
	cout << "c1 = " << c1 << endl;
	cout << "CalculateJacobian () - Error - Division by Zero" << endl;
	return Hj;
	}
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	//compute the Jacobian 
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;	
}


void ekf::ProcessFusion(const SensorType &sensor_type_ )
{
	/****************************************************************************
	*  Initialization                                                           *
	****************************************************************************/
	if (!is_initialized_) {
		/**
	    * Initialize the state ekf_.x_ with the first measurement.
	    * Create the covariance matrix.
	    * Remember: you'll need to convert radar from polar to cartesian coordinates.
		*/
		if (sensor_type_ == RADAR) {
			/**
			Convert radar from polar to cartesian coordinates and initialize state.
			*/
			double rho = raw_measurements_[0];
			double phi = raw_measurements_[1];
			double rho_dot = raw_measurements_[2];
			x_ << rho*cos(phi), rho*sin(phi), 0, 0;
		}
		else if (sensor_type_ == LIDAR) {
			/**
			Initialize state.
			*/
			x_ << raw_measurements_[0], raw_measurements_[1], 0, 0;
		}

		previous_timestamp_ = present_timestamp_;
		is_initialized_ = true;

		return;
	}

	/****************************************************************************
	*  Prediction                                                               *
	****************************************************************************/

	/**
	TODO:
	 * Update the state transition matrix F according to the new elapsed time.
	  - Time is measured in seconds.
	 * Update the process noise covariance matrix.
	 * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
	*/

	double dt = (present_timestamp_ - previous_timestamp_) / 1000000.0;	
	previous_timestamp_ = measurement_pack.timestamp_;

	//Include delta time into the F matrix 
	F_(0, 2) = dt;
	F_(1, 3) = dt;

	if (dt>0){
		float dt_2 = dt * dt;
	  	float dt_3 = dt_2 * dt;
	  	float dt_4 = dt_3 * dt;
        
        //set the process covariance matrix Q
	    Q_ = MatrixXd(4, 4);
	    Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
	           0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
	           dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
	           0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

	    Predict();
	}

	/****************************************************************************
	*  Update                                                                   *
	****************************************************************************/

	/**
	TODO:
	 * Use the sensor type to perform the update step.
	 * Update the state and covariance matrices.
	*/

	if (sensor_type_ == RADAR) {
	// Radar updates
	R_ = R_radar_;
	H_ = CalculateJacobian(x_);
	UpdateEKF(raw_measurements_);
	} else {
	// Laser updates
	R_ = R_lidar_;
	H_ = H_lidar_;
	Update(raw_measurements_);
	}

	// print the output
	cout << "x_ = " << x_ << endl;
	cout << "P_ = " << P_ << endl;
}