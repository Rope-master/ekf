/**************************************************************************
 * @author z.h
 * @date 2018.12.30
 * @usage:kf.h 
 * @brief:@params[IN]:@return:
 **************************************************************************/
#ifndef KF_H_KF_H
#define KF_H_KF_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>

class kf
{
public:

	/**
	* Constructor
	*/
	kf();

	/**
	* Destructor
	*/
	virtual ~kf(); 

	/**
	* Init Initializes Kalman filter
	* @param x_in Initial state
	* @param P_in Initial state covariance
	* @param F_in Transition matrix
	* @param Q_in Process covariance matrix
	* @param H_in Measurement matrix
	* @param R_in Measurement covariance matrix
	*/
	void Init(Eigen::VectorXd &x_in, Eigen::MatrixXd &P_in, Eigen::MatrixXd &F_in, 
			  Eigen::MatrixXd &Q_in, Eigen::MatrixXd &H_in, Eigen::MatrixXd &R_in);

	/**
	* Prediction Predicts the state and the state covariance
	*/
	void Predict();


	/**
	* Update Updates the state by using Standard Kalman Filter equations
	* @param z The measurement at k+1
	*/
	void Update(const Eigen::VectorXd &z);

	/**
	* Update Updates the state by using Extended Kalman Filter equations
	* @param z The measurement at k+1
	*/   
	virtual void Update_ekf(const Eigen::VectorXd &z) = 0;


protected:

	// state vector
	Eigen::VectorXd x_;

	// state covariance matrix
	Eigen::MatrixXd P_;

	// state transistion matrix
	Eigen::MatrixXd F_;

	// process covariance matrix
	Eigen::MatrixXd Q_;

	// measurement matrix
	Eigen::MatrixXd H_;

	// measurement covariance matrix
	Eigen::MatrixXd R_;

	
};

#endif /* KF_H_KF_H */