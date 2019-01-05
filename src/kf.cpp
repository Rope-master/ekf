/**************************************************************************
 * @author z.h
 * @date 2018.12.30
 * @usage:kf.cpp 
 * @brief:@params[IN]:@return:
 **************************************************************************/
#include <iostream>
#include "kf.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

const double PI = 3.141592653589793238463;
const float EPS = 0.0001;


kf::kf() {}

kf::~kf() {}

void kf::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, 
			  MatrixXd &Q_in, MatrixXd &H_in, MatrixXd &R_in){
	x_ = x_in;
	P_ = P_in;
	F_ = F_in;
	Q_ = Q_in;
	H_ = H_in;
	R_ = R_in;
}

void kf::Predict(){
	/**
	TODO:
	* predict the state
	*/
	x_ = F_ * x_;
	P_ = F_ * P_ * F_.transpose() + Q_;
}

void kf::Update(const VectorXd &z) {
	/**
	TODO:
	* update the state by using Kalman Filter equations
	*/
	VectorXd y = z - H_ * x_;
	/**
	* @param y adjusted between -pi and pi;
	*/
	while (y(1) < -PI)	{y(1) += 2 * PI;}
	while (y(1) >  PI)	{y(1) += 2 * PI;}

	MatrixXd S = H_ * P_ * H_.transpose() + R_;
	MatrixXd K = P_ * H_.transpose() * S.inverse();
	x_ = x_ + (K * y);
	long size = x_.size();
	MatrixXd I = MatrixXd::Identity(size,size);
	P_ = (I - K * H_) * P_;
  
}


