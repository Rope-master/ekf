/**************************************************************************
 * @author z.h
 * @date 2018.12.28
 * usage:kalman_filter_cv.hpp //opencv
 *
 **************************************************************************/

#ifndef KALMAN_FILTER_CV_H
#define KALMAN_FILTER_CV_H

#include <opencv2/opencv.hpp>

class kalman_filter_cv
{
public:
	kalman_filter_cv();
	~kalman_filter_cv();

	void _init(std::vector<float> v, int _mode)
	{
		if(_mode == 4)
		{
			state_num = 4;
			measure_num = 2;

			kf.init(state_num, measure_num, 0);
			kf.transitionMatrix = (cv::Mat_<float>(state_num, state_num) << 
				1, 0, 1, 0, 
				0, 1, 0, 1, 
				0, 0, 1, 0, 
				0, 0, 0, 1);

			setIdentity(kf.measurementMatrix, cv::Scalar::all(1));
			setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-5));
			setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1));
			setIdentity(kf.errorCovPost, cv::Scalar::all(1));

			kf.statePost.at<float>(0, 0) = v[0];
			kf.statePost.at<float>(1, 0) = v[1];
			kf.statePost.at<float>(2, 0) = 0;
			kf.statePost.at<float>(3, 0) = 0;
		}
		else if(_mode == 8)
		{
			state_num = 8;
			measure_num = 4;

			kf.init(state_num, measure_num, 0);

			kf.transitionMatrix = (cv::Mat_<float>(state_num, state_num) <<
				1, 0, 0, 0, 1, 0, 0, 0,
				0, 1, 0, 0, 0, 1, 0, 0,
				0, 0, 1, 0, 0, 0, 1, 0,
				0, 0, 0, 1, 0, 0, 0, 1,
				0, 0, 0, 0, 1, 0, 0, 0,
				0, 0, 0, 0, 0, 1, 0, 0,
				0, 0, 0, 0, 0, 0, 1, 0,
				0, 0, 0, 0, 0, 0, 0, 1);

			setIdentity(kf.measurementMatrix, cv::Scalar::all(1));
			setIdentity(kf.processNoiseCov, cv::Scalar::all(1e-3));
			setIdentity(kf.measurementNoiseCov, cv::Scalar::all(1e-2));
			setIdentity(kf.errorCovPost, cv::Scalar::all(1));

			kf.statePost.at<float>(0, 0) = v[0];
			kf.statePost.at<float>(1, 0) = v[1];
			kf.statePost.at<float>(2, 0) = v[2];
			kf.statePost.at<float>(3, 0) = v[3];
			kf.statePost.at<float>(4, 0) = 0;
			kf.statePost.at<float>(5, 0) = 0;
			kf.statePost.at<float>(6, 0) = 0;
			kf.statePost.at<float>(7, 0) = 0;

		}
	}

	std::vector<float> _predict()
	{
		std::vector<float> res;
		cv::Mat state_ = kf.predict();

		for(int i = 0; i < state_.rows; ++i)
		{
			res.push_back(state_.at<float>(i,0));
		}
		return res;
	}

	void _correct(std::vector<float> v, int _mode)
	{
		if(_mode == 4)
		{
			cv::Mat measure_m = (cv::Mat_<float>(measure_m, 1) << v[0], v[1]);
			kf.correct(measure_m);
		}
		else if(_mode == 8)
		{
			cv::Mat measure_m = (cv::Mat_<float>(measure_m, 1) << v[0], v[1], v[2], v[3]);
			kf.correct(measure_m);
		}
	}

private:
	cv::KalmanFilter kf;
	int mode;
	int state_num;
	int measure_num;

	
};

#endif