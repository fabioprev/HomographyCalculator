#pragma once

#include <opencv2/core/core.hpp>
#include <vector>

class HomographyCalculator
{
	private:
		std::vector<cv::Point2f> srcPoints;
		std::vector<cv::Point2f> dstPoints;
		
		void calculateProjection(cv::Point2f& point, const cv::Mat& H, double& x, double& y);
		
	public:
		HomographyCalculator(std::vector<cv::Point2f> srcPoints, std::vector<cv::Point2f> dstPoints);
		
		~HomographyCalculator();
		
		cv::Mat calculateHomography();
		void draw(const cv::Mat& srcImage, const cv::Mat& dstImage, const cv::Mat& H);
};
