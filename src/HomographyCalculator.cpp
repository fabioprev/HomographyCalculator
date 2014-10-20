#include "HomographyCalculator.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace std;
using namespace cv;

HomographyCalculator::HomographyCalculator(vector<Point2f> srcPoints, vector<Point2f> dstPoints)
{
	this->srcPoints = srcPoints;
	this->dstPoints = dstPoints;
}

HomographyCalculator::~HomographyCalculator() {;}

Mat HomographyCalculator::calculateHomography()
{
	Mat H = findHomography(Mat(srcPoints),Mat(dstPoints),0,0.5);
	
	cout << "H:" << endl;
	
	for (int i = 0; i < H.rows; ++i)
	{
		for (int j = 0; j < H.cols; ++j)
		{
			cout << H.at<double>(i,j) << " ";
		}
		
		cout << endl;
	}
	
	cout << endl;
	
	return H;
}

void HomographyCalculator::calculateProjection(Point2f& point, const Mat& H, double& x, double& y)
{
	x =	(point.x * H.at<double>(0,0) + point.y * H.at<double>(0,1) + H.at<double>(0,2)) / (point.x * H.at<double>(2,0) + point.y * H.at<double>(2,1) + H.at<double>(2,2));
	y =	(point.x * H.at<double>(1,0) + point.y * H.at<double>(1,1) + H.at<double>(1,2)) / (point.x * H.at<double>(2,0) + point.y * H.at<double>(2,1) + H.at<double>(2,2));
}

void HomographyCalculator::draw(const Mat& srcImage, const Mat& dstImage, const Mat& H)
{
	Mat dstMat, srcMat;
	
	dstImage.copyTo(dstMat);
	srcImage.copyTo(srcMat);
	
	/// Draw the inlier points.
	vector<Point2f>::const_iterator itPts = dstPoints.begin();
	
	while (itPts != dstPoints.end())
	{
		circle(dstMat,*itPts,6,Scalar(255,255,255),2);
		++itPts;
	}
	
	itPts = srcPoints.begin();
	
	while (itPts != srcPoints.end())
	{
		circle(srcMat,*itPts,6,Scalar(120,120,120),2);
		++itPts;
	}
	
	/// Display the images with points.
	namedWindow("Dst Points");
	namedWindow("Src Points");
	
	imshow("Dst Points", dstMat);
	imshow("Src Points",srcMat);
	
	Mat reprojectionImage = dstMat.clone();
	itPts = srcPoints.begin();
	
	double x, y;
	int cnt = 0;
	
	for(; itPts != srcPoints.end(); ++itPts)
	{
		Point2f p((*itPts).x, (*itPts).y);
		calculateProjection(p,H,x,y);
		
		line(reprojectionImage,Point2f(x,y),Point2f(x + 10,y),CV_RGB(0,0,255),2);
		line(reprojectionImage,Point2f(x,y),Point2f(x - 10,y),CV_RGB(0,0,255),2);
		line(reprojectionImage,Point2f(x,y),Point2f(x,y + 10),CV_RGB(0,0,255),2);
		line(reprojectionImage,Point2f(x,y),Point2f(x,y - 10),CV_RGB(0,0,255),2);
		
		string s;
		stringstream out;
		
		out << cnt;
		s.assign(out.str());
		
		putText(reprojectionImage,s,Point2f(x + 5,y - 5),CV_FONT_HERSHEY_SIMPLEX,0.6,CV_RGB(0,0,255));
	}
	
	namedWindow("Reprojected Points");
	imshow("Reprojected Points",reprojectionImage);
	
	waitKey(0);
}

int main(int argc, char** argv)
{
	Mat kinect, planarView;
	vector<Point2f> obj;
	vector<Point2f> scene;
	
	obj.push_back(Point2f(100.83,334.17));
	obj.push_back(Point2f(177.92,164.58));
	obj.push_back(Point2f(218.75,74.17));
	obj.push_back(Point2f(573.75,75));
	obj.push_back(Point2f(669.58,330.42));
	obj.push_back(Point2f(426.67,293.75));
	obj.push_back(Point2f(414.58,84.58));
	obj.push_back(Point2f(718.33,82.92));
	obj.push_back(Point2f(750.83,125));
	obj.push_back(Point2f(639.58,125.42));
	obj.push_back(Point2f(688.33,217.5));
	obj.push_back(Point2f(325,384.58));
	obj.push_back(Point2f(134.17,58.33));
	obj.push_back(Point2f(361.67,59.58));
	obj.push_back(Point2f(538.33,165.83));
	
	/*
	scene.push_back(Point2f(92.52,772.70));
	scene.push_back(Point2f(343.38,775.69));
	scene.push_back(Point2f(573.63,776.48));
	scene.push_back(Point2f(569.06,974.27));
	scene.push_back(Point2f(95.49,969.05));
	scene.push_back(Point2f(154.40,883.15));
	scene.push_back(Point2f(541.62,885.20));
	scene.push_back(Point2f(541.23,1049.11));
	scene.push_back(Point2f(432.05,1048.81));
	scene.push_back(Point2f(431.28,995.32));
	scene.push_back(Point2f(248.51,995.47));
	scene.push_back(Point2f(38.74,851.34));
	scene.push_back(Point2f(623.54,723.79));
	scene.push_back(Point2f(620.00,855.27));
	scene.push_back(Point2f(341.06,939.89));
	*/
	
	scene.push_back(Point2f(772.70,586.48));
	scene.push_back(Point2f(775.69,335.62));
	scene.push_back(Point2f(776.48,105.37));
	scene.push_back(Point2f(974.27,109.94));
	scene.push_back(Point2f(969.05,583.51));
	scene.push_back(Point2f(883.15,534.6));
	scene.push_back(Point2f(885.20,137.38));
	scene.push_back(Point2f(1049.11,137.77));
	scene.push_back(Point2f(1048.81,246.95));
	scene.push_back(Point2f(995.32,247.72));
	scene.push_back(Point2f(995.47,430.49));
	scene.push_back(Point2f(851.34,640.26));
	scene.push_back(Point2f(723.79,55.46));
	scene.push_back(Point2f(855.27,59));
	scene.push_back(Point2f(939.89,337.94));
	
	HomographyCalculator homography(obj,scene);
	
	const Mat& H = homography.calculateHomography();
	
	if ((argv[1] != 0) && (argv[2] != 0))
	{
		kinect = imread(argv[1]);
		planarView = imread(argv[2]);
	}
	
	if ((!kinect.empty()) && (!planarView.empty())) homography.draw(kinect,planarView,H);
	else cerr << "Failed to load images. Projection is not possible..." << endl;
	
	return 0;
}
