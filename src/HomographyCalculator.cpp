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
	
	obj.push_back(Point2f(190,165));
	obj.push_back(Point2f(394,165));
	obj.push_back(Point2f(393,215));
	obj.push_back(Point2f(393,125));
	obj.push_back(Point2f(595,165));
	obj.push_back(Point2f(393,426));
	obj.push_back(Point2f(393,40));
	
	scene.push_back(Point2f(410,340));
	scene.push_back(Point2f(523,340));
	scene.push_back(Point2f(523,450));
	scene.push_back(Point2f(524,230));
	scene.push_back(Point2f(638,340));
	scene.push_back(Point2f(524,676));
	scene.push_back(Point2f(524,2));
	
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
