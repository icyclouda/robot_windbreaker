#pragma once
#include"headline.h"

class myimg
{
public:

	void load_img(Mat image);

	void load_img_draw(Mat image);

	Mat get_img();
	
	Mat get_img_draw();

	double load_h();

	double get_h();

	void test_pre(int median_blur ,int h_high ,int h_low ,int size,int size_1,int dilate_size);

	void low_pre();

	void get_contours();

	void test_high_treat(string path, int median_blur, int h_high, int h_low,int size,int size_1, int dilate_size);

	void test_low_treat(string path);
	
	void show_result(string name);

private:
	Mat img;
	Mat img_draw;
	double h;
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
};
