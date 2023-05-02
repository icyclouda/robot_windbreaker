#include"get_contours.h"
#define Run

void myimg::load_img(Mat image)
{
	img = image;
}

Mat myimg::get_img()
{
	return img;
}

void myimg::load_img_draw(Mat image)
{
	img_draw = image;
}

Mat myimg::get_img_draw()
{
	return img_draw;
}

double myimg::load_h()
{
	if (img.empty())
	{
		cout << "could not find image..." << endl;
		return -1;
	}

	Mat hsvImg;
	cvtColor(img, hsvImg, COLOR_BGR2HSV);
	waitKey(1);
	h = hsvImg.at<Vec3b>(1, 1)[0];
	return h;
}

double myimg::get_h()
{
	return h;
}

void myimg::test_pre(int median_blur, int h_high, int h_low,int size,int size_1, int dilate_size)
{
	Mat kernel = getStructuringElement(MORPH_RECT, Size(size,size));
	Mat kernel1 = getStructuringElement(MORPH_RECT, Size(size_1,size_1));
	// morphologyEx(img,img,2,kernel);

	// #ifndef Run
	// imshow("morphologyEx2",img);
	// #endif

	medianBlur(img, img, median_blur);
	
	#ifndef Run
	imshow("median",img);
	#endif
	
	//GaussianBlur(img, img, Size(guass_size, guass_size), guass_blur, 0);
	
	#ifndef Run
	//imshow("Gaussian",img);
	#endif
	
	load_h();
	
	cvtColor(img, img, COLOR_BGR2HSV);
	
	#ifndef Run
	imshow("HSV",img);
	#endif
	
	int hmin = h - h_low, smin = 0, vmin = 0;
	int hmax = h + h_high, smax = 255, vmax = 255;
	Scalar lower(hmin, smin, vmin);//HSV��Χ���ֵ
	Scalar upper(hmax, smax, vmax);//HSV��Χ���ֵ
	inRange(img, lower, upper, img);
	
	#ifndef Run
	imshow("inRange",img);
	#endif

	morphologyEx(img,img,3,kernel);
	
	#ifndef Run
	imshow("morphologyEx1",img);
	#endif
	
	morphologyEx(img,img,2,kernel1);

	#ifndef Run
	imshow("morphologyEx2",img);
	#endif
	
	Canny(img, img, 50 ,50);
	
	#ifndef Run
	imshow("Canny",img);
	#endif
	
	Mat d_kernel = getStructuringElement(MORPH_RECT, Size(dilate_size, dilate_size));//
	
	dilate(img, img, d_kernel);
	
	#ifndef Run
	imshow("dilate",img);
	#endif
}

void myimg::low_pre()
{
	//预处理:灰度,高斯,边际检测,扩张
	cvtColor(img, img, COLOR_BGR2GRAY);

	//GaussianBlur(img_me, imgBlur, Size(5, 5), 3, 0);
	//GaussianBlur(img_me, imgBlur, Size(3, 3), 5, 0);
	Canny(img , img ,25 , 60);
	Mat d_kernel_2 = getStructuringElement(MORPH_RECT, Size(2, 2));
	dilate(img, img, d_kernel_2);
}

void myimg::get_contours()
{
	findContours(img, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
	drawContours(img_draw, contours, -1, Scalar(255, 0, 255), 2);

	vector<vector<Point>> conPoly(contours.size());
	vector<Rect> boundRect(contours.size());

	for (int i = 0; i < contours.size(); i++)
	{
		int area = contourArea(contours[i]);

		string objectType;

		if (500 < area)
		{
			float peri = arcLength(contours[i], true);
			approxPolyDP(contours[i], conPoly[i], 0.02 * peri, true);
			drawContours(img_draw, conPoly, i, Scalar(255, 0, 255), 2);

			//cout << conPoly[i].size() << endl;
			boundRect[i] = boundingRect(conPoly[i]);
			rectangle(img_draw, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 5);


			int objCor = (int)conPoly[i].size();

			if (objCor == 3) { objectType = "3Tri"; }
			if (objCor == 4) {
				float aspRatio = (float)boundRect[i].width / (float)boundRect[i].height;
				if (aspRatio < 0.95 && aspRatio < 1.05) { objectType = "4Square"; }
				else { objectType = "4Rect"; }
			}
			if (objCor == 5) { objectType = "5pentagon"; }
			if (objCor == 6) { objectType = "6hexagon"; }
			if (objCor == 7) { objectType = "7heptagon"; }
			if (objCor == 8) { objectType = "8octagon"; }
			if (objCor == 10 || objCor == 9) { objectType = "star"; }
			if (objCor > 10) { objectType = "Circle"; }

			putText(img_draw, objectType, { boundRect[i].x,boundRect[i].y - 5 }, FONT_HERSHEY_DUPLEX, 0.75, Scalar(0, 69, 255), 2);
		}
	}

}

void myimg::test_high_treat(string path, int median_blur, int h_high, int h_low,int size,int size_1, int dilate_size)
{
	Mat temp_img1 = imread(path);
	Mat temp_img2 = imread(path);
	load_img(temp_img1);
	load_img_draw(temp_img2);

	test_pre(median_blur, h_high, h_low,size,size_1, dilate_size);
	get_contours();
}

void myimg::test_low_treat(string path)
{
	Mat temp_img1 = imread(path);
	Mat temp_img2 = imread(path);
	load_img(temp_img1);
	load_img_draw(temp_img2);

	low_pre();
	get_contours();
}

void myimg::show_result(string name)
{
	//resize(img_draw, img_draw, Size(), 0.4, 0.4);
	imshow(name, img_draw);
}