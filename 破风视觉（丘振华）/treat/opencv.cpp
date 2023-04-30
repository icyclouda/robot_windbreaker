#include"include/get_contours.h"
#include"include/headline.h"
#include <conio.h>

#define low

int main()
{
	#ifndef low
	
	
	//median_blur , h_high , h_low, size , size_1 , dilate_size;
	int Median_blur = 3, H_high = 1, H_low = 1,size = 2,size_1=2, Dilate_size = 2;
	
	
	
	namedWindow("Trackbars", (640, 200));
	createTrackbar("median_blur", "Trackbars", &Median_blur, 30);
	createTrackbar("h_high", "Trackbars", &H_high, 10);
	createTrackbar("h_low", "Trackbars", &H_low, 10);
	createTrackbar("size", "Trackbars", &size, 20);
	createTrackbar("size_1", "Trackbars", &size_1, 20);
	createTrackbar("dilate_size", "Trackbars", &Dilate_size, 15);
	
	while (1)
	{	
		int true_median_blur=Median_blur*2-1;

		myimg photo1, photo2, photo3, photo4;
		string path1 = "/home/qiuzhenhua/robotgame/photo/high/1.jpg";
		// string path2 = "/home/qiuzhenhua/robotgame/photo/high/2.jpg";
		// string path3 = "/home/qiuzhenhua/robotgame/photo/high/3.jpg";
		// string path4 = "/home/qiuzhenhua/robotgame/photo/high/4.jpg";
		photo1.test_high_treat(path1 , true_median_blur , H_high , H_low,size ,size_1 , Dilate_size);
		// photo2.test_treat(path2 , true_median_blur , H_high , H_low,size ,size_1 , Dilate_size);
		// photo3.test_treat(path3 , true_median_blur , H_high , H_low,size ,size_1 , Dilate_size);
		// photo4.test_treat(path4 , true_median_blur , H_high , H_low,size ,size_1 , Dilate_size);
		photo1.show_result("img1");
		// photo2.show_result("img2");
		// photo3.show_result("img3");
		// photo4.show_result("img4");
		// }

	}
	#endif

	#ifdef low
		myimg photo1, photo2, photo3, photo4;
		string path1 = "/home/qiuzhenhua/robotgame/photo/low/1.jpg";
		photo1.test_low_treat(path1);
		photo1.show_result("img1");
		waitKey(0);
	#endif
	return 0;
}