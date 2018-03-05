/* Test */

// #pragma warning(disable: 4819)

#include "lane_detector/LineDetector.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <cmath>
#include <string>

//ȭ�� resize
#define width 960/2
#define length 540/2

//�� �������� ���� �Ӱ谪.
#define THRESHOLD 5  //5

// ���� ��ġ
#define LINE1 -25
#define LINE2 -30
#define LINE3 -15



int main(int argc, char** argv)
{
	ros::init(argc, argv, "lane_detector");
	ros::NodeHandle nh;

	ros::Publisher control_pub = nh.advertise<std_msgs::String>("write", 100);
	VideoCapture cap(1);
	// cap.open("cameraimage_color_camera3.mp4");

	double avg = 0;
	double sum = 0;
	int temp = 0;
	double angle = 0.0;


	if (!cap.isOpened())
	{
		cout << "Not opened cap" << endl;
		return -1;
	}

	int fps = 500;


	Mat frame, gray, bi;
	Mat Roi;
	Mat hsv;
	Mat hsv_s;
	Mat a, b;

	int framecount1_R = 0;
	int framecount1_L = 0;

	int framecount2_R = 0;
	int framecount2_L = 0;

	int framecount3_R = 0;
	int framecount3_L = 0;

	// ù��° ��ǥ ( ù��° ������ )
	int r0_p1 = 0;
	int l0_p1 = 0;

	int r0_p2=0;
	int l0_p2=0;

	int r0_p3=0;
	int l0_p3=0;

	// ������ǥ, ������ ��ǥ
	Point right_P1;
	Point left_P1;

	Point right_P2;
	Point left_P2;

	Point right_P3;
	Point left_P3;

	Point middle;

	LaneDetect linedetect;

	string tmp_control_value = "";
	std_msgs::String control_msg;

	for (;;)
	{
		int64 t1 = getTickCount();

		temp++;

		cap >> frame;

		resize(frame, frame, Size(width, length));


		
		if (frame.empty())
		{
			cout << "ȭ���� ��� �־��!!" << endl;
			break;
		}

		Roi = frame(Rect(0, length / 2, width, length / 2));
		//cvtColor(Roi, hsv, COLOR_BGR2HSV);
		cvtColor(Roi, gray, COLOR_BGR2GRAY);
		/*
		cvtColor(Roi, hsv, COLOR_BGR2HSV);

		vector<Mat> hsv_planes;

		split(hsv, hsv_planes);
		hsv_s = hsv_planes[1];  //s�� ����
		*/
		double bb = threshold(gray, b, 180, 255, THRESH_BINARY);
		//double aa = threshold(hsv_s, a, 110, 255, THRESH_BINARY);

		//bi = a + b; // bgr, hsv ����ȭ �Ȱ� ��ġ�� 
		bi = b;


		// �� �׳� ������ ���� �������� �ν��� �ʿ� ����!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!




		/*
		���� ���� �� ��..
		1. ����ȭ�� ȭ�鿡 �޼����� ���� ����Ʈ��, ���������� ������ ����Ʈ�� �д�.
		2. ���� ����Ʈ, ������ ����Ʈ �߽� ã��
		3. ���� ��ġ�� �� ����Ʈ ��ġ ���� - > ���Ⱒ // �߽ɰ� ������Ʈ, ������ ����Ʈ �Ÿ� - > ���� ���ǵ�
		4. ����Ʈ���� ��ġ�� �� ������ �ʵ��� ����ó��.
		*/
		
		// �ʱ��� ���ϱ�
		if (framecount1_L < 1) {
			l0_p1 = linedetect.find_L0_x(bi, bi.rows / 2 + LINE1, &framecount1_L , l0_p1);
			cout << "framecount1_L  " << framecount1_L << endl;
		}
		if (framecount1_R <1)	r0_p1 = linedetect.find_R0_x(bi, bi.rows /2 + LINE1, &framecount1_R , r0_p1);

		if (framecount2_L < 1) 	l0_p2 = linedetect.find_L0_x(bi, bi.rows / 2 + LINE2, &framecount2_L , l0_p2);
		if (framecount2_R <1)	r0_p2 = linedetect.find_R0_x(bi, bi.rows /2 + LINE2, &framecount2_R , r0_p2);

		if (framecount3_L < 1) 	l0_p3 = linedetect.find_L0_x(bi, bi.rows / 2 + LINE3, &framecount3_L , l0_p3);
		if (framecount3_R <1)	r0_p3 = linedetect.find_R0_x(bi, bi.rows /2 + LINE3, &framecount3_R , r0_p3);

		// ����Ʈ ���ϱ�
		right_P1.x = linedetect.find_RN_x(bi, r0_p1, LINE1, THRESHOLD);
		right_P1.y = bi.rows / 2 + LINE1;
		r0_p1 = right_P1.x;
		left_P1.x = linedetect.find_LN_x(bi, l0_p1, LINE1, THRESHOLD);
		left_P1.y = bi.rows / 2 + LINE1;
		l0_p1 = left_P1.x;
		

		right_P2.x = linedetect.find_RN_x(bi, r0_p2, LINE2, THRESHOLD);
		right_P2.y = bi.rows / 2 + LINE2;
		r0_p2 = right_P2.x;
		left_P2.x = linedetect.find_LN_x(bi, l0_p2, LINE2, THRESHOLD);
		left_P2.y = bi.rows / 2 + LINE2;
		l0_p2 = left_P2.x;

		right_P3.x = linedetect.find_RN_x(bi, r0_p3, LINE3, THRESHOLD);
		right_P3.y = bi.rows / 2 + LINE3;
		r0_p3 = right_P3.x;
		left_P3.x = linedetect.find_LN_x(bi, l0_p3, LINE3, THRESHOLD);
		left_P3.y = bi.rows / 2 + LINE3;
		l0_p3 = left_P3.x;

		middle = Point((right_P1.x + left_P1.x) / 2, bi.rows / 2 + LINE1);

		int dy = middle.x - bi.cols / 2;
		int dx = bi.rows - middle.y;
		angle = atan2(dy, dx) * 180 / CV_PI;


		int64 t2 = getTickCount();

		double ms = (t2 - t1) * 1000 / getTickFrequency();
		sum += ms;
		avg = sum / temp;

		cout << "---------------------------------" << endl;
		cout << "it took : " << ms << "ms." << "avg: " << avg << " fps : " << 1000 / avg << endl;

		// ���� �� �� ���� �Ÿ��� �ʹ� ������ �ٽ� �ν��ض� - > �� ������ ���� �ν��ϰ� ������..
		if (abs(right_P1.x - left_P1.x) < 15)
		{
			framecount1_L = 0;
			framecount1_R = 0;
		}

		if (abs(right_P2.x - left_P2.x) < 15)
		{
			framecount2_L = 0;
			framecount2_R = 0;
		}

		if (abs(right_P3.x - left_P3.x) < 15)
		{
			framecount3_L = 0;
			framecount3_R = 0;
		}

		
		//line(frame, left_P + Point(0,length / 2), right_P+ Point(0, length / 2), Scalar(255, 0, 0), 2);
		line(frame, right_P1 + Point(0, length / 2), left_P1 + Point(0, length / 2), Scalar(0, 255, 0), 5);
		line(frame, right_P2 + Point(0, length / 2), left_P2 + Point(0, length / 2), Scalar(0, 0, 255), 5);
		line(frame, right_P3 + Point(0, length / 2), left_P3 + Point(0, length / 2), Scalar(255, 0, 0), 5);
		line(frame, middle + Point(0, length / 2), Point(frame.cols / 2, frame.rows), Scalar(0, 0, 255), 5);
		//circle(frame, left_P + Point(0, length / 2), 5, Scalar(255, 0, 0), 5);
		//circle(frame, right_P + Point(0, length / 2), 5, Scalar(0, 255, 0), 5);
		imshow("binary img", bi);
		imshow("frame", frame);

		ROS_INFO("Angle: %f", angle);

		int angle_for_msg = 0;	// For parsing double value to int
		int control_factor = 25;
		// arduino steering range: -1500 < steer < 1500
		if(angle < control_factor && angle > (-1) * control_factor)
			angle_for_msg = static_cast<int>(1500 / control_factor * angle);
		else if(angle >= control_factor)
			angle_for_msg = 1500;
		else if(angle <= (-1) * control_factor)
			angle_for_msg = -1500;

		tmp_control_value = to_string(angle_for_msg);
		// cout << "test angle: " << tmp_control_value << endl;

		control_msg.data = tmp_control_value + ",0,";	// Make message
		cout << "test msg: " << control_msg.data << endl;

		control_pub.publish(control_msg);

		if (waitKey(1000 / fps) >= 0) break;


	}

	ros::spin();
	return 0;
}



	/*
	���� ���� �����ؼ� �����ϱ�
	*/
