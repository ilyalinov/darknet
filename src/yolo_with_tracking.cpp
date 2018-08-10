#include <string>
#include <vector>
#include <fstream>

#define OPENCV
#define GPU

// import functions from YOLO DLL
#include "yolo_v2_class.hpp"

#include "tracker.h"

// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

Mat draw_boxes(Mat mat_img, vector<bbox_t> result_vec, vector<long double> distance_vec, std::vector<std::string> obj_names) 
{
	int j = 0;
	for (auto &i : result_vec)
	{
		Scalar color = obj_id_to_color(i.obj_id);
		rectangle(mat_img, Rect(i.x, i.y, i.w, i.h), color, 3);
		if (obj_names.size() > i.obj_id)
		{
			string obj_name;
			// obj_name = obj_names[i.obj_id];
			if (i.track_id > 0)
			{
				// obj_name += " - " + to_string(i.track_id);
				int distance = int(distance_vec[j]);
				// obj_name += " - " + ;
				obj_name += to_string(distance) + "m";
			}

			putText(mat_img, obj_name, Point2i(i.x, i.y - 10), FONT_HERSHEY_TRIPLEX, 1, color, 2);
		}

		j++;
	}

	cv::imshow("res/window name", mat_img);
	// cv::imwrite("res/res" + to_string(k) + ".jpg", mat_img);

	return mat_img;
}

vector<string> obj_names_from_file(string const filename)
{
	ifstream f(filename);
	vector<string> file_lines;
	if (!f.is_open())
	{
		return file_lines;
	}

	string line;
	while (getline(f, line))
	{
		file_lines.push_back(line);
	}

	f.close();
	return file_lines;
}

int main()
{
	string names_file = "data/coco.names";
	string weights_file = "yolov3.weights";
	string cfg_file = "yolov3.cfg";

	Detector detector(cfg_file, weights_file);
	detector.nms = float(0.2);
	Tracker tracker;

	// { 0, 2, 3, 5, 7 };
	std::vector<unsigned int> obj_ids { 2, 3, 5, 7 };

	string path = "C:/Users/ilya_/Downloads/2011_09_26/test/0000000%03d.png";
	VideoCapture cap(path);
	if (!cap.isOpened())
	{
		cout << "Error opening video stream or file" << endl;
		system("pause");
		return -1;
	}

	int width = (int)cap.get(CV_CAP_PROP_FRAME_WIDTH);
	int height = (int)cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	// int fps = (int)cap.get(CAP_PROP_FPS);
	int fps = 10;
	VideoWriter video("result.avi", CV_FOURCC('I', 'Y', 'U', 'V'), fps, cv::Size(width, height), true);

	auto obj_names = obj_names_from_file(names_file);
	Mat frame;
	vector<bbox_t> result_vec, tracking_vec, detection_vec;
	vector<long double> distance_vec;
	while (true)
	{
		try
		{
			cap >> frame;
			// cv::imwrite("res/cur_frame" + to_string(k) + ".jpg", frame);
			if (!frame.empty())
			{
				detection_vec = detector.detect(frame);
				// delete all unnecessary detections

				unsigned int detection_vec_size = detection_vec.size();
				cout << "size = " << detection_vec_size << endl;
				int erased = 0;
				for (unsigned i = 0; i < detection_vec_size; i++)
				{
					bool add = false;
					for (auto j: obj_ids)
					{
						if (detection_vec[i - erased].obj_id == j)
						{
							
							add = true;
						}
					}

					if (!add)
					{
						cout << "erased" << detection_vec[i - erased].obj_id << " ";
						detection_vec.erase(detection_vec.begin() + (i - erased));
						erased++;
					}
				}
				cout << endl;

				detection_vec = detector.tracking_id(detection_vec, true, 10, 100);
				result_vec = detection_vec;

				// comment it if tracking isn't needed
				tracking_vec = tracker.get_result();

				// add only not detected tracked objects
				for (auto &i: tracking_vec)
				{
					bool is_detected = false;
					for (auto &j: result_vec)
					{
						if ((i.track_id == j.track_id) && (i.obj_id == j.obj_id))
						{
							is_detected = true;
						}
					}
					
					if (!is_detected)
					{
						result_vec.push_back(i);
					}
				}

				// calculate distance
				for (auto &i: result_vec)
				{
					long double y = long double(int(i.y) + int(i.h) - (frame.size().height / 2)); // y coordinate of contact point with the road
					long double f = 721.5; // camera focal distance in pixels
					long double h = 1.65; // camera height in metres
					long double distance = (y > 0 ? f * h / y : 0.);

					distance_vec.push_back(distance);
				}

				Mat output_frame = draw_boxes(frame, result_vec, distance_vec, obj_names);
				video.write(output_frame);
				tracker.extrapolate(detection_vec, frame.size());
				distance_vec.clear();
			}
			else
			{
				break;
			}

			if (waitKey(30) >= 0)
			{
				break;
			}
		}

		catch (exception &e)
		{
			cerr << "exception " << e.what() << endl;
			getchar();
		}
		catch (...)
		{
			cerr << "unknown exception" << endl;
			getchar();
		}
	}

	video.release();
	return 0;
}