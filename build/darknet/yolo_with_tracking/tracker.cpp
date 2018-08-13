#include "tracker.h"

Tracker::Tracker()
{
	detection_history = std::deque<std::vector<bbox_t>>();
	tracking_history = std::vector<tracked_bbox_t>();
}

void Tracker::extrapolate(std::vector<bbox_t> detection_vec, cv::Size size)
{
	add_new_result(detection_vec);
	auto last_detection_vec = detection_history.back();
	// set correct last_detection_frames_ago counter for tracked objects
	for (auto &i: tracking_history)
	{
		for (auto &j: last_detection_vec)
		{
			if ((i.bbox.obj_id == j.obj_id) && (i.bbox.track_id == j.track_id))
			{
				i.current_detection = true;
				i.last_detection_frames_ago = 0;
				break;
			}
		}

		if (!i.current_detection)
		{
			i.last_detection_frames_ago++;
		}
	}

	std::vector<tracked_bbox_t> tracking_result_vec;
	// extrapolate new detected objects
	for (auto &i: last_detection_vec)
	{
		auto res = extrapolate_bbox(i, size);
		if (res.bbox.w != 0 && res.bbox.h != 0)
			tracking_result_vec.push_back(res);
	}

	// add old tracked objects
	for (auto &i: tracking_history)
	{
		if (i.current_detection || (i.last_detection_frames_ago >= track_frames_history))
		{
			continue;
		}

		bool add_tracked_bbox = false;
		// i.bbox.x += int(i.shiftX);
		// i.bbox.y += int(i.shiftY);
		int x_new = int(i.bbox.x) + int(i.shiftX);
		int y_new = int(i.bbox.y) + int(i.shiftY);
		if (x_new >= 0 && x_new <= size.width && y_new >= 0 && y_new <= size.height)
		{
			i.bbox.x = x_new;
			i.bbox.y = y_new;
			add_tracked_bbox = true;
		}

		// correct width and height if needed
		if (int(i.bbox.x + i.bbox.w) > size.width)
		{
			i.bbox.w = size.width - i.bbox.x;
		}
		if (int(i.bbox.y + i.bbox.h) > size.height)
		{
			i.bbox.h = size.height - i.bbox.y;
		}

		if (add_tracked_bbox)
			tracking_result_vec.push_back(i);
	}

	tracking_history = tracking_result_vec;
}

std::vector<bbox_t> Tracker::get_result()
{
	if (tracking_history.empty())
	{
		return std::vector<bbox_t>();
	}

	std::vector<bbox_t> tracking_result_bbox_vec;
	for (auto &i: tracking_history)
	{
		tracking_result_bbox_vec.push_back(i.bbox);
	}

	return tracking_result_bbox_vec;
}

void Tracker::add_new_result(std::vector<bbox_t> detection_vec)
{
	detection_history.push_back(detection_vec);

	if (detection_history.size() > det_frames_history)
	{
		detection_history.pop_front();
	}

	set_cur_detection_flag_to_false();
}

Tracker::tracked_bbox_t Tracker::extrapolate_bbox(bbox_t b, cv::Size size)
{
	int counter_of_detections = 0;
	auto last_detected_frames_ago = (unsigned int)detection_history.size();
	std::vector<bbox_t> cur_bbox_detection;
	std::vector<unsigned int> detection_timestamps;
	for (auto &vec: detection_history)
	{
		last_detected_frames_ago--;
		auto it = std::find_if(vec.begin(), vec.end(), [&b](bbox_t i) { return (b.track_id == i.track_id) && (b.obj_id == i.obj_id); });
		if (!(it == vec.end()))
		{
			counter_of_detections++;
			cur_bbox_detection.push_back(*it);
			detection_timestamps.push_back(last_detected_frames_ago);
		}
	}
	
	auto result_tracked_bbox_t = Tracker::tracked_bbox_t();
	result_tracked_bbox_t.current_detection = true;
	result_tracked_bbox_t.last_detection_frames_ago = 0;
	// object detected once in last frame
	if (counter_of_detections == 1)
	{
		result_tracked_bbox_t.bbox = b;
		result_tracked_bbox_t.shiftX = 0;
		result_tracked_bbox_t.shiftY = 0;
	}
	// more than 1 detection
	else 
	{
		double shiftX = 0;
		double shiftY = 0;
		for (size_t i = 0; i < cur_bbox_detection.size() - 1; i++)
		{
			auto shift = calculate_shift(cur_bbox_detection[i], cur_bbox_detection[i + 1]);
			shiftX += shift.first;
			shiftY += shift.second;
		}

		auto time_between = detection_timestamps[0] - detection_timestamps[detection_timestamps.size() - 1];
		shiftX /= double(time_between);
		shiftY /= double(time_between);
		result_tracked_bbox_t.bbox = cur_bbox_detection[cur_bbox_detection.size() - 1];
		int shiftX_int = shiftX >= 0 ? int(shiftX + 0.5) : int(shiftX - 0.5);
		int shiftY_int = shiftY >= 0 ? int(shiftY + 0.5) : int(shiftY - 0.5);
		result_tracked_bbox_t.shiftX = shiftX;
		result_tracked_bbox_t.shiftY = shiftY;
		// result_tracked_bbox_t.bbox.x += int(shiftX);
		// result_tracked_bbox_t.bbox.y += int(shiftY);
		bool add_result_tracked_bbox = false;
		int x_new = int(result_tracked_bbox_t.bbox.x) + shiftX_int;
		int y_new = int(result_tracked_bbox_t.bbox.y) + shiftY_int;
		if (x_new >= 0 && x_new <= size.width && y_new >= 0 && y_new <= size.height)
		{
			result_tracked_bbox_t.bbox.x = x_new;
			result_tracked_bbox_t.bbox.y = y_new;

			// correct width and height if needed
			if (int(result_tracked_bbox_t.bbox.x + result_tracked_bbox_t.bbox.w) > size.width)
			{
				result_tracked_bbox_t.bbox.w = size.width - result_tracked_bbox_t.bbox.x;
			}
			if (int(result_tracked_bbox_t.bbox.y + result_tracked_bbox_t.bbox.h) > size.height)
			{
				result_tracked_bbox_t.bbox.h = size.height - result_tracked_bbox_t.bbox.y;
			}
		}
		else // return empty box
		{
			result_tracked_bbox_t.bbox.w = 0;
			result_tracked_bbox_t.bbox.h = 0;
		}
	}

	return result_tracked_bbox_t;
}

std::pair<double, double> Tracker::calculate_shift(bbox_t b1, bbox_t b2)
{
	auto shift = std::pair<double, double>();
	// shiftX
	shift.first = (double(b2.x) + (double)b2.w / 2.) - (double(b1.x) + (double)b1.w / 2.);
	// shiftY
	shift.second = (double(b2.y) + (double)b2.h / 2.) - (double(b1.y) + (double)b1.h / 2.);
	return shift;
}

void Tracker::set_cur_detection_flag_to_false()
{
	for (auto &i: tracking_history)
	{
		i.current_detection = false;
	}
}