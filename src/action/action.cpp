//
// Created by root on 11/29/17.
//

#include "pp.h"

void EventAction::run() {
	while (ros::ok()) {
		bool eh_status_now = false, eh_status_last = false;
		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
			usleep(100);
			continue;
		}

		if (isExit()) {
			break;
		}

		doSomething();
		if (isStop()) {
			setNext();
		}
	}
}

//
//void EventBackFromCharger::run() {
//	while (ros::ok()) {
//		bool eh_status_now = false, eh_status_last = false;
//		if (event_manager_check_event(&eh_status_now, &eh_status_last) == 1) {
//			usleep(100);
//			continue;
//		}
//
//		if (isExit()) {
//			break;
//		}
//
//		if (cs.is_back_from_charger()) {
//			// run
//			wheel.set_speed(20, 20);
//
//			// switch
//			if (two_points_distance_double(charger_pose.getX(), charger_pose.getY(), odom.getX(), odom.getY()) > 0.5)
//				cs.setNext(CS_OPEN_LASER);
//
//		}
//		else if (cs.is_open_laser()) {
//			// run
//			wheel.set_speed(0, 0);
//
//			// switch
//			if (laser.isScanOriginalReady() == 1) {
//				// Open laser succeeded.
//				if ((g_is_manual_pause || g_is_low_bat_pause) && slam.isMapReady())
//					cs.setNext(CS_CLEAN);
//				else
//					cs.setNext(CS_ALIGN);
//			}
//		}
//		else if (cs.is_align()) {
//			// run
//			wheel.set_speed(0, 0);
//			std::vector<LineABC> lines;
//			float align_angle = 0.0;
//			if (laser.findLines(&lines)) {
//				if (laser.getAlignAngle(&lines, &align_angle))
//					laser.alignAngle(align_angle);
//			}
//
//			// switch
//			if (laser.alignTimeOut())
//				cs.setNext(CS_OPEN_SLAM);
//			if (laser.alignFinish()) {
//				float align_angle = laser.alignAngle();
//				align_angle += (float) (LIDAR_THETA / 10);
//				robot::instance()->offsetAngle(align_angle);
//				g_homes[0].TH = -(int16_t) (align_angle);
//				ROS_INFO("%s %d: align_angle angle (%f), g_homes[0].TH (%d).", __FUNCTION__, __LINE__, align_angle,
//								 g_homes[0].TH);
//				usleep(230000);
//				cs.setNext(CS_OPEN_SLAM);
//			}
//		}
//		else if (cs.is_open_slam()) {
//			if (slam.isMapReady() && robot::instance()->isTfReady()) {
//				cs.setNext(CS_CLEAN);
//				break;
//			}
//		}
//	}
//}
//
//bool EventBackFromCharger::isStop() {
//	return gyro.isOn();
//}
//
//bool EventBackFromCharger::setNext() {
//	if (charger.is_on_stub()) {
//		cs.setNext(CS_BACK_FROM_CHARGER);
//		charger_pose.setX(odom.getX());
//		charger_pose.setY(odom.getY());
//	}
//	else
//		cs.setNext(CS_OPEN_LASER);
//}
//
//void EventBackFromCharger::doSomething() {
//	wheel.set_speed(20, 20);
//}
//

