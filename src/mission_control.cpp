#include "ros/ros.h"

#include "rasendriya/Dropzone.h"

#include "mavros_msgs/WaypointReached.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/WaypointList.h"
#include "mavros_msgs/WaypointPull.h"

#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/Altitude.h"

#include "geometry_msgs/TwistStamped.h"

#include "sensor_msgs/NavSatFix.h"

#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

<<<<<<< HEAD
#include "std_srvs/SetBool.h"

=======
>>>>>>> b1ec7fec8e7b577cb23b8266008eaabdca61e7b1
#include <math.h>
#include <bits/stdc++.h>

// REMINDER: wp_num STARTS FROM 0

int waypoint_reached = 0;

void waypoint_reached_callback(const mavros_msgs::WaypointReached& wp_reached){
	waypoint_reached = wp_reached.wp_seq;
}

// CALLBACKS //

int x_pixel, y_pixel;
float alt, gps_hdg;
double gps_long, gps_lat;
float vel_y, vel_z;
bool mission_flag;

mavros_msgs::WaypointPush waypoint_push;

<<<<<<< HEAD
bool dropzone_target_callback(rasendriya::Dropzone::Request& dropzone_req, rasendriya::Dropzone::Response& dropzone_res){
	x_pixel = dropzone_req.x;
	y_pixel = dropzone_req.y;
	dropzone_res.status = true;
	return true;
=======
void dropzone_target_callback(const rasendriya::Dropzone& dropzone_loc){
	x_pixel = dropzone_loc.x;
	y_pixel = dropzone_loc.y;
>>>>>>> b1ec7fec8e7b577cb23b8266008eaabdca61e7b1
}

void gps_callback(const sensor_msgs::NavSatFix& gps_data){
	gps_long = gps_data.longitude;
	gps_lat = gps_data.latitude;
}

void alt_callback(const mavros_msgs::Altitude& alt_data){
	alt = alt_data.relative;
}

void gps_hdg_callback(const std_msgs::Float64& gps_hdg_data){
	gps_hdg = gps_hdg_data.data;
}

void vel_callback(const geometry_msgs::TwistStamped& vel_data){
	vel_y = vel_data.twist.linear.y;
	vel_z = vel_data.twist.linear.z;
}

<<<<<<< HEAD
=======
void mission_flag_callback(const std_msgs::Bool& mis_flag){
	mission_flag = mis_flag.data;
}

>>>>>>> b1ec7fec8e7b577cb23b8266008eaabdca61e7b1
void waypoint_list_callback(const mavros_msgs::WaypointList& wplist) {
	waypoint_push.request.waypoints = wplist.waypoints;
}

/*
	WAYPOINT MODIFIER APIs
*/

void insert_wp(int _wp_num, const mavros_msgs::Waypoint& _wp){	
	waypoint_push.request.waypoints.insert(waypoint_push.request.waypoints.begin() + _wp_num, _wp);
}

void erase_wp(int _wp_num){
	waypoint_push.request.waypoints.erase(waypoint_push.request.waypoints.begin() + _wp_num);
}

void swap_wp(int _wp_num, const mavros_msgs::Waypoint& _wp) {
	erase_wp(_wp_num);
	insert_wp(_wp_num, _wp);
}

void servo_drop_wp(int servo_ch, int wp_drop_num, mavros_msgs::Waypoint& _wp){
	_wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
	_wp.command = mavros_msgs::CommandCode::DO_SET_SERVO;
	_wp.is_current = true;
	_wp.autocontinue = true;
	_wp.param1 = servo_ch;
	_wp.param2 = 1100;

	insert_wp(wp_drop_num, _wp);
}

// mathematical conversion APIs because c++ library doesn't have it. math.h sucks 

float radians(float _deg) {
	return _deg*(3.14159/180);
} 

float degrees(float _rad) {
	return _rad*(180/3.14159);
}

// projectile motion calculator API

const float gravity = 9.81; // m/s^2
#define EPSILON 0.01

float projectile_func(float& _drop_offset, float _drop_alt, float _drag_coeff, float _vert_speed, float _hor_speed) {
	return _drop_alt + (_vert_speed+gravity/(2*_drag_coeff*_hor_speed))*_drop_offset/_hor_speed - gravity/(4*pow(_drag_coeff*_hor_speed, 2))*(exp(2*_drag_coeff*_drop_offset) -1);
}

float projectile_func_deriv(float& _drop_offset, float _drag_coeff, float _vert_speed, float _hor_speed) {
	return (_vert_speed+gravity/(2*_drag_coeff*_hor_speed))*1/_hor_speed - (gravity/4*pow(_drag_coeff*_hor_speed, 2)*(2*_drag_coeff*exp(2*_drag_coeff*_drop_offset)));
}


void calc_projectile_distance(float& _drop_offset, float _drop_alt, float _drag_coeff) {
	float vert_speed = vel_z;
	float hor_speed = vel_y;
	float h;

	// using newton-raphson technique
	do {
		h = projectile_func(_drop_offset, _drop_alt, _drag_coeff, vert_speed, hor_speed)/projectile_func_deriv(_drop_offset, _drag_coeff, vert_speed, hor_speed);
		// x(i+1) = x(i) - f(x) / f'(x) 
		_drop_offset = _drop_offset - h;
	}
  while (abs(h) >= EPSILON);

	return;

}

// coordinate calculator API

void calc_drop_coord(double& _tgt_latx, double& _tgt_lony, float _drop_offset, int _calc_mode){
	const float pi = 3.14159;
	const float R_earth = 6378.1*1e3;
	
	float hdg = radians(gps_hdg);
	double lat = radians(gps_lat);
	double lon = radians(gps_long);
	float cam_angle, r_dist;

	if(_calc_mode <= 2) {
		const double focal_length_x = 1.3817707364539*1e3; // calibrated result
		const double focal_length_y = 1.3766463458240*1e3; // calibrated result
		//const float pixel_to_mm = 0.2645;

		float X_meter = x_pixel*alt/focal_length_x;
		float Y_meter = y_pixel*alt/focal_length_y;
		
		r_dist = sqrt(pow(X_meter, 2) + pow(Y_meter + _drop_offset, 2));
		cam_angle = radians(atan2(X_meter, Y_meter));
	}
	else {
		cam_angle = radians(180);
		r_dist = _drop_offset;
	}
	
	// using haversine law
	_tgt_latx = degrees(asin(sin(lat)*cos(r_dist/R_earth) + cos(lat)*sin(r_dist/R_earth)*cos(hdg+cam_angle)));
	_tgt_lony = degrees(lon + atan2(sin(hdg+cam_angle)*sin(r_dist/R_earth)*cos(lat) , (cos(r_dist/R_earth)-sin(lat)*sin(_tgt_latx))));
}

// MAIN FUNCTION //

int main(int argc, char **argv) {
	int mission_repeat_counter = 0;
	int hit_count = 0;

	double tgt_latx, tgt_lony;

<<<<<<< HEAD
	mavros_msgs::WaypointPull waypoint_pull;

	std_srvs::SetBool vision_flag;
=======
	std_msgs::Bool vision_flag;

	mavros_msgs::WaypointPull waypoint_pull;
>>>>>>> b1ec7fec8e7b577cb23b8266008eaabdca61e7b1

	mavros_msgs::Waypoint wp;

	ros::init(argc, argv, "mission_control");
	ros::NodeHandle nh;

	ros::ServiceClient waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");

	ros::ServiceClient waypoint_pull_client = nh.serviceClient<mavros_msgs::WaypointPull>("/mavros/mission/pull");
<<<<<<< HEAD
	
	ros::ServiceClient vision_flag_client = nh.serviceClient<std_srvs::SetBool>("/rasendriya/vision_flag");

	ros::ServiceServer dropzone_service = nh.advertiseService("/rasendriya/dropzone", dropzone_target_callback);
=======

	ros::Subscriber mission_flag_subscriber = nh.subscribe("/rasendriya/mission_flag", 1, mission_flag_callback);
>>>>>>> b1ec7fec8e7b577cb23b8266008eaabdca61e7b1

	ros::Subscriber waypoint_list_sub = nh.subscribe("/mavros/mission/waypoints", 1, waypoint_list_callback);

	ros::Subscriber waypoint_reached_sub = nh.subscribe("/mavros/mission/reached", 1, waypoint_reached_callback);
<<<<<<< HEAD
=======
	ros::Subscriber dropzone_target_sub = nh.subscribe("/rasendriya/dropzone", 3, dropzone_target_callback);
>>>>>>> b1ec7fec8e7b577cb23b8266008eaabdca61e7b1
	ros::Subscriber gps_coordinate_sub= nh.subscribe("/mavros/global_position/global", 1, gps_callback);
	ros::Subscriber alt_sub = nh.subscribe("/mavros/altitude", 1, alt_callback);
	ros::Subscriber gps_hdg_sub = nh.subscribe("/mavros/global_position/compass_hdg", 1, gps_hdg_callback);
	ros::Subscriber vel_sub = nh.subscribe("/mavros/global_position/gp_vel", 1, vel_callback);

<<<<<<< HEAD
=======
	ros::Publisher vision_flag_publisher = nh.advertise<std_msgs::Bool>("/rasendriya/vision_flag", 1, true);

>>>>>>> b1ec7fec8e7b577cb23b8266008eaabdca61e7b1
	ros::Rate rate(25);
	
	// ROS LAUNCH PARAMETERS //
	
	int calc_mode;
	ros::param::get("/rasendriya/calc_mode", calc_mode);
	
	float dropping_offset, dropping_altitude, drag_coeff;
	ros::param::get("/rasendriya/dropping_altitude", dropping_altitude);

	// first WP loading from FCU. Ensures that companion computer has the same waypoints as FCU
	while(ros::ok()) {
		if(waypoint_push.request.waypoints.size() != 0) {
			ROS_INFO("Number of loaded waypoints: %d", int(waypoint_push.request.waypoints.size()));
			ROS_INFO("Waypoint load from FCU completed");
			break;
		}
		ros::spinOnce();
		rate.sleep();
	}

	while(ros::ok()) {
		
		ROS_INFO_ONCE("Mission program ready");
		
<<<<<<< HEAD
		ROS_INFO_ONCE("Mission program started");
		
		// turn on vision node when wp3 has reached
		if(waypoint_reached == 1 && mission_repeat_counter == 1){
			vision_flag.request.data = true;
			vision_flag_client.call(vision_flag);
			ROS_INFO_ONCE("Vision program started");
		}
		else {
			vision_flag.request.data = false;
			vision_flag_client.call(vision_flag);
			ROS_INFO_ONCE("Vision program stopped");
		}

		// dropzone confirmed
		if((x_pixel != NAN) && (y_pixel != NAN)){
			
			ROS_INFO_ONCE("DROPZONE TARGET ACQUIRED. PROCEED TO EXECUTE DROPPING SEQUENCE");

			if(calc_mode % 2 == 0) {
				ros::param::get("/rasendriya/drag_coefficient", drag_coeff);
				dropping_offset = 0;
				calc_projectile_distance(dropping_offset, dropping_altitude, drag_coeff);
			}
			else {
				ros::param::get("/rasendriya/dropping_offset", dropping_offset);
			}

			calc_drop_coord(tgt_latx, tgt_lony, dropping_offset, calc_mode);
			
			// send do jump command to WP 1 from WP 4 after dropzone has found as WP 5
			wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			wp.command = mavros_msgs::CommandCode::DO_JUMP;
			wp.is_current = false;
			wp.autocontinue = true;
			wp.param1 = 1;
			wp.param2 = 2;
			insert_wp(4, wp);

			// send plane attitude waypoint for dropping as WP 3 in WP 2
			wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
			wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
			wp.is_current = false;
			wp.autocontinue = true;
			wp.param2 = 1;
			wp.param3 = 0;
			wp.x_lat = tgt_latx;
			wp.y_long = tgt_lony;
			wp.z_alt = dropping_altitude;
			insert_wp(2, wp);

			// send waypoint to drop front ball
			servo_drop_wp(7, 3, wp);

			if(waypoint_push_client.call(waypoint_push) && waypoint_pull_client.call(waypoint_pull)){
				ROS_INFO("FIRST WAYPOINT NAVIGATION DROP SENT");
			}
			else {
				ROS_WARN("FAILED TO SEND FIRST WAYPOINT NAVIGATION DROP");
			}

			hit_count = 0;
		}

		// send drop back ball
		if(mission_repeat_counter == 3){
			erase_wp(3);
			servo_drop_wp(8, 3, wp);

			if(waypoint_push_client.call(waypoint_push) && waypoint_pull_client.call(waypoint_pull)){
				ROS_INFO("SECOND WAYPOINT NAVIGATION DROP SENT");
			}
			else {
				ROS_WARN("FAILED TO SEND SECOND WAYPOINT NAVIGATION DROP");
			}

			mission_repeat_counter = 0;
=======
		if(mission_flag) {
			ROS_INFO_ONCE("Mission program started");
			// increase counter if wp3 reached
			if(waypoint_reached == 1){
				++mission_repeat_counter;
			}
			
			// turn on vision node when wp3 has reached
			if(waypoint_reached == 1 && mission_repeat_counter == 1){
				vision_flag.data = true;
				vision_flag_publisher.publish(vision_flag);
			}
			else {
				vision_flag.data = false;
				vision_flag_publisher.publish(vision_flag);
				ROS_INFO_ONCE("Vision program stopped");
			}

			// dropzone found, confirm by wait for hit_point
			if((x_pixel && y_pixel) != 3000){
				++hit_count;
			}
			else {
				--hit_count;
			}

			// dropzone confirmed
			if(hit_count >= 3){
				
				ROS_INFO_ONCE("DROPZONE TARGET ACQUIRED. PROCEED TO EXECUTE DROPPING SEQUENCE");

				if(calc_mode % 2 == 0) {
					ros::param::get("/rasendriya/drag_coefficient", drag_coeff);
					dropping_offset = 0;
					calc_projectile_distance(dropping_offset, dropping_altitude, drag_coeff);
				}
				else {
					ros::param::get("/rasendriya/dropping_offset", dropping_offset);
				}

				calc_drop_coord(tgt_latx, tgt_lony, dropping_offset, calc_mode);
				
				// send do jump command to WP 1 from WP 4 after dropzone has found as WP 5
				wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
				wp.command = mavros_msgs::CommandCode::DO_JUMP;
				wp.is_current = false;
				wp.autocontinue = true;
				wp.param1 = 1;
				wp.param2 = 2;
				insert_wp(4, wp);

				// send plane attitude waypoint for dropping as WP 3 in WP 2
				wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
				wp.command = mavros_msgs::CommandCode::NAV_WAYPOINT;
				wp.is_current = false;
				wp.autocontinue = true;
				wp.param2 = 1;
				wp.param3 = 0;
				wp.x_lat = tgt_latx;
				wp.y_long = tgt_lony;
				wp.z_alt = dropping_altitude;
				insert_wp(2, wp);

				// send waypoint to drop front ball
				servo_drop_wp(7, 3, wp);

				if(waypoint_push_client.call(waypoint_push) && waypoint_pull_client.call(waypoint_pull)){
					ROS_INFO("FIRST WAYPOINT NAVIGATION DROP SENT");
				}
				else {
					ROS_WARN("FAILED TO SEND FIRST WAYPOINT NAVIGATION DROP");
				}

				hit_count = 0;
			}

			// send drop back ball
			if(mission_repeat_counter == 3){
				erase_wp(3);
				servo_drop_wp(8, 3, wp);

				if(waypoint_push_client.call(waypoint_push) && waypoint_pull_client.call(waypoint_pull)){
					ROS_INFO("SECOND WAYPOINT NAVIGATION DROP SENT");
				}
				else {
					ROS_WARN("FAILED TO SEND SECOND WAYPOINT NAVIGATION DROP");
				}

				mission_repeat_counter = 0;
			}
>>>>>>> b1ec7fec8e7b577cb23b8266008eaabdca61e7b1
		}

		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}

