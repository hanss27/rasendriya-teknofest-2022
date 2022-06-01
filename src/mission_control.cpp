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

#include "std_srvs/SetBool.h"

#include <math.h>
#include <bits/stdc++.h>

// REMINDER: wp_num STARTS FROM 0

int waypoint_reached = 0;

void waypoint_reached_callback(const mavros_msgs::WaypointReached& wp_reached){
	waypoint_reached = wp_reached.wp_seq;
}

// CALLBACKS //

float x_pixelc = -3000;
float y_pixel = -3000;
float alt, gps_hdg;
double gps_long, gps_lat;
float vel_y, vel_z;
bool mission_flag;

mavros_msgs::WaypointPush waypoint_push;

bool dropzone_target_callback(rasendriya::Dropzone::Request& dropzone_req, rasendriya::Dropzone::Response& dropzone_res){
	x_pixel = dropzone_req.x;
	y_pixel = dropzone_req.y;
	dropzone_res.status = true;
	return true;
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

void waypoint_list_callback(const mavros_msgs::WaypointList& wplist) {
	waypoint_push.request.waypoints = wplist.waypoints;
}

/*
	WAYPOINT MODIFIER APIs
*/

void insert_wp(const int& _wp_num, const mavros_msgs::Waypoint& _wp){	
	waypoint_push.request.waypoints.insert(waypoint_push.request.waypoints.begin() + _wp_num, _wp);
}

void erase_wp(const int& _wp_num){
	waypoint_push.request.waypoints.erase(waypoint_push.request.waypoints.begin() + _wp_num);
}

void swap_wp(const int& _wp_num, const mavros_msgs::Waypoint& _wp) {
	erase_wp(_wp_num);
	insert_wp(_wp_num, _wp);
}

void servo_drop_wp(const int& servo_ch, const int& wp_drop_num, mavros_msgs::Waypoint& _wp){
	_wp.frame = mavros_msgs::Waypoint::FRAME_GLOBAL_REL_ALT;
	_wp.command = mavros_msgs::CommandCode::DO_SET_SERVO;
	_wp.is_current = true;
	_wp.autocontinue = true;
	_wp.param1 = servo_ch;
	_wp.param2 = 1100;

	insert_wp(wp_drop_num, _wp);
}

// mathematical conversion APIs because c++ library doesn't have it. math.h sucks 

float radians(const float& _deg) {
	return _deg*(M_PI/180);
} 

float degrees(const float& _rad) {
	return _rad*(180/M_PI);
}

// projectile motion calculator API
#define gravity 9.81 // m/s^2

float calc_projectile_distance(const float& _drop_alt) {
	float _drop_offset = vel_y*sqrt(2*_drop_alt/gravity);
	return _drop_offset;
}

// coordinate calculator API
#define R_earth 6378.1*1e3

void calc_drop_coord(double& _tgt_latx, double& _tgt_lony, const float& _drop_offset){	
	float hdg = radians(gps_hdg);
	double lat = radians(gps_lat);
	double lon = radians(gps_long);

	float cam_angle, r_dist;
	double focal_length_x, focal_length_y;
	//const float pixel_to_mm = 0.2645;

	ros::param::get("/rasendriya/focal_length/x", focal_length_x);
	ros::param::get("/rasendriya/focal_length/x", focal_length_x);

	float X_meter = x_pixel*alt/focal_length_x;
	float Y_meter = y_pixel*alt/focal_length_y;
	
	r_dist = sqrt(pow(X_meter, 2) + pow(Y_meter + _drop_offset, 2));
	cam_angle = radians(atan2(X_meter, Y_meter));

	// using haversine law
	_tgt_latx = degrees(asin(sin(lat)*cos(r_dist/R_earth) + cos(lat)*sin(r_dist/R_earth)*cos(hdg+cam_angle)));
	_tgt_lony = degrees(lon + atan2(sin(hdg+cam_angle)*sin(r_dist/R_earth)*cos(lat) , (cos(r_dist/R_earth)-sin(lat)*sin(_tgt_latx))));
}

// MAIN FUNCTION //

int main(int argc, char **argv) {
	double tgt_latx, tgt_lony;

	bool vision_started = false;

	mavros_msgs::WaypointPull waypoint_pull;

	std_srvs::SetBool vision_flag;

	mavros_msgs::Waypoint wp;

	ros::init(argc, argv, "mission_control");
	ros::NodeHandle nh;

	ros::ServiceClient waypoint_push_client = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");

	ros::ServiceClient waypoint_pull_client = nh.serviceClient<mavros_msgs::WaypointPull>("/mavros/mission/pull");
	
	ros::ServiceClient vision_flag_client = nh.serviceClient<std_srvs::SetBool>("/rasendriya/vision_flag");

	ros::ServiceServer dropzone_service = nh.advertiseService("/rasendriya/dropzone", dropzone_target_callback);

	ros::Subscriber waypoint_list_sub = nh.subscribe("/mavros/mission/waypoints", 1, waypoint_list_callback);

	ros::Subscriber waypoint_reached_sub = nh.subscribe("/mavros/mission/reached", 1, waypoint_reached_callback);
	ros::Subscriber gps_coordinate_sub= nh.subscribe("/mavros/global_position/global", 1, gps_callback);
	ros::Subscriber alt_sub = nh.subscribe("/mavros/altitude", 1, alt_callback);
	ros::Subscriber gps_hdg_sub = nh.subscribe("/mavros/global_position/compass_hdg", 1, gps_hdg_callback);
	ros::Subscriber vel_sub = nh.subscribe("/mavros/global_position/gp_vel", 1, vel_callback);

	ros::Rate rate(25);
	
	// ROS LAUNCH PARAMETERS //
	float dropping_altitude;
	ros::param::get("/rasendriya/dropping_altitude", dropping_altitude);

	int wp_prepare_scan;
	int wp_drop[2];
	ros::param::get("/rasendriya/wp_drop_first", wp_drop[0]);
	ros::param::get("/rasendriya/wp_drop_second", wp_drop[1]);
	ros::param::get("/rasendriya/wp_prepare_scan", wp_prepare_scan);

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
				
		// turn on vision node when wp3 has reached
		if(!vision_started) {
			if(waypoint_reached == wp_prepare_scan - 1) {
				vision_flag.request.data = true;
				vision_flag_client.call(vision_flag);
				ROS_INFO_ONCE("Vision program started");
				vision_started = true;
			}
		}
		else {
			if(waypoint_reached == wp_prepare_scan + 1) {
				vision_flag.request.data = false;
				vision_flag_client.call(vision_flag);
				ROS_INFO_ONCE("Vision program stopped");
				vision_started = false;
			}
		}
		
		// dropzone confirmed
		if((x_pixel != -3000) && (y_pixel != -3000)){
			
			ROS_INFO_ONCE("DROPZONE TARGET ACQUIRED. PROCEED TO EXECUTE DROPPING SEQUENCE");

			dropping_altitude = waypoint_push.request.waypoints[wp_drop[0] - 1].z_alt;

			calc_drop_coord(tgt_latx, tgt_lony, calc_projectile_distance(dropping_altitude));
			
			// change WP NAV directly before dropping
			for(int i = 0; i <= 1; i++) {
				waypoint_push.request.waypoints[wp_drop[i] - 1].x_lat = tgt_latx;
				waypoint_push.request.waypoints[wp_drop[i] - 1].y_long = tgt_lony;
			}

			if(waypoint_push_client.call(waypoint_push) && waypoint_pull_client.call(waypoint_pull)){
				ROS_INFO("Image processed coordinates sent");
			}
			else {
				ROS_WARN("Failed to send image processed coordinates. Using written default coordinate");
			}

			x_pixel = NAN;
			y_pixel = NAN;
		}

		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}

