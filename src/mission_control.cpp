#include "ros/ros.h"

#include "rasendriya/Dropzone.h"

#include "mavros_msgs/WaypointReached.h"
#include "mavros_msgs/Waypoint.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/WaypointList.h"
#include "mavros_msgs/WaypointPull.h"
#include "mavros_msgs/CommandCode.h"
#include "mavros_msgs/State.h"
#include "mavros_msgs/StreamRate.h"
#include "mavros_msgs/Altitude.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float64.h"

#include "std_srvs/SetBool.h"

#include <math.h>
//#include <bits/stdc++.h>

// REMINDER: wp_num STARTS FROM 0

int waypoint_reached = 0;

void waypoint_reached_callback(const mavros_msgs::WaypointReached& wp_reached){
	waypoint_reached = wp_reached.wp_seq;
}

// CALLBACKS //

float x_pixel = -3000;
float y_pixel = -3000;
float gps_alt;
float alt, gps_hdg;
double gps_long, gps_lat;
float vel_x, vel_y, vel_z;
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
	gps_alt = gps_data.altitude;
}

void alt_callback(const mavros_msgs::Altitude& alt_data){
	alt = alt_data.relative;
}

void gps_hdg_callback(const std_msgs::Float64& gps_hdg_data){
	gps_hdg = gps_hdg_data.data;
}

void vel_callback(const geometry_msgs::TwistStamped& vel_data){
	vel_x = vel_data.twist.linear.y;
        vel_y = vel_data.twist.linear.x; // Flip vel_x with vel_y
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

double radians(const double& _deg) {
	return _deg*(M_PI/180);
} 

double degrees(const double& _rad) {
	return _rad*(180/M_PI);
}

// projectile motion calculator API
#define gravity 9.81 // m/s^2

float calc_projectile_distance(const float& _drop_alt) {
	float _drop_offset = vel_y*sqrt(2*_drop_alt/gravity);
	ROS_INFO("Speed: %f | Height: %f | Drop distance: %f", vel_y, _drop_alt, _drop_offset);
	return _drop_offset;
}

// camera transformation API (pinhole model)
void transform_camera(float& _X_meter, float& _Y_meter, ros::NodeHandle& __nh) {
	double focal_length_x, focal_length_y, principal_point_x, principal_point_y;
        
	__nh.getParam("/mission_control/rasendriya/camera/focal_length/x", focal_length_x);
	__nh.getParam("/mission_control/rasendriya/camera/focal_length/y", focal_length_y);
	__nh.getParam("/mission_control/rasendriya/camera/principal_point/x", principal_point_x);
	__nh.getParam("/mission_control/rasendriya/camera/principal_point/y", principal_point_y);

	ROS_INFO("X camera: %f | Y camera: %f | Altitude: %f", x_pixel, y_pixel, alt);

	_X_meter = (x_pixel - principal_point_x*alt)/focal_length_x;
	_Y_meter = (y_pixel - principal_point_y*alt)/focal_length_y;
}

// coordinate calculator API
#define R_earth 6378137 // in meters

void calc_drop_coord(double& _tgt_latx, double& _tgt_lony, const float& _drop_offset, ros::NodeHandle& _nh){	
	float hdg = radians(gps_hdg - 180);
	double lat = radians(gps_lat);
	double lon = radians(gps_long);
	
	float X_meter, Y_meter, cam_angle, r_dist;

	transform_camera(X_meter, Y_meter, _nh);

	r_dist = sqrt(pow(X_meter, 2) + pow(Y_meter + _drop_offset, 2));
	ROS_INFO("X: %f | Y: %f | Total distance: %f | Heading: %f", X_meter, Y_meter, r_dist, hdg);
	cam_angle = radians(atan2(X_meter, Y_meter));

	// using haversine law
	_tgt_latx = degrees(asin(sin(lat)*cos(r_dist/R_earth) + cos(lat)*sin(r_dist/R_earth)*cos(hdg+cam_angle)));
	_tgt_lony = degrees(lon + atan2(sin(hdg+cam_angle)*sin(r_dist/R_earth)*cos(lat) , (cos(r_dist/R_earth)-sin(lat)*sin(_tgt_latx))));
}

// MAIN FUNCTION //

int main(int argc, char **argv) {
	double tgt_latx, tgt_lony;

	float dropping_altitude;

	bool vision_started = false;

	mavros_msgs::WaypointPull waypoint_pull;

	std_srvs::SetBool vision_flag;

	mavros_msgs::Waypoint wp;

	ros::init(argc, argv, "mission_control");
	ros::NodeHandle nh;

	ros::ServiceClient waypoint_push_cli = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros/mission/push");

	ros::ServiceClient waypoint_pull_cli = nh.serviceClient<mavros_msgs::WaypointPull>("/mavros/mission/pull");

	ros::ServiceClient set_stream_rate_cli = nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
	
	ros::ServiceClient vision_flag_cli = nh.serviceClient<std_srvs::SetBool>("/rasendriya/vision_flag");

	ros::ServiceServer dropzone_srv = nh.advertiseService("/rasendriya/dropzone", dropzone_target_callback);

	ros::Subscriber waypoint_list_sub = nh.subscribe("/mavros/mission/waypoints", 1, waypoint_list_callback);
	ros::Subscriber waypoint_reached_sub = nh.subscribe("/mavros/mission/reached", 1, waypoint_reached_callback);
	ros::Subscriber gps_coordinate_sub = nh.subscribe("/mavros/global_position/global", 1, gps_callback);
	ros::Subscriber alt_sub = nh.subscribe("/mavros/altitude", 1, alt_callback);
	ros::Subscriber gps_hdg_sub = nh.subscribe("/mavros/global_position/compass_hdg", 1, gps_hdg_callback);
	ros::Subscriber vel_sub = nh.subscribe("/mavros/local_position/velocity_body", 1, vel_callback);
	
	ROS_INFO("Loading ROS params");

	int loop_rate;
	ros::param::get("/rasendriya/loop_rate", loop_rate);
	ROS_INFO("Loop rate used: %d", loop_rate);

	ros::Rate rate(loop_rate);

	int wp_prepare_scan;
	int wp_drop[2];
  
	nh.getParam("/mission_control/rasendriya/wp_drop_first", wp_drop[0]);
	nh.getParam("/mission_control/rasendriya/wp_drop_second", wp_drop[1]);
	nh.getParam("/mission_control/rasendriya/wp_prepare_scan", wp_prepare_scan);
	ROS_INFO("Start scanning waypoint: %d", wp_prepare_scan);
	ROS_INFO("First dropping waypoint: %d", wp_drop[0]);
	ROS_INFO("Second dropping waypoint: %d", wp_drop[1]);
	
	ROS_INFO("ROS params loading completed");

	// first WP loading from FCU. Ensures that companion computer has the same waypoints as FCU
	while(ros::ok() && (waypoint_push.request.waypoints.size() == 0)) {
		ros::spinOnce();
		rate.sleep();
	}
	ROS_INFO("Number of loaded waypoints: %d", int(waypoint_push.request.waypoints.size()));

	// set stream rate
	while(ros::ok()) {
		mavros_msgs::StreamRate stream_fcu;
		stream_fcu.request.stream_id = 0;
		stream_fcu.request.message_rate = 30;
		stream_fcu.request.on_off = true;
		if(set_stream_rate_cli.call(stream_fcu)) {
			ROS_INFO("Stream from FCU set!");
			break;
		}
		else {
			ROS_ERROR("Failed to stream from FCU set, retrying");
		}

		ros::spinOnce();
		rate.sleep();
	}

	while(ros::ok()) {
		
		ROS_INFO_ONCE("Mission program ready");		
		// turn on vision node when wp3 has reached
		if(!vision_started) {
			if(waypoint_reached == wp_prepare_scan) {
				vision_flag.request.data = true;
				if(vision_flag_cli.call(vision_flag)) {
					ROS_INFO("Starting vision program");
					vision_started = true;
				}
				else {
					ROS_ERROR("Failed to start vision program, retrying");
				}
			}
		}
		else {
			if( (waypoint_reached == wp_prepare_scan + 1) && vision_flag.request.data) {
				vision_flag.request.data = false;
				if(vision_flag_cli.call(vision_flag)) {
					ROS_INFO("Final scanning waypoint reached. Stopping vision program");
					vision_started = false;
				}
				else {
					ROS_ERROR("Failed to stop vision program, retrying");
				}
			}
		}
		
		// dropzone confirmed
		if((x_pixel != -3000) && (y_pixel != -3000)){
			
			ROS_INFO("DROPZONE TARGET ACQUIRED. EXECUTING DROPPING SEQUENCE");

			dropping_altitude = waypoint_push.request.waypoints[wp_drop[0]].z_alt;

			calc_drop_coord(tgt_latx, tgt_lony, calc_projectile_distance(dropping_altitude), nh);
			
			// change WP NAV directly before dropping
			ROS_INFO("Updating waypoints");
			for(int i = 0; i <= 1; i++) {
				waypoint_push.request.waypoints[wp_drop[i]].x_lat = int(tgt_latx * 1e7); // 1e7 is mavlink coordinate integer format
				waypoint_push.request.waypoints[wp_drop[i]].y_long = int(tgt_lony * 1e7);
				ROS_INFO("WP: %d | Latitude: %f | Longitude: %f", wp_drop[i], tgt_latx, tgt_lony);
			}

			if(waypoint_push_cli.call(waypoint_push) && waypoint_pull_cli.call(waypoint_pull)){
				ROS_INFO("Image processed coordinates sent!");
				vision_flag.request.data = false;
				if(vision_flag_cli.call(vision_flag)) {
					ROS_INFO("Stopping vision program. Hibernating");
					x_pixel = -3000;
					y_pixel = -3000;
					waypoint_reached =+ 1;
				}
				else {
					ROS_ERROR("Failed to stop vision program, retrying");
				}
			}
			else {
				ROS_WARN("Failed to send image processed coordinates, retrying");
			}

		}

		ros::spinOnce();
		rate.sleep();
	}
	
	return 0;
}

