#include "ros/ros.h"
#include "rasendriya/Dropzone.h"
#include "mavros_msgs/CommandLong.h"
#include "std_srvs/SetBool.h"
#include "mavros_msgs/StreamRate.h"

float x_dz = -3000;
float y_dz = -3000;

bool dropzone_target_callback(rasendriya::Dropzone::Request& dropzone_req, rasendriya::Dropzone::Response& dropzone_res){
	x_dz = dropzone_req.x;
	y_dz = dropzone_req.y;
	dropzone_res.status = true;
	return true;
}

bool trigger_servo(const int& servo_num, const int& pwm, ros::ServiceClient& _cmd_client){
	mavros_msgs::CommandLong do_set_servo;
	do_set_servo.request.command = 183;
	do_set_servo.request.param1 = servo_num-1;
	do_set_servo.request.param2 = pwm;

	if (_cmd_client.call(do_set_servo)) {
		ROS_INFO("Triggering servo %d", servo_num);
		return true;
	}
	else {
		ROS_WARN("Failed to trigger servo %d", servo_num);
		return false;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_servo");
	ros::NodeHandle nh;

	bool req_stream = false;
	bool req_vision = false;

  int loop_rate;
	ros::param::get("/rasendriya/loop_rate", loop_rate);

	ros::Rate rate(loop_rate);

	std_srvs::SetBool vision_flag;
	
	ros::ServiceClient vision_flag_cli = nh.serviceClient<std_srvs::SetBool>("/rasendriya/vision_flag");

	ros::ServiceClient set_stream_rate_cli = nh.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
	
	ros::ServiceClient set_servo_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command", 1);

	ros::ServiceServer dropzone_service = nh.advertiseService("/rasendriya/dropzone", dropzone_target_callback);

	int servo_pwm[2];
	nh.getParam("/test_servo/rasendriya/testing/servo_pwm/7", servo_pwm[0]);
	nh.getParam("/test_servo/rasendriya/testing/servo_pwm/8", servo_pwm[1]);
	/*
	ros::param::get("/rasendriya/testing/servo_pwm/7", servo_pwm[0]);
	ros::param::get("/rasendriya/testing/servo_pwm/8", servo_pwm[1]);
	*/
	ros::Duration(5.0).sleep();

	// set stream rate
	while(ros::ok()) {
		ROS_INFO_ONCE("Requesting stream and vision");
		if(!req_stream) {
			mavros_msgs::StreamRate stream_fcu;
			stream_fcu.request.stream_id = 0;
			stream_fcu.request.message_rate = 30;
			stream_fcu.request.on_off = true;
			if(set_stream_rate_cli.call(stream_fcu)) {
				ROS_INFO("Stream from FCU set!");
				req_stream = true;
			}
			else {
				ROS_ERROR("Failed to stream from FCU set, retrying");
			}
		}

		if(!req_vision) {
			vision_flag.request.data = true;
			if(vision_flag_cli.call(vision_flag)) {
				ROS_INFO("Starting vision program");
				req_vision = true;
			}
			else {
				ROS_ERROR("Failed to start vision program, retrying");
			}
		}

		if(req_vision && req_stream) {
			break;
		}

		ros::spinOnce();
		rate.sleep();
	}

	while(ros::ok()) {
		ROS_INFO_ONCE("Waiting for target");

		if((x_dz != -3000) && (y_dz != -3000)) {
			ROS_INFO("DROPZONE TARGET ACQUIRED. PROCEED TO EXECUTE DROPPING SEQUENCE");
			trigger_servo(7, servo_pwm[0], set_servo_client);
			trigger_servo(8, servo_pwm[1], set_servo_client);

			vision_flag.request.data = false;
			if(vision_flag_cli.call(vision_flag)) {
				ROS_INFO("Stopping vision program");
			}
			else {
				ROS_ERROR("Failed to stop vision program, retrying");
			}

			x_dz = -3000;
			y_dz = -3000;
		}
		ros::spinOnce();
    rate.sleep();
	}
	return 0;
}
