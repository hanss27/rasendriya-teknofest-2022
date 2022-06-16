#include "ros/ros.h"
#include "rasendriya/Dropzone.h"
#include "mavros_msgs/CommandLong.h"
#include "std_srvs/SetBool.h"

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
		ROS_INFO("SERVO %d IS TRIGGERED", servo_num);
		return true;
	}
	else {
		ROS_WARN("FAILED TO TRIGGER SERVO %d", servo_num);
		return false;
	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "test_servo");
	ros::NodeHandle nh;

        ros::Rate rate(30);
	
	std_srvs::SetBool vision_flag;
	
	ros::ServiceClient vision_flag_cli = nh.serviceClient<std_srvs::SetBool>("/rasendriya/vision_flag");

	ros::ServiceClient set_servo_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command", 1);

	ros::ServiceServer dropzone_service = nh.advertiseService("/rasendriya/dropzone", dropzone_target_callback);
	
	ros::Duration(5.0).sleep();
	
	vision_flag.request.data = true;
	vision_flag_cli.call(vision_flag);
	ROS_INFO_ONCE("Calling vision program");

	while(ros::ok()) {
		if((x_dz != -3000) && (y_dz != -3000)) {
			ROS_INFO_ONCE("DROPZONE TARGET ACQUIRED. PROCEED TO EXECUTE DROPPING SEQUENCE");
			trigger_servo(7, 1600, set_servo_client);
			trigger_servo(8, 1600, set_servo_client);
			ros::shutdown();
		}
		ros::spinOnce();
                rate.sleep();
	}
	return 0;
}
