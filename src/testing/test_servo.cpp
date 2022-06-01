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

bool trigger_servo(int servo_num, ros::ServiceClient& _svo_client){
	mavros_msgs::CommandLong do_set_servo;
	do_set_servo.request.broadcast = true;
	do_set_servo.request.command = 183;
	do_set_servo.request.param1 = servo_num;
	do_set_servo.request.param2 = 1100;

	if (_svo_client.call(do_set_servo)) {
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

	int hit_count;

	
	ros::ServiceClient set_servo_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command", 1);

	ros::ServiceServer dropzone_service = nh.advertiseService("/rasendriya/dropzone", dropzone_target_callback);
	ros::Rate rate(25);

	while(ros::ok()) {

		ROS_INFO_ONCE("Test servo program started");
		//std::string str1 = std::to_string(x_dz);

		if((x_dz != -3000) && (y_dz != -3000)) {
			ROS_INFO_ONCE("DROPZONE TARGET ACQUIRED. PROCEED TO EXECUTE DROPPING SEQUENCE");
			trigger_servo(6, set_servo_client);
			trigger_servo(7, set_servo_client);
		}
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
