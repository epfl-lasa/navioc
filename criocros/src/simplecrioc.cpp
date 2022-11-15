#include <criocros/OCreply.h>
#include <criocros/OCcall.h>
#include <criocros/configcrioc.hpp>
#include <frame_msgs/TrackedPersons.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <vector>
#include <string>
#define USE_MATH_CONSTANTS
#include <cmath>
#include <iostream>
#include <fstream>

using std::vector;
using std::string;
using tf2::Quaternion;
using tf2::Vector3;
using tf2::Transform;

struct RobotStateEstimator
{
	RobotStateEstimator(float alpha, float beta, float pAlpha)
	: empty(true)
	, alpha(alpha)
	, beta(beta)
	, pAlpha(pAlpha) { }

	void update(const ros::Time& tObs, float* pObs, float* vObs)
	{
		if (false) //(!empty)
		{
			float dt((tObs - t).toSec());
			for (unsigned int i = 0; i != 3; i++)
			{
				float vPred(v[i] + dt*a[i]);
				float vResiduum(vObs[i] - vPred);
				v[i] = vPred + alpha*vResiduum;
				a[i] += beta*vResiduum/dt;

				float pPred(p[i] + dt*v[i]);
				float pResiduum(pObs[i] - pPred);
				p[i] = pPred + pAlpha*pResiduum;
			}
		}
		else
		{
			empty = false;
			for (unsigned int i = 0; i != 3; i++)
			{
				p[i] = pObs[i];
				v[i] = vObs[i];
				a[i] = 0.f;
			}
		}
		t = tObs;
	}

	bool empty;
	ros::Time t;
	float p[3], v[3], a[3];
	const float alpha, beta, pAlpha;
};

struct VDesFrame
{
	VDesFrame() : empty(true) { }
	bool empty;
	ros::Time t;
	float v[2];
};

struct TrajectoryFrame
{
	TrajectoryFrame()
	: empty(true)
	, dt(ConfigCrioc::h_step) { }

	bool empty;
	ros::Time t;
	float Px[ConfigCrioc::T_horizon + 1];
	float Py[ConfigCrioc::T_horizon + 1];
	float Vx[ConfigCrioc::T_horizon + 1];
	float Vy[ConfigCrioc::T_horizon + 1];
	float Ax[ConfigCrioc::T_horizon];
	float Ay[ConfigCrioc::T_horizon];
	const float dt;
};

struct LinearState
{
	float p[2], v[2], a[2];
};

struct CrowdFrame
{
	CrowdFrame() : empty(true) { }
	bool empty;
	ros::Time t;
	vector<LinearState> s;
};

RobotStateEstimator robot(0.5f, 1.f, 0.5f);
VDesFrame vDesRob;
TrajectoryFrame refRob;
CrowdFrame crowd;

bool constantCommand(true);
ros::Duration dtTolVDes(0.1);
ros::Duration dtTolRobot(0.1);
ros::Duration dtTolRef(2.0);
ros::Time t0;
const float maxV(1.5), maxW(1.5), minV(0.1), kV(0.f), kWPhi(0.f), kWXY(0.f);
const float vMultiplier(1.f), wMultiplier(1.f);
bool adaptiveFeedforward(true), wFromlateralAcceleration(false);
float desiredSpeed(0.5f);

/*struct Observation
{
	Observation(float* p, float* v, const ros::Time& t)
	: p { p[0], p[1], p[2] }
	, v { v[0], v[1], v[2] }
	, t(t) { }
	
	float p[3], v[3];
	ros::Time t;
};*/

//vector<Observation> samples;

void receiveOdom(const nav_msgs::Odometry::ConstPtr& msg)
{
	Transform rot_odom_pose(Quaternion(msg->pose.pose.orientation.x, 
		msg->pose.pose.orientation.y, msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w));

	float omega = (rot_odom_pose*Vector3(msg->twist.twist.angular.x,
		msg->twist.twist.angular.y, msg->twist.twist.angular.z)).getZ();

	Vector3 velocity(rot_odom_pose*Vector3(msg->twist.twist.linear.x,
		msg->twist.twist.linear.y, msg->twist.twist.linear.z));

	float vObs[3] = { velocity.getX(), velocity.getY(), omega };

	const Vector3& basisX(rot_odom_pose.getBasis().getColumn(0));
	float phi(std::atan2(basisX.getY(), basisX.getX()));

	float pObs[3] = {msg->pose.pose.position.x, msg->pose.pose.position.y, phi};

	robot.update(msg->header.stamp, pObs, vObs);
	//samples.push_back(Observation(pObs, vObs, msg->header.stamp));
}

void receiveVDes(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
	if (!robot.empty)
	{
		float cos = std::cos(robot.p[2]);
		float sin = std::sin(robot.p[2]);
		vDesRob.v[0] = cos*msg->data[1] - sin*msg->data[2];
		vDesRob.v[1] = sin*msg->data[1] + cos*msg->data[2];
		vDesRob.empty = false;
		vDesRob.t = ros::Time::now();
	}
}

void receiveOCreply(const criocros::OCreply::ConstPtr& msg)
{
	refRob.t = msg->header.stamp;
	refRob.empty = false;
	for (unsigned int i = 1; i != (ConfigCrioc::T_horizon + 1); i++)
	{
		 refRob.Px[i] = msg->Px[i];
		 refRob.Py[i] = msg->Py[i];
		 refRob.Vx[i] = msg->Vx[i];
		 refRob.Vy[i] = msg->Vy[i];
		 refRob.Ax[i-1] = msg->Ax[i-1];
		 refRob.Ay[i-1] = msg->Ay[i-1];
	}
	refRob.Px[0] = msg->Px[0];
	refRob.Py[0] = msg->Py[0];
	refRob.Vx[0] = msg->Vx[0];
	refRob.Vy[0] = msg->Vy[0];
}

void receiveTracks(const frame_msgs::TrackedPersons::ConstPtr& msg)
{
	crowd.t = msg->header.stamp;
	crowd.s.resize(msg->tracks.size());
	for (unsigned int i = 0; i != msg->tracks.size(); i++)
	{
		crowd.s[i].p[0] = msg->tracks[i].pose.pose.position.x;
		crowd.s[i].p[1] = msg->tracks[i].pose.pose.position.y;
		crowd.s[i].v[0] = msg->tracks[i].twist.twist.linear.x;
		crowd.s[i].v[1] = msg->tracks[i].twist.twist.linear.y;
		crowd.s[i].a[0] = 0.0;
		crowd.s[i].a[1] = 0.0;
	}
	crowd.empty = false;
}

/*{
	float a(2.f*std::atan2(1.f, msg->pose.pose.orientation.w));
	float pObs[3] = { msg->pose.pose.position.x,  msg->pose.pose.position.y, a};
	float wMag(std::sqrt(msg->twist.twist.angular.x*msg->twist.twist.angular.x +
		msg->twist.twist.angular.y*msg->twist.twist.angular.y +
		msg->twist.twist.angular.z*msg->twist.twist.angular.z));

	float vMag(std::sqrt(msg->twist.twist.linear.x*msg->twist.twist.linear.x +
		msg->twist.twist.linear.y*msg->twist.twist.linear.y +
		msg->twist.twist.linear.z*msg->twist.twist.linear.z));

	float w = wMag;
	if ( msg->twist.twist.angular.z < 0.f)
	{
		w = -w;
	}
	float v = vMag;
	if ( msg->twist.twist.linear.x < 0.f)
	{
		v = -v;
	}
	float vObs[3] = {std::cos(pObs[2])*v, std::sin(pObs[2])*v, w};

	robot.update(msg->header.stamp, pObs, vObs);
}*/

/*
void exportSamples()
{
  	std::ofstream myfile;
  	myfile.open ("odomSamples.txt");

  	for (auto& s : samples)
  	{
  		myfile << s.p[0] << ", " << s.p[1] << ", " << s.p[2] << ", ";
  		myfile << s.v[0] << ", " << s.v[1] << ", " << s.v[2] << ", ";
  		myfile << (s.t - t0).toSec() << std::endl;
  	}

  	myfile.close();
}*/

bool updateOCcall(const ros::Time& t, criocros::OCcall& msg)
{
	if ((!constantCommand) && (vDesRob.empty || t - vDesRob.t > dtTolVDes || vDesRob.t - t > dtTolVDes) ||
		robot.empty || t - robot.t > dtTolRobot || robot.t - t > dtTolRobot)
	{
		return false;
	}

	msg.header.stamp = t;

	unsigned int Du = 2;
	if (!crowd.empty)
	{
		Du += crowd.s.size()*2;
	}

	msg.v_des = vector<float>(Du, 0.f);
	if (constantCommand)
	{
		msg.v_des[0] = desiredSpeed;
	}
	else
	{
		msg.v_des[0] = vDesRob.v[0];
		msg.v_des[1] = vDesRob.v[1];
	}

	msg.x.resize(Du*2);
	for (unsigned int j = 0; j != 2; j++)
	{
		msg.x[j] = robot.p[j];
		msg.x[Du + j] = robot.v[j];
	}
	if (!crowd.empty)
	{
		for (unsigned int i = 0; i != crowd.s.size(); i++)
		{
			//float dx = robot.p[0] - crowd.s[i].p[0];
			//float dy = robot.p[1] - crowd.s[i].p[1];
			//float dRob = std::sqrt(dx*dx + dy*dy);
			for (unsigned int j = 0; j != 2; j++)
			{
				msg.x[2 + i*2 + j] = crowd.s[i].p[j];
				//msg.x[i*2 + j] += 0.5*(robot.p[j] - crowd.s[i].p[j])/dRob;
				msg.x[Du + 2 + i*2 + j] = crowd.s[i].v[j];

				msg.v_des[2 + i*2 + j] = crowd.s[i].v[j];
			}
		}
	}

	return true;
}

float clip(float value, float absBound)
{
	if (value > absBound)
	{
		value = absBound;
	}
	else if (value < -absBound)
	{
		value = -absBound;
	}
	return value;
}

void computeCommand(const ros::Time& t, float& v, float& w)
{
	float dt_total = (t - refRob.t).toSec();
	unsigned int i = dt_total/refRob.dt;
	if (i >= ConfigCrioc::T_horizon)
	{
		i = ConfigCrioc::T_horizon - 1;
	}
	float dt_ext = dt_total - i*refRob.dt;

	float Ax = refRob.Ax[i];
	float Ay = refRob.Ay[i];
	float Vx = refRob.Vx[i] + dt_ext*Ax;
	float Vy = refRob.Vy[i] + dt_ext*Ay;
	//float Px = refRob.Px[i] + dt_ext*refRob.Vx[i] + dt_ext*dt_ext*Ax/2.f;
	//float Py = refRob.Py[i] + dt_ext*refRob.Vy[i] + dt_ext*dt_ext*Ay/2.f;


	float phi = robot.p[2];
	v = std::cos(phi)*Vx + std::sin(phi)*Vy;	

	float v2(Vx*Vx + Vy*Vy);
	if (v2 < 0.09f)
	{
		v2 = 0.09f;
	}

	//v = std::sqrt(v2);

	float vxa(Vx*Ay - Vy*Ax);

	w = vxa/v2;
}
	/*
	unsigned int i = (t - refRob.t).toSec()/refRob.dt;
	i += 9;
	if (desiredSpeed > 0.75)
	{
		i -= 3;
	}

	float phi(refRob.Pp[i]);
	if (adaptiveFeedforward)
	{
		phi = robot.p[2];
	}
	v = std::cos(phi)*refRob.Vx[i] + std::sin(phi)*refRob.Vy[i];	
	
	if (wFromlateralAcceleration)
	{
		float aLateral = -std::sin(robot.p[2])*refRob.Ax[i] + std::cos(robot.p[2])*refRob.Ay[i];
		float vDenomFF = v;
		if (vDenomFF < minV)
		{
			vDenomFF = minV;
		}
		w = aLateral/vDenomFF;
	}
	else
	{
		w = refRob.Vp[i];
	}

	v*= vMultiplier;
	w*= wMultiplier;

	float cos = std::cos(robot.p[2]);
	float sin = std::sin(robot.p[2]);
	float d = -sin*(refRob.Px[i] - robot.p[0]) + cos*(refRob.Py[i] - robot.p[1]);
	float l = cos*(refRob.Px[i] - robot.p[0]) + sin*(refRob.Py[i] - robot.p[1]);

	v += kV*l;
	v = clip(v, maxV);
	float vDenom = v;
	if (vDenom < minV)
	{
		vDenom = minV;
	}
	w += kWXY*d/vDenom;

	float angularDifference = refRob.Pp[i] - robot.p[2];
	while (angularDifference < -M_PI)
	{
		angularDifference += 2.f*M_PI;
	}
	while (angularDifference > M_PI)
	{
		angularDifference -= 2.f*M_PI;
	}

	w += kWPhi*angularDifference;

	w = clip(w, maxW);
	*/

bool updateQoloCommand(const ros::Time& t, std_msgs::Float32MultiArray& msg)
{
	if (refRob.empty || t - refRob.t > dtTolRef || refRob.t - t > dtTolRef ||
		robot.empty || t - robot.t > dtTolRobot || robot.t - t > dtTolRobot)
	{
		return false;
	}

	float v, w;
	computeCommand(t, v, w);

	msg.data[0] = (t - t0).toSec();
	msg.data[1] = clip(v, maxV);
	msg.data[2] = clip(w, maxW);

	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "simplecrioc");
	ros::NodeHandle node;
	ros::Rate loop_rate(300);

	ros::Subscriber odomSub(node.subscribe("t265/odom/sample", 1, receiveOdom));
	ros::Subscriber phoneSub(node.subscribe("qolo/user_commands", 1, receiveVDes));
	ros::Subscriber OCsub(node.subscribe("oc_reply", 1, receiveOCreply));
	ros::Subscriber pedSub(node.subscribe("rwth_tracker/tracked_persons", 1,
		receiveTracks));

	ros::Publisher OCcaller(node.advertise<criocros::OCcall>("oc_call", 1));
	criocros::OCcall msgOCcall;
	msgOCcall.header.seq = 0;
	msgOCcall.header.frame_id = "tf_qolo_world";

	ros::Publisher QOLOcommander(node.advertise<std_msgs::Float32MultiArray>(
		"qolo/remote_commands", 1));
	std_msgs::Float32MultiArray msgQoloCommand;
	msgQoloCommand.data.resize(3);

	t0 = ros::Time::now();
	while (ros::ok())
	{
		ros::spinOnce();

		ros::Time t(ros::Time::now());

		if (updateOCcall(t, msgOCcall))
		{
			OCcaller.publish(msgOCcall);
		}

		if (updateQoloCommand(t, msgQoloCommand))
		{
			QOLOcommander.publish(msgQoloCommand);
		}

		loop_rate.sleep();	
	}

	//exportSamples();
}
