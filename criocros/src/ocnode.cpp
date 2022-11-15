#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"
#include <ros/ros.h>
#include <ros/time.h>
#include <criocros/OCreply.h>
#include <criocros/OCcall.h>
#include <criocros/configcrioc.hpp>

using std::vector;
using namespace matlab::engine;
using namespace matlab::data;

MATLABEngine* matl;
ArrayFactory* factory;

ros::Publisher publisher;

//vector<double> U0(ConfigCrioc::T_horizon, 0.0);
TypedArray<double>* Ux_1_init(0);//({ConfigCrioc::T_horizon, 1}, U0.begin(), U0.end());
TypedArray<double>* Uy_1_init(0);//({ConfigCrioc::T_horizon, 1}, U0.begin(), U0.end());

void process_message(const criocros::OCcall::ConstPtr& msg)
{
    TypedArray<double> x(factory->createArray<double>({ 1, msg->x.size() }));
    unsigned int i = 0;
    for (auto& z : x)
    	z = msg->x[i++];

    if (!Ux_1_init)
    {
        Ux_1_init = new TypedArray<double>(factory->createArray<double>({ConfigCrioc::T_horizon, 1}));
        Uy_1_init = new TypedArray<double>(factory->createArray<double>({ConfigCrioc::T_horizon, 1}));
        for (unsigned int i = 0; i != ConfigCrioc::T_horizon; ++i)
        {
            (*Ux_1_init)[i] = 0.0;
            (*Uy_1_init)[i] = 0.0;
        }
    }
    //TypedArray<double> u_1_init(factory->createArray<double>({ 1, msg->u_1_init.size() }));
    //i = 0;
    //for (auto& z : v_des)
    //    z = msg->v_des[i++];

    TypedArray<double> v_des(factory->createArray<double>({ 1, msg->v_des.size() }));
    i = 0;
    for (auto& z : v_des)
    	z = msg->v_des[i++];

    vector<Array> args({x, *Ux_1_init, *Uy_1_init, v_des});

    vector<Array> res = matl->feval(u"optimalcontrol", 6, args);

    TypedArray<double> Px_ref(res[0]);
    TypedArray<double> Py_ref(res[1]);
    TypedArray<double> Vx_ref(res[2]);
    TypedArray<double> Vy_ref(res[3]);
    TypedArray<double> Ax_ref(res[4]);
    TypedArray<double> Ay_ref(res[5]);

	criocros::OCreply reply;
    reply.Px = vector<float>(Px_ref.begin(), Px_ref.end());
    reply.Py = vector<float>(Py_ref.begin(), Py_ref.end());
    reply.Vx = vector<float>(Vx_ref.begin(), Vx_ref.end());
    reply.Vy = vector<float>(Vy_ref.begin(), Vy_ref.end());
    reply.Ax = vector<float>(Ax_ref.begin(), Ax_ref.end());
    reply.Ay = vector<float>(Ay_ref.begin(), Ay_ref.end());
    
    for (unsigned int i = 0; i != ConfigCrioc::T_horizon; ++i)
    {
        (*Ux_1_init)[i] = Ax_ref[i];
        (*Uy_1_init)[i] = Ay_ref[i];
    }

	reply.header.seq = msg->header.seq;
	reply.header.stamp = msg->header.stamp;
	reply.header.frame_id = msg->header.frame_id;

    publisher.publish(reply);
}

void add_repo_paths()
{
    matl->feval(u"addpath", factory->createCharArray("Features2"));
    matl->feval(u"addpath", factory->createCharArray("Crowdworld2"));
    matl->feval(u"addpath", factory->createCharArray("cioc/Reward"));
    matl->feval(u"addpath", factory->createCharArray("cioc/FastHess"));
    matl->feval(u"addpath", factory->createCharArray("cioc/Utilities/minFunc"));
}

void change_MATLAB_directory(char* path)
{
    CharArray arg = factory->createCharArray(path);
//        "~/github/errpnav/matlab/oc_cpp");

    matl->feval(u"cd", arg);
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        ROS_ERROR("Usage: ./ocnode path/to/oc_cpp");
        return 0;
    }

    std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB();
    matl = matlabPtr.get();
    ROS_INFO("Started MATLAB Engine");

    //Create MATLAB data array factory
    ArrayFactory f;
    factory = &f;

    change_MATLAB_directory(argv[1]);

    add_repo_paths();

	ros::init(argc, argv, "optimal_controller");
	ros::NodeHandle node;

	ros::Rate(0.25).sleep();

	publisher = node.advertise<criocros::OCreply>("oc_reply", 1);

	ros::Subscriber subscriber = node.subscribe("oc_call",
		1, process_message);

    ROS_INFO("Setup is ready.");

	ros::spin();

    if (Ux_1_init)
    {
        delete Ux_1_init;
    }
    if (Uy_1_init)
    {
        delete Uy_1_init;
    }
}
