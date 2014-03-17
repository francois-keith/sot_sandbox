#include "start_controller.h"
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <ros/console.h>

// -*- C++ -*-
/*!
 * @file  MinimalStackOfTasks.cpp * @brief Module for controlling humanoid robot * $Date$
 *
 * $Id$ 
 */
#include <stdlib.h>
#include <dlfcn.h>
#include <cmath>
#include <fstream>
#include <exception>
#include <iomanip>
#include <iostream>
#include <sys/time.h>
#include <signal.h>
#include <boost/shared_ptr.hpp>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include <boost/algorithm/string.hpp>


//#define DEBUG
#ifdef DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x)
#endif
#define ODEBUG3(x) std::cout << x << std::endl

#define DBGFILE "/tmp/rtc-stack-of-tasks-comp"
#define RESETDEBUG5() { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::out);  \
DebugFile.close();}
#define ODEBUG5FULL(x) { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::app);   \
DebugFile << __FILE__ << ":" \
          << __FUNCTION__ << "(#" \
          << __LINE__ << "):" << x << std::endl; \
          DebugFile.close();}
#define ODEBUG5(x) { std::ofstream DebugFile; \
    DebugFile.open(DBGFILE,std::ofstream::app); \
DebugFile << x << std::endl; \
          DebugFile.close();}



MinimalStackOfTasks::MinimalStackOfTasks()
    // <rtc-template block="initializer">
  : started_(false)
  , initialize_library_(false)
    // </rtc-template>
{
  RESETDEBUG5()
}

MinimalStackOfTasks::~MinimalStackOfTasks()
{
}


void MinimalStackOfTasks::setRobot(ROBOT robot)
{
  if(robot == PR2)
  {
    ROS_INFO("Using the PR2 robot");
    robot_config_.libname="libsot_pr2.so";
  }
  else if (robot == HRP4)
  {
    ROS_INFO("Using the HRP4 robot");
    robot_config_.libname="libsot-hrp4-controller.so";
  }
  else if (robot == ROMEO)
  {
    ROS_INFO("Using the Romeo robot");
    robot_config_.libname="libsot-romeo-controller.so";
  }
}

std::string filename ("/tmp/rtc-log-time.txt");
std::ofstream logTimeFile (filename.c_str());

void MinimalStackOfTasks::LoadSot()
{
  char * sLD_LIBRARY_PATH;
  sLD_LIBRARY_PATH=getenv("LD_LIBRARY_PATH");
  ODEBUG5("LoadSot - Start " << sLD_LIBRARY_PATH);
  char * sPYTHONPATH;
  sPYTHONPATH=getenv("PYTHONPATH");
  ODEBUG5("PYTHONPATH:" << sPYTHONPATH );
  sPYTHONPATH=getenv("PYTHON_PATH");
  ODEBUG5("PYTHON_PATH:" << sPYTHONPATH);

  // Load the SotHRP2Controller library.
  void * SotHRP2ControllerLibrary = dlopen(robot_config_.libname.c_str(),
                                           RTLD_GLOBAL | RTLD_NOW);
  if (!SotHRP2ControllerLibrary) {
    std::ostringstream oss;
    oss << "Cannot load library: " << dlerror() << std::endl;
    ODEBUG5(oss.str());
    std::cerr << oss.str() << std::endl;
    throw oss.str();
  }
  ODEBUG5("Success in loading the library:" << robot_config_.libname);
  // reset errors
  dlerror();
  
  // Load the symbols.
  createSotExternalInterface_t * createHRP2Controller =
    (createSotExternalInterface_t *) dlsym(SotHRP2ControllerLibrary, 
                                           "createSotExternalInterface");
  ODEBUG5("createHRPController call "<< std::hex
          << std::setbase(10));
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    std::ostringstream oss;
    oss << "Cannot load symbol create: " << dlsym_error << std::endl;
    throw oss.str();
  }
  ODEBUG5("Success in getting the controller factory");
  
  // Create hrp2-controller
  try 
    {
      ODEBUG5("exception handled createHRP2Controller call "<< std::hex 
              << std::setbase(10));
      m_sotController = createHRP2Controller();
      ODEBUG5("After createHRP2Controller.");

    } 
  catch (std::exception &e)
    {
      ODEBUG5("Exception: " << e.what());
      std::ostringstream oss;
      oss << "Exception: " << e.what() << std::endl;
      throw oss.str();
    }
  ODEBUG5("LoadSot - End");
}


void
MinimalStackOfTasks::captureTime (timeval& t)
{
  gettimeofday (&t, NULL);
}

void
MinimalStackOfTasks::logTime (const timeval& t0, const timeval& t1)
{
  double dt =
    (t1.tv_sec - t0.tv_sec)
    + (t1.tv_usec - t0.tv_usec + 0.) / 1e6;

  logTimeFile << timeIndex_ << "   " << "   " << dt << std::endl;
  ++timeIndex_;
}

void MinimalStackOfTasks::start()
{
  ROS_INFO("Control started");
  started_ = true;
}

void MinimalStackOfTasks::onInitialize()
{
  if (!initialize_library_)
  {
    LoadSot();
    initialize_library_ = true;
  }
}

void MinimalStackOfTasks::setPosition(const std::vector<double> & pos)
{
  position_ = pos;
}

void MinimalStackOfTasks::fillSensors(std::map<std::string,dgsot::SensorValues> & sensor)
{
  if (position_.size() == 0)
    return;

  sensor["joints"].setName("joints");
  sensor["joints"].setValues(position_);
}

void MinimalStackOfTasks::onExecute()
{
//  ODEBUG(m_configsets.getActiveId());
  if (!started_)
    return;

  // Log control loop start time.
  captureTime (t0_);
  

  fillSensors(sensorsIn_);
  try
    {
      m_sotController->setupSetSensors(sensorsIn_);
      m_sotController->getControl(controlValues_);
    }
  catch (std::exception &e)
    {  ODEBUG5("Exception on Execute: " << e.what());throw e; }

  // Log control loop end time and compute time spent.
  captureTime (t1_);
  logTime (t0_, t1_);
  ODEBUG("onExecute - end");
  return;
}


bool callback(boost::shared_ptr<MinimalStackOfTasks>m, std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  m->start();
  return true;
}

//TODO: coder of the year !
boost::shared_ptr<MinimalStackOfTasks> m;

void positionCallback(const dynamic_graph_bridge_msgs::Vector & msg)
{
  if (msg.data.size() != 0)
  {
      std::vector<double> pos (msg.data.size());
      for(unsigned i=0; i<pos.size(); ++i)
        pos[i] = msg.data[i];
      m->setPosition(pos);
  }
}


int main (int argc, char** argv)
{
  ros::init(argc, argv, "sampling_learned_models");
  ros::NodeHandle nh;

  m.reset(new MinimalStackOfTasks);

  //substrack to feedback
  ros::Subscriber sub = nh.subscribe("positionFeedback", 1000, positionCallback);

  ros::ServiceServer service = nh.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>
    ("start_dynamic_graph", boost::bind(callback, m, _1, _2));
  std::string robot = "";
  int frequency = 200;
  if(! nh.getParam("/robot", robot))
  {
    ROS_ERROR("robot param not given");
  }
  
  boost::algorithm::to_lower(robot);

  if (robot == "pr2")
  {
    m->setRobot(PR2);
    frequency = 1000;
  }
  else if (robot == "hrp4")
  {
    m->setRobot(HRP4);
    frequency = 200;
  }
  else if (robot == "romeo")
  {
    m->setRobot(ROMEO);
    frequency = 200;
  }
  else
    ROS_ERROR("Robot not handled");

  m->onInitialize();

  ROS_INFO("Waiting for you to start start_dynamic_graph");
  ros::service::waitForService	("/start_dynamic_graph");

  ros::Rate loop_rate(frequency);
  while (ros::ok())
  {
    m->onExecute();
    ros::spinOnce();
    loop_rate.sleep();    
  }
}

