#include <iostream>

#include <functional>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

#include <mutex>
#include <string>
#include <vector>
#include <sdf/sdf.hh>
#include <ignition/math/Filter.hh>
#include <gazebo/common/Assert.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/transport/transport.hh>
#include "plugins/ArduPilotPlugin.hh"

#define MAX_MOTORS 255

using namespace gazebo;

// GZ_REGISTER_MODEL_PLUGIN(ArduPilotPlugin)

int main()
{
  std::cout << "Hello World!" << std::endl;
  return 0;
}
