#include <iostream>
#include <math.h>
#define png_infopp_NULL (png_infopp)NULL
#define int_p_NULL (int*)NULL
#define png_bytep_NULL (png_bytep)NULL
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>
#include<string>
#include <boost/shared_ptr.hpp>
#include <sdf/sdf.hh>
#include <ignition/math/Vector3.hh>
#include "gazebo/transport/transport.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/common/common.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"

  struct dimensions{
    int topleft_x;
    int topleft_y;
    int topright_x;
    int topright_y;
    int bottomleft_x;
    int bottomleft_y;
    int bottomright_x;
    int bottomright_y;
    double resolution;
    int height;

  };
namespace gazebo
{

class CollisionMapCreator : public WorldPlugin
{
    physics::WorldPtr world;
  public: void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
  {
  	world = _parent;
    func();
  }
  public:void func(){
  	
    struct dimensions msg={-10,10,10,10,-10,-10,10,-10,0.01,10};
    double dX_vertical = msg.topleft_x - msg.bottomleft_x; //hardcode
    double dY_vertical = msg.topleft_y - msg.bottomleft_y;//hardcode
    double mag_vertical = sqrt(dX_vertical * dX_vertical + dY_vertical * dY_vertical); //eucledian distance
    dX_vertical = msg.resolution * dX_vertical / mag_vertical; //hardcode
    dY_vertical = msg.resolution * dY_vertical / mag_vertical; //hardcode

    double dX_horizontal = msg.topright_x - msg.topleft_x; //hardcode
    double dY_horizontal = msg.topright_y - msg.topleft_y; //hardcode
    double mag_horizontal = sqrt(dX_horizontal * dX_horizontal + dY_horizontal * dY_horizontal);
    dX_horizontal = msg.resolution * dX_horizontal / mag_horizontal; //hardcode
    dY_horizontal = msg.resolution * dY_horizontal / mag_horizontal; //hardcode

    int count_vertical = mag_vertical / msg.resolution;
    int count_horizontal = mag_horizontal / msg.resolution;

    if (count_vertical == 0 || count_horizontal == 0)
    {
      std::cout << "Image has a zero dimensions, check coordinates"
                << std::endl;
      return;
    }
    double x,y;

    boost::gil::gray8_pixel_t fill(255); //hardcode
    boost::gil::gray8_pixel_t blank(255);
    boost::gil::gray8_image_t image(count_horizontal, count_vertical);

    double dist;
    std::string entityName;
    ignition::math::Vector3d start, end;
    start.Z(msg.height);//hardcode
    end.Z(0.001);

#if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::PhysicsEnginePtr engine = world->Physics();
#else
    gazebo::physics::PhysicsEnginePtr engine = world->GetPhysicsEngine(); //Return the physics engine.
#endif
    engine->InitForThread();
    gazebo::physics::RayShapePtr ray = boost::dynamic_pointer_cast<gazebo::physics::RayShape>(engine->CreateShape("ray", gazebo::physics::CollisionPtr()));

    std::cout << "Rasterizing model and checking collisions" << std::endl;
    boost::gil::fill_pixels(image._view, blank);

    for (int i = 0; i < count_vertical; ++i)
    {
      std::cout << "Percent complete: " << i * 100.0 / count_vertical
                << std::endl;
      x = i * dX_vertical + msg.bottomleft_x; //hardcode
      y = i * dY_vertical + msg.bottomleft_y; //hardcode
      for (int j = 0; j < count_horizontal; ++j)
      {
        x += dX_horizontal;
        y += dY_horizontal;

        start.X(x);
        end.X(x);
        start.Y(y);
        end.Y(y);
        ray->SetPoints(start, end);
        ray->GetIntersection(dist, entityName);
        if (!entityName.empty())
        {
          image._view(i,j) = fill;
        }
      }
    }

   std::cout << "Completed calculations, writing to image" << std::endl;
   boost::gil::gray8_view_t view = image._view;
   std::string fname="/home/tushar/occupancyMap.png";
   boost::gil::png_write_view(fname.c_str(), view); 

  }
};
// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(CollisionMapCreator)
}
