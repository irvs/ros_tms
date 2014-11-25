#include <example_bar.h>

#include <cnoid/Plugin>
#include <cnoid/ItemManager>
#include <cnoid/BodyMotionItem>

#include <iostream>
#include <string>
#include <stdlib.h>

using namespace cnoid;
using namespace grasp;

namespace 
{

class ExamplePlugin : public Plugin
{
  public:

  ExamplePlugin() : Plugin("Example")
  {
  }

  bool initialize()
  {
    addToolBar(grasp::ExampleBar::instance());
    return true;
  }
};

}

CNOID_IMPLEMENT_PLUGIN_ENTRY(ExamplePlugin);
