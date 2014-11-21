#include <example_bar.h>

//------------------------------------------------------------------------------
using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;

bool ExampleBar::isRosInit = false;

//------------------------------------------------------------------------------
ExampleBar* ExampleBar::instance(){
  static ExampleBar* instance = new ExampleBar();
  return instance;
}

//------------------------------------------------------------------------------
ExampleBar::ExampleBar(): ToolBar("ExampleBar"),
                            mes(*MessageView::mainInstance()),
                            os(MessageView::mainInstance()->cout()),
                            argc(), argv() 
{
  try
  {
    if (!isRosInit) 
    {
      ros::init(argc, argv, "example_plugin");
      isRosInit=true;
      cout << "Success: connecting roscore.." << endl;
    }
  }
  catch(...)
  {
    cout << "Error: ros init" << endl;
  }

  // ros nodehandle, topic, service init
  static ros::NodeHandle nh;
  
  //------------------------------------------------------------------------------
  addSeparator();

  addLabel(("[ExamplePlugin]"));

  addButton(("Test"), ("Test Button"))->
    sigClicked().connect(bind(&ExampleBar::onTestButtonClicked, this));
}

//------------------------------------------------------------------------------
ExampleBar::~ExampleBar()
{
}

//------------------------------------------------------------------------------
void ExampleBar::onTestButtonClicked()
{
  static boost::thread t(boost::bind(&ExampleBar::rosOn, this));
}

//------------------------------------------------------------------------------
void ExampleBar::rosOn()
{
  static ros::Rate loop_rate(10); // 0.1sec

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}

//------------------------------------------------------------------------------
