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
                            trc_(*TmsRpController::instance()),
                            argc(), argv() 
{
  try
  {
    if (!isRosInit) 
    {
      ros::init(argc, argv, "tms_rp_viewer");
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
  static ros::Rate loop_rate(30); // 0.1sec
  bool isTest=false;

  while (ros::ok())
  {
    if (isTest==false)
    {
      callSynchronously(bind(&grasp::TmsRpController::appear,trc_,"tv"));
      isTest=true;
    }
    else
    {
      callSynchronously(bind(&grasp::TmsRpController::disappear,trc_,"tv"));
      isTest=false;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

//------------------------------------------------------------------------------
