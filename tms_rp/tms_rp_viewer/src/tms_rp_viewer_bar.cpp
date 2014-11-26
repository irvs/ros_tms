#include <tms_rp_viewer_bar.h>

//------------------------------------------------------------------------------
using namespace std;
using namespace boost;
using namespace cnoid;
using namespace grasp;
using namespace tms_rp;

bool RpViewerBar::isRosInit = false;

//------------------------------------------------------------------------------
RpViewerBar* RpViewerBar::instance(){
  static RpViewerBar* instance = new RpViewerBar();
  return instance;
}

//------------------------------------------------------------------------------
RpViewerBar::RpViewerBar(): ToolBar("RpViewerBar"),
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

  addLabel(("[RpViewerPlugin]"));

  addButton(("Test"), ("Test Button"))->
    sigClicked().connect(bind(&RpViewerBar::onTestButtonClicked, this));
}

//------------------------------------------------------------------------------
RpViewerBar::~RpViewerBar()
{
}

//------------------------------------------------------------------------------
void RpViewerBar::onTestButtonClicked()
{
  static boost::thread t(boost::bind(&RpViewerBar::rosOn, this));
}

//------------------------------------------------------------------------------
void RpViewerBar::rosOn()
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
