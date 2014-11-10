#ifndef DRAW_POINTS2_H
#define DRAW_POINTS2_H

#include <cnoid/SceneView>
#include <sensor_msgs/LaserScan.h>

namespace grasp
{

class SgPointsDrawing2;

class SgPointsDrawing2 : public cnoid::SgCustomGLNode
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SgPointsDrawing2(sensor_msgs::LaserScan* raw_data)
  {
    setRenderingFunction2(boost::bind(&SgPointsDrawing2::renderLaserRawData2, this));
    this->laser_raw_data_ = raw_data;
  }

  void renderLaserRawData2() {
    glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    float r=0,g=0,b=0;
    glPointSize(3.0);
    glBegin(GL_POINTS);

    float angle, distance, lrf_x, lrf_y;
    float lrf_set_x;
    float lrf_set_y;

    r=1,g=0,b=0;
    glColor3f(r,g,b);

    lrf_set_x = 2.5;
    lrf_set_y = 0.15;

    for(int i=0; i<laser_raw_data1_->ranges.size(); i++)
    {
      angle = (i * 0.25 - 45)* M_PI/180.;
      if(angle>=0 && angle<=M_PI)
      {
        distance = laser_raw_data1_->ranges[i];
        lrf_x = distance * cos(angle) + lrf_set_x;
        lrf_y = distance * sin(angle) + lrf_set_y;
        glVertex3d(lrf_x,lrf_y,0.875);
       }
    }

    SgVertexArrayPtr vertices = new SgVertexArray();
    //vertices->reserve(numPoints); a lot of PCD
    SgVector3 vertex = SgVector3(1,1,1);
    vertices->push_back(vertex);
    SgPointSetPtr pointSet = new SgPointSet();
    pointSet->setVertices(vertices);

    if (pointSet)
    {
      SgInvariantGroupPtr group = new SgInvariantGroup();
      group->addChild(pointSet);
      SceneView::instance()->addEntity(group);
      //os << pointSet->vertices()->size() << " points have been added.";
    }

    glEnd();
    glPopAttrib();
  }

  void removeLaserRawData2()
  {
    SceneView::instance()->removeEntity(group);
  }
  sensor_msgs::LaserScan*   laser_raw_data_;
};

}
#endif // DRAW_POINTS2_H
