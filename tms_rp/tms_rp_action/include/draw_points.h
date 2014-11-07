#ifndef DRAW_POINTS_H
#define DRAW_POINTS_H

#include <cnoid/ScenePieces>
#include <cnoid/SceneGraph>
#include <math.h>
#include <QPoint>

#include <sensor_msgs/LaserScan.h>
#include <tms_msg_ss/tracking_points.h>

namespace grasp
{

class SgPointsDrawing;

class SgPointsDrawing : public cnoid::SgCustomGLNode
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SgPointsDrawing(tms_msg_rp::rps_map_full* map_data)
  {
    setRenderingFunction(boost::bind(&SgPointsDrawing::renderStaticMap, this));
    this->static_map_ = map_data;
  }

  SgPointsDrawing(tms_msg_rp::rps_route* map_data)
  {
    setRenderingFunction(boost::bind(&SgPointsDrawing::renderPathMap, this));
    this->path_map_ = map_data;
  }

  SgPointsDrawing(tms_msg_rp::rps_route* map_data, bool)
  {
    setRenderingFunction(boost::bind(&SgPointsDrawing::renderRobotMap, this));
    this->path_map_ = map_data;
  }

  SgPointsDrawing(sensor_msgs::LaserScan* raw_data1, sensor_msgs::LaserScan* raw_data2)
  {
    setRenderingFunction(boost::bind(&SgPointsDrawing::renderLaserRawData, this));
    this->laser_raw_data1_ = raw_data1;
    this->laser_raw_data2_ = raw_data2;
  }

  SgPointsDrawing(tms_msg_ss::tracking_points* person_data)
  {
    setRenderingFunction(boost::bind(&SgPointsDrawing::renderObject, this));
    this->person_data_ = person_data;
  }

  void renderStaticMap() {
    glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    float r=0,g=0,b=0;
    glPointSize(5.0);
    glColor3f(r,g,b);
    glBegin(GL_POINTS);
    r = 1.0;  g = 0.0;  b = 0.0;
    for(unsigned int x=0;x<static_map_->rps_map_x.size();x++){
      for(unsigned int y=0;y<static_map_->rps_map_x[x].rps_map_y.size();y++){
        if(static_map_->rps_map_x[x].rps_map_y[y].voronoi){
          glVertex3f((float)(x*0.1),(float)(y*0.1),0.01);
        }
      }
    }
    glEnd();
    glPopAttrib();
  }

  void renderPathMap() {
    glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    float r=0.0,g=0.0,b=0.0;
    glPointSize(10.0);
    glLineWidth(5);
    float current_point_x, current_point_y, next_point_x, next_point_y;

    for(unsigned int i=0;i<path_map_->rps_route.size()-1;i++){
      // set point
      current_point_x   = path_map_->rps_route[i].x/1000;
      current_point_y   = path_map_->rps_route[i].y/1000;
      next_point_x      = path_map_->rps_route[i+1].x/1000;
      next_point_y      = path_map_->rps_route[i+1].y/1000;

      // trajectory point
      r=0.0,g=1.0,b=0.0;
      glColor3f(r,g,b);
      glBegin(GL_POINTS);
      glVertex3f(current_point_x, current_point_y, 0.01);
      glEnd();

      // trajectory line
      r=0.0,g=0.0,b=1.0;
      glColor3f(r,g,b);
      glBegin(GL_LINES);
      glVertex3f(current_point_x, current_point_y, 0.01);
      glVertex3f(next_point_x, next_point_y, 0.01);
      glEnd();
    }
    glPopAttrib();
  }

  void renderRobotMap() {
    glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    float r=0.0,g=0.0,b=0.0;
    glPointSize(10.0);
    glLineWidth(5);
    float current_point_x, current_point_y, current_point_yaw, rotaion_x, rotaion_y;

    for(unsigned int i=0;i<path_map_->rps_route.size()-1;i++){
      // set point
      current_point_x   = path_map_->rps_route[i].x/1000;
      current_point_y   = path_map_->rps_route[i].y/1000;
      current_point_yaw = path_map_->rps_route[i].th;

      // robot mark
      unsigned int point_num = 24;
      QPointF robot_points[point_num];
      QPointF smartpal5_points[24] =
      {
        QPointF( 237 , 0 ),
        QPointF( 229 , -72 ),
        QPointF( 205 , -139),
        QPointF( 168 , -196),
        QPointF( 119 , -240),
        QPointF( 61  , -268),
        QPointF( 0 , -278),
        QPointF( -103, -268),
        QPointF( -192, -240),
        QPointF( -268, -196),
        QPointF( -327, -139),
        QPointF( -363, -72 ),
        QPointF( -376, 0 ),
        QPointF( -363, 72  ),
        QPointF( -327, 139 ),
        QPointF( -268, 196 ),
        QPointF( -192, 240 ),
        QPointF( -103, 268 ),
        QPointF( 0 , 278 ),
        QPointF( 61  , 268 ),
        QPointF( 119 , 240 ),
        QPointF( 168 , 196 ),
        QPointF( 205 , 139 ),
        QPointF( 229 , 72  )
      };

      for (int i=0; i<point_num; i++) {
        robot_points[i] = smartpal5_points[i]/1000;
      }

      for (int i=0; i<point_num; i++) {
        robot_points[i].setX(robot_points[i].x() + current_point_x);
        robot_points[i].setY(robot_points[i].y() + current_point_y);
      }

      for (int i=0; i<point_num; i++) {
        rotaion_x = (robot_points[i].x()-current_point_x)*cos(current_point_yaw) - (robot_points[i].y()-current_point_y)*sin(current_point_yaw);
        rotaion_y = (robot_points[i].x()-current_point_x)*sin(current_point_yaw) + (robot_points[i].y()-current_point_y)*cos(current_point_yaw);

        robot_points[i].setX(rotaion_x+current_point_x);
        robot_points[i].setY(rotaion_y+current_point_y);
      }

      // draw robot
      r=1.0,g=1.0,b=1.0;
      glColor3f(r,g,b);
      glLineWidth(1);
      glBegin(GL_LINE_LOOP);
      for (int i=0; i<point_num; i++) {
        glVertex3f(robot_points[i].x(), robot_points[i].y(), 0.01);
      }
      glEnd();
    }
    glPopAttrib();
  }

  void renderLaserRawData() {
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

    r=1,g=1,b=0;
    glColor3f(r,g,b);

    lrf_set_x = 5.2;
    lrf_set_y = 0.15;

    for(int i=0; i<laser_raw_data2_->ranges.size(); i++)
    {
      angle = (i * 0.25 - 45)* M_PI/180.;
      if(angle>=0 && angle<=M_PI)
      {
        distance = laser_raw_data2_->ranges[i];
        lrf_x = distance * cos(angle) + lrf_set_x;
        lrf_y = distance * sin(angle) + lrf_set_y;
        glVertex3d(lrf_x,lrf_y,1.05);
       }
    }

    glEnd();
    glPopAttrib();
  }

  void renderObject()
  {
    glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    float r=0,g=0,b=1;
    glPointSize(10.0);
    glColor3f(r,g,b);

    glBegin(GL_POINTS);
    for(unsigned int i=0;i<person_data_->tracking_grid.size();i++)
    {
      glVertex3f(person_data_->tracking_grid[i].x/1000,person_data_->tracking_grid[i].y/1000,1);
    }
    glEnd();

    float radius, height, slices, stacks, rings;
    radius = 0.1;
    height = 1.7;
    slices = 15; stacks = rings = 1;

    for(unsigned int i=0;i<person_data_->tracking_grid.size();i++)
    {
      GLUquadricObj *cylinder = gluNewQuadric();
      gluQuadricDrawStyle(cylinder, GLU_FILL);
      gluQuadricNormals(cylinder, GLU_SMOOTH);

      glPushMatrix();
      glTranslated(person_data_->tracking_grid[i].x/1000, person_data_->tracking_grid[i].y/1000, 0.0);

      gluCylinder(cylinder,radius,radius,height,slices,stacks);
      glPushMatrix();
      glTranslated(0.0, 0.0, height);
      r=0,g=0,b=0.5;
      glColor3f(r,g,b);
      gluDisk(cylinder, 0.0, radius, slices, 1);
      glPopMatrix();
      gluQuadricOrientation(cylinder, GLU_INSIDE);
      r=0,g=0,b=1;
      glColor3f(r,g,b);
      gluCylinder(cylinder, 0.0, 0.0, height, slices, stacks);
      gluDisk(cylinder,0.0,radius,slices,rings);
      gluDeleteQuadric(cylinder);

      glPopMatrix();
    }

    glPopAttrib();
  }

  virtual void accept(cnoid::SgVisitor& visitor){
    cnoid::SgCustomGLNode::accept(visitor);
    SgLastRenderer(this, true);
  }

  static SgPointsDrawing* SgLastRenderer(SgPointsDrawing* sg, bool is_write){
    static SgPointsDrawing* last;
    if(is_write) last=sg;
    return last;
  }

  tms_msg_rp::rps_map_full* static_map_;
  tms_msg_rp::rps_route*    path_map_;
  sensor_msgs::LaserScan*   laser_raw_data1_;
  sensor_msgs::LaserScan*   laser_raw_data2_;
  tms_msg_ss::tracking_points* person_data_;
};

typedef boost::intrusive_ptr<SgPointsDrawing> SgPointsDrawingPtr;
}

#endif // DRAW_POINTS_H
