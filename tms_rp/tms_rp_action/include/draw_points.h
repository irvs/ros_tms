#ifndef DRAW_POINTS_H
#define DRAW_POINTS_H

#include <cnoid/ScenePieces>
#include <cnoid/SceneGraph>
#include <math.h>
#include <QPoint>

#define PI 3.141592

namespace grasp{

class SgPointsDrawing;

class SgPointsDrawing : public cnoid::SgCustomGLNode {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SgPointsDrawing(tms_msg_rp::rps_map_full* mapData) {
    setRenderingFunction(boost::bind(&SgPointsDrawing::RenderStaticMap, this));
    this->static_map_ = mapData;
  }

  SgPointsDrawing(tms_msg_rp::rps_route* mapData) {
    setRenderingFunction(boost::bind(&SgPointsDrawing::RenderPathMap, this));
    this->path_map_ = mapData;
  }

  SgPointsDrawing(tms_msg_rp::rps_route* mapData, bool) {
    setRenderingFunction(boost::bind(&SgPointsDrawing::RenderRobotMap, this));
    this->path_map_ = mapData;
  }

  void RenderStaticMap() {
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

  void RenderPathMap() {
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

  void RenderRobotMap() {
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

  virtual void accept(cnoid::SgVisitor& visitor){
    cnoid::SgCustomGLNode::accept(visitor);
    SgLastRenderer(this, true);
  }

  static SgPointsDrawing* SgLastRenderer(SgPointsDrawing* sg, bool isWrite){
    static SgPointsDrawing* last;
    if(isWrite) last=sg;
    return last;
  }

  tms_msg_rp::rps_map_full* static_map_;
  tms_msg_rp::rps_route*    path_map_;
};

typedef boost::intrusive_ptr<SgPointsDrawing> SgPointsDrawingPtr;
}

#endif // DRAW_POINTS_H
