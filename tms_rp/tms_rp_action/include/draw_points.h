#ifndef DRAW_POINTS_H
#define DRAW_POINTS_H

#include <cnoid/ScenePieces>
#include <cnoid/SceneGraph>
#include <math.h>

#define PI 3.141592

namespace grasp{

class SgPointsDrawing;

class SgPointsDrawing : public cnoid::SgCustomGLNode {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SgPointsDrawing(tms_msg_rp::rps_map_full* mapData) {
    setRenderingFunction(boost::bind(&SgPointsDrawing::renderPoints, this));
    this->mapData = mapData;
  }

  void renderPoints() {
    glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
    glDisable(GL_LIGHTING);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    float r=0,g=0,b=0;

    glPointSize(5.0);
    glColor3f(r,g,b);

    glBegin(GL_POINTS);

    r = 1.0;
    g = 0.0;
    b = 0.0;

    for(unsigned int x=0;x<mapData->rps_map_x.size();x++){
      for(unsigned int y=0;y<mapData->rps_map_x[x].rps_map_y.size();y++){
        if(mapData->rps_map_x[x].rps_map_y[y].voronoi){
          glVertex3d(x*0.1,y*0.1,0.1);
        }
      }
    }

    glEnd();

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

    tms_msg_rp::rps_map_full* mapData;
};

typedef boost::intrusive_ptr<SgPointsDrawing> SgPointsDrawingPtr;
}

#endif // DRAW_POINTS_H
