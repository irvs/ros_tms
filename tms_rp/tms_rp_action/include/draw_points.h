#ifndef DRAW_POINTS_H
#define DRAW_POINTS_H

#include <cnoid/ScenePieces>
#include <cnoid/SceneGraph>
#include <math.h>

#define PI 3.141592

namespace grasp{

class SgDrawPoints;

class SgGetPoints : public cnoid::SgVisitor {
    public:
    std::vector<cnoid::SgShape*> shape;
    virtual void visitShape(cnoid::SgShape* shape){
          this->shape.push_back(shape);
        }
};

class SgDrawPoints : public cnoid::SgCustomGLNode {
  public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SgDrawPoints() {
            setRenderingFunction(boost::bind(&SgDrawPoints::renderPoints, this));
//            this->pcd = pcd;
        }

        void renderPoints() {
            glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
            glDisable(GL_LIGHTING);
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

            glBegin(GL_POINTS);

            float d=0,r=0,g=0,b=0;

              r = 1;
              g = 1;
              b = 1;
              glColor3f(r,g,b);
              //glVertex3d(pcd->points[i].x,pcd->points[i].y,pcd->points[i].z);
              glVertex3d(1,1,1);

            glEnd();
            glPopAttrib();
        }
        virtual void accept(cnoid::SgVisitor& visitor){
            cnoid::SgCustomGLNode::accept(visitor);
            SgLastRenderer(this, true);
        }
        static SgDrawPoints* SgLastRenderer(SgDrawPoints* sg, bool isWrite){
            static SgDrawPoints* last;
            if(isWrite) last=sg;
            return last;
        }

};

typedef boost::intrusive_ptr<SgDrawPoints> SgDrawPointsPtr;
}

#endif // DRAW_POINTS_H
