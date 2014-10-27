#ifndef _SGPOINTSGET_H_INCLUDED
#define _SGPOINTSGET_H_INCLUDED

#include <cnoid/ScenePieces>
#include <cnoid/SceneGraph>
#include <math.h>

#define PI 3.141592

namespace grasp{

class SgPointsRenderer;

class SgPointsGet : public cnoid::SgVisitor {
	public:
    std::vector<cnoid::SgShape*> shape;
    virtual void visitShape(cnoid::SgShape* shape){
		  this->shape.push_back(shape);
		}
};

class SgPointsRenderer : public cnoid::SgCustomGLNode {
  public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		SgPointsRenderer(pcl::PointCloud<pcl::PointXYZ>* pcd) {
			setRenderingFunction(boost::bind(&SgPointsRenderer::renderPoints, this));
			this->pcd = pcd;
		}

		void renderPoints() {
			glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
			glDisable(GL_LIGHTING);
			glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

			glBegin(GL_POINTS);

			float d=0,r=0,g=0,b=0;

			for(int i=0; i<pcd->points.size(); i++) {
			  d = sqrt((pcd->points[i].x)*(pcd->points[i].x) + (pcd->points[i].y)*(pcd->points[i].y));
              r = sin((d/10)*(PI/2)); //距離10mで最大、r^2+g^2=1
			  g = sqrt(1-r*r);
              b = 1;
			  glColor3f(r,g,b);
              //glVertex3d(pcd->points[i].x,pcd->points[i].y,pcd->points[i].z);
              glVertex3d(pcd->points[i].x,pcd->points[i].y,1);
			}

			glEnd();
			glPopAttrib();
		}
		virtual void accept(cnoid::SgVisitor& visitor){
			cnoid::SgCustomGLNode::accept(visitor);
			SgLastRenderer(this, true);
		}
		static SgPointsRenderer* SgLastRenderer(SgPointsRenderer* sg, bool isWrite){
			static SgPointsRenderer* last;
			if(isWrite) last=sg;
			return last;
		}
		pcl::PointCloud<pcl::PointXYZ>* pcd;
};

typedef boost::intrusive_ptr<SgPointsRenderer> SgPointsRendererPtr;
}

#endif
