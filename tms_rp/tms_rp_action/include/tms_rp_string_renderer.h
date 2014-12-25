#ifndef _SgCharcterRenderer_h__
#define _SgCharcterRenderer_h__

#include <QPainter>
#include <GL/freeglut.h>

#include <cnoid/SceneView>
#include <cnoid/SceneGraph>
#include <cnoid/GLSceneRenderer>
#include <QGLWidget>

#include <string>
#include <boost/bind.hpp>


//#include "../GeometryHandler/GeometryHandle.h"


namespace grasp{

	class SgStringRenderer : public cnoid::SgCustomGLNode
	{
		public:
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

			SgStringRenderer(std::string str, cnoid::Vector3 pos){
				setRenderingFunction(boost::bind(&SgStringRenderer::render, this));
				displayString = str;
				position = pos;

				static bool first=true;
				if(first==true){
					first=false;
					int argc=1;
					char* argv[] = {"chorenoid" };
					glutInit(&argc, argv);
				}
			}

			void render(){
				glPushAttrib(GL_LIGHTING_BIT | GL_CURRENT_BIT);
				glDisable(GL_LIGHTING);
				glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
				
				glRasterPos3f(position[0], position[1], position[2]);
				for(int i=0;i<displayString.size();i++){
					glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, displayString.c_str()[i]);
				}

				glPopAttrib();
			}
			void setString(std::string str){
				displayString = str;
			}
			void setPosition(cnoid::Vector3 pos){
				position = pos;
			}
			std::string displayString;
			cnoid::Vector3 position;
			
	};
	typedef boost::intrusive_ptr<SgStringRenderer> SgStringRendererPtr;
}

#endif