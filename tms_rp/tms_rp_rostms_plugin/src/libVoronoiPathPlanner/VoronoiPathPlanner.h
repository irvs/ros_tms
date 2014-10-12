#ifndef _VORONOI_PATH_PLANNER_H
#define _VORONOI_PATH_PLANNER_H

#include <QFileDialog>
#include <QTextStream>
#include <Grasp/PlanBase.h>
#include <Grasp/VectorMath.h>
#include <boost/make_shared.hpp>

#define rad2deg(x)	((x)*(180.0)/M_PI)
#define deg2rad(x)	((x)*M_PI/180.0)

using namespace boost;
using namespace cnoid;
using namespace std;

namespace grasp{
	
	class EXCADE_API CollisionMapData
	{
		public:
			bool object;
			double dist_from_obj;
			
			bool collision;
			
			bool voronoi;
			int thinning_flg;
			double dist_from_voronoi;
			
			bool path;
			double dist_from_path;
			
			double dist_from_goal;
	};
	
	class EXCADE_API CollisionTarget
	{
		public:
			CollisionTarget(BodyItemPtr bodyItem);
			const string& name() { return bodyItemCollisionTarget->name(); }
			
			cnoid::BodyItemPtr bodyItemCollisionTarget;
			cnoid::Link *base;
	};

	class EXCADE_API VoronoiPathPlanner
	{
		public:
			VoronoiPathPlanner();
			~VoronoiPathPlanner();
			
			static VoronoiPathPlanner* instance(VoronoiPathPlanner *gc=NULL);
			
			ostream& os;
			
			double x_llimit, x_ulimit, y_llimit, y_ulimit, cell_size;
			vector<double> start_pos, goal_pos;
			
			CollisionTarget* collisionTarget;
			cnoid::Link* collisionTargetBase() { return collisionTarget->base; }
			vector<ColdetLinkPairPtr> collisionMapPairs;
			
			vector<vector<CollisionMapData> > collisionMap;
			
			void initialCollision();
			bool isColliding();
			void SetCollisionTarget(BodyItemPtr bodyItem);
			
			void MapOutput(vector<vector<CollisionMapData> > Map);
			
			bool calcDistFromObj(vector<vector<CollisionMapData> >& Map);
			bool calcDistFromVoronoi(vector<vector<CollisionMapData> >& Map);
			bool calcDistFromGoal(vector<vector<CollisionMapData> >& Map, vector<double> goal_point);
			bool calcDistFromPath(vector<vector<CollisionMapData> >& Map);
			bool setCollisionArea(vector<vector<CollisionMapData> >& Map, double threshold);
			bool setVoronoiLine(vector<vector<CollisionMapData> >& Map);
			bool connectToVoronoi(vector<vector<CollisionMapData> >& Map, vector<double> connect_point);
			bool calcVoronoiPath(vector<vector<CollisionMapData> >& Map, vector<double> start, vector<double> goal, vector<vector<double> >& out_path);
			bool smoothVoronoiPath(vector<vector<CollisionMapData> >& Map, vector<double> start, vector<double> goal, vector<vector<double> > in_path, vector<vector<double> >& out_path, double threshold);
			bool compVoronoiPath(vector<vector<double> > in_path, vector<vector<double> >& out_path);
			bool planVoronoiPath(vector<vector<CollisionMapData> >& Map, vector<double> start, vector<double> goal, vector<vector<double> >& out_path);
			
			bool initCollisionMap(vector<vector<CollisionMapData> >& Map);
			//~ void setCollisionMap(vector<vector<CollisionMapData> >& Map);
			bool loadCollisionMap(QString select_file_name);
			bool makeCollisionMap(vector<vector<int> >& out_collision_map);
	};
	
};
#endif
