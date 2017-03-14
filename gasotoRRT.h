#include "NodeTree.h"
using namespace OpenRAVE;

class rrt : public ModuleBase
{
public:
    void init();

    //Messages
    bool StartRRT(std::ostream& sout, std::istream& sinput);
    bool SmoothPath(std::ostream& sout, std::istream& sinput);
    //common
    bool checkCollision(std::vector<double> &config);
    //RRT
    int connect(std::vector<double> &node);
    int extend(std::vector<double> &node, RRTNode* &near);
    std::vector<double> RandomConfig();

    //ShortcutSmoothing
    bool ShortcutSmoothing();
    bool CheckTraj(std::vector<double> &a, std::vector<double> &b);
    double getDistance(std::vector<double> &a,std::vector<double> &b);
    std::vector<double> getUnitVector(std::vector<double> &a,std::vector<double> &b);
    //Constructor
    rrt(EnvironmentBasePtr penv, std::istream& ss) : ModuleBase(penv) {
        RegisterCommand("StartRRT",boost::bind(&rrt::StartRRT,this,_1,_2),
                        "Start the RRT Class");
        RegisterCommand("SmoothPath",boost::bind(&rrt::SmoothPath,this,_1,_2),
                        "Smoothes RRT path found by RRT planner");
        init();
    }
    virtual ~rrt() {}
    double goal_bias;
    double goal_thresh;
    double step_size;
    bool isGoal;
    bool displaySearch;
    bool bidirectional;
    int t_turn;
    NodeTree *c_tree;
    NodeTree tree;
    NodeTree gTree;
    //RRTNode node;
    std::vector<double> start;
    std::vector<double> goal;
    std::vector<double> g2;
    EnvironmentBasePtr env;
    RobotBasePtr robot;
    int count;
    //std::vector<double> Weights;
    std::vector<double> LowerLimit;
    std::vector<double> UpperLimit;
    std::vector<double> DOF_weights;
    std::vector<std::vector<double> > path;
    TrajectoryBasePtr Traj;
  };
