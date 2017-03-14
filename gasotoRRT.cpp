#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <boost/algorithm/string.hpp>
#include <iostream>
#include "gasotoRRT.h"

#define REACHED 0
#define ADVANCED 1
#define TRAPPED -1

static const long MAX_TIME = 180*CLOCKS_PER_SEC*2.5; //3 minutes
using namespace OpenRAVE;


bool rrt::SmoothPath(std::ostream& sout, std::istream& sinput)
{
  std::string fullinput;
  //Parse input
  while (sinput){
    std::string input;
    sinput >> input ;
    fullinput +=input;
  }
  int nSmooth = atoi(fullinput.c_str());

  while (nSmooth){
    ShortcutSmoothing();
    std::cout << " path length: "<< path.size()<<std::endl;
    nSmooth--;
  }
  robot->SetActiveDOFValues(start);
  env->UpdatePublishedBodies();
  //std::cout<< (std::clock() - startTime)/CLOCKS_PER_SEC << std::endl;
  //std::cout <<"Length: "<<path.size();
  for (uint i = 0; i< path.size(); i++ ){
    std::vector<double> node = path[i];
    //sout << "[";
    for(uint j=0; j<node.size()-1; j++){
      sout << node[j]<<",";
    }
    sout << node[node.size()-1] <<"\n";
  }

  return true;
}

bool rrt::ShortcutSmoothing(){
  if (path.size() <=2) return false;
  uint i =((double)rand()/RAND_MAX)*path.size();
  uint j = ((double)rand()/RAND_MAX)*path.size();
  while(abs(j - i)<2){
    i = ((double)rand()/RAND_MAX)*path.size();
    j = ((double)rand()/RAND_MAX)*path.size();
  }
  if( j < i){ //make sure i is the smaller number
    int a = j;
    j = i;
    i = a;
  }

  if (CheckTraj(path[i],path[j])){
    //std::cout << "can Shortcut "<< i << " to "<< j <<std::endl;
    //debug();
    path.erase(path.begin() + i + 1,path.begin() + j);
    return true;
  }
  return false;
}



bool rrt::CheckTraj(std::vector<double> &a, std::vector<double> &b){
  std::vector<double> u = getUnitVector(a,b);
  std::vector<double> p = a;
  //std::cout << &p << "  " << &a << std::endl;
  while (p != b){
    if (checkCollision(p)){return false;}
    if(getDistance(p,b)< step_size/2){
      p = b;
    }else{
      for (uint i =0; i< u.size(); i++){
        p[i]+=u[i]*step_size/2;
      }
    }
  }
  return true;
}
double rrt::getDistance(std::vector<double> &a,std::vector<double> &b){

  //Gets squared Euclidean Distance on C-Space between node and a given
  double accumulated = 0.0f;
  std::vector<double> diff = b;

  for (uint i = 0; i< diff.size(); i++){
    diff[i]-=a[i];
    //double dif = _configuration[i] - (config)[i];
    accumulated += (diff[i]*diff[i]*DOF_weights[i]*DOF_weights[i]);
  }
  return sqrt(accumulated);
}
std::vector<double> rrt::getUnitVector(std::vector<double> &a,std::vector<double> &b){

  //Gets squared Euclidean Distance on C-Space between node and a given
  double accumulated = 0.0f;
  std::vector<double> diff = b;

  for (uint i = 0; i< diff.size(); i++){
    diff[i]-=a[i];//double dif = _configuration[i] - (config)[i];
    accumulated += (diff[i]*diff[i]*DOF_weights[i]*DOF_weights[i]);
  }
  accumulated = sqrt(accumulated);
  for (uint i = 0; i < diff.size(); i++){
    diff[i]/=accumulated;
    //std::cout << diff[i];
  }
  //std::cout << std::endl;
  return diff;
}

bool rrt::StartRRT(std::ostream& sout, std::istream& sinput){
  std::string fullinput;
  std::vector<std::string> startconfig;
  std::vector<std::string> endgoal;
  std::vector<std::string> weightsstr;
  std::vector<std::string> parameters;
  std::vector<std::string> inputs;
  tree._nodes.clear();
  gTree._nodes.clear();
  goal.clear();
  start.clear();
  DOF_weights.clear();
  LowerLimit.clear();
  UpperLimit.clear();

  std::vector<int> ActiveDOFs = robot->GetActiveDOFIndices() ;
  //robot->GetActiveDOFValues(start) ;

  //Parse input
  while (sinput){
    std::string input;
    sinput >> input ;
    fullinput +=input;
  }
  boost::erase_all(fullinput,"[");

  boost::split(inputs,fullinput,boost::is_any_of("]"));
  boost::split(startconfig,inputs[0],boost::is_any_of(","));
  boost::split(endgoal,inputs[1],boost::is_any_of(","));
  boost::split(weightsstr,inputs[2],boost::is_any_of(","));

  if(inputs.size() > 3){
    //std::cout <<inputs[3] <<std::endl;
    boost::split(parameters,inputs[3],boost::is_any_of(","));

    step_size = atof(parameters[0].c_str());
    goal_bias = atof(parameters[1].c_str());
    displaySearch = atoi(parameters[2].c_str());
    bidirectional = atoi(parameters[3].c_str());
  }
  for (uint i = 0; i< startconfig.size(); i++){
    DOF_weights.push_back(atof(weightsstr[i].c_str()));
    start.push_back(atof(startconfig[i].c_str()));
    goal.push_back(atof(endgoal[i].c_str()));
    //sout << goal[i]<< "\t";
  }
  robot->GetActiveDOFLimits(LowerLimit,UpperLimit) ;
  for (uint i =0 ;i< LowerLimit.size(); i++){

        if (start[i] < LowerLimit[i]){
          start[i] = LowerLimit[i];
        }
        if (goal[i] < LowerLimit[i]){
          goal[i] = LowerLimit[i];
        }
        if (start[i] > UpperLimit[i]){
          start[i] = UpperLimit[i];
        }
        if (goal[i] > UpperLimit[i]){
          goal[i] = UpperLimit[i];
        }
        if (LowerLimit[i] == -10000){ //Rotational Joint with no constraints;
          LowerLimit[i] = -M_PI;
          UpperLimit[i] = M_PI;
        }
  	}
  //checkCollision(goal);


  RRTNode n(start,0);
  tree.addNode(n);
  c_tree = &tree;
  t_turn = 0;
  g2 = goal;
  RRTNode g(goal,0);
  gTree.addNode(g);

  std::clock_t startTime = std::clock();
  bool finished = false;
  while(double(std::clock() - startTime) < MAX_TIME){
    count = 0;
    if(bidirectional){
      if (t_turn == 1){
        t_turn = 2;
        g2 = tree._nodes.back()->getConfiguration();
        c_tree = &gTree;
      }else{
        t_turn = 1;
        g2 = gTree._nodes.back()->getConfiguration();
        c_tree = &tree;
      }
    }
    // std::cout << c_tree  << std::endl;
    // for(uint i = 0; i< g2.size(); i++){
    //   std::cout << g2[i]<<" ";
    // }
    //std::cout << std::endl;
    std::vector<double> node = RandomConfig();
    if(this->connect(node) == REACHED && isGoal){
      //std::cout << "FINISHED " << c_tree << std::endl;
      finished = true;
      break;
    }


  }
    //debug();

  robot->SetActiveDOFValues(start);
  env->UpdatePublishedBodies();
  //std::cout<< (std::clock() - startTime)/CLOCKS_PER_SEC << std::endl;
  if (finished){
    // std::cout << "FINISHED" << std::endl;
    // for (uint i = 0; i< goal.size(); i++){
    //   std::cout << tree._nodes.back()->getConfiguration()[i] << "  "<<tree._nodes.back()->getConfiguration()[i]<<std::endl;
    // }
    // debug();
    path = tree.getPath();
    if(bidirectional){
      std::vector<std::vector<double> > p2;
      p2 = gTree.getPath();
      std::reverse(p2.begin(),p2.end());
      path.insert(path.end(),p2.begin(),p2.end());
      //std::reverse(p2.begin(),p2.end());
    }

    // if(!bidirectional){
    //   std::cout <<"RRT Size: "<<tree._nodes.size();
    // }
    // else{
    //   std::cout <<"RRT Size: "<<tree._nodes.size() + gTree._nodes.size();
    // }
    for (uint i = 0; i< path.size(); i++ ){
      std::vector<double> node = path[i];
      //sout << "[";
      for(uint j=0; j<node.size()-1; j++){
        sout << node[j]<<",";
        //std::cout << node[j]<<",";
      }
      sout << node[node.size()-1] <<"\n";
      //std::cout << node[node.size()-1] <<"\n";
    }
  }
  //std::cout<<std::endl;


  return true;
}
int rrt::connect(std::vector<double> &node){

  RRTNode* near = c_tree->getNearest(robot,node,DOF_weights);
  int s = this->extend(node,near);
  while (s != TRAPPED){
    if (s == REACHED){
      return REACHED;
    }
    s = this->extend(node,near);
    //std::cout << "test";

  }
  return TRAPPED;


}
int rrt::extend(std::vector<double> &node, RRTNode* &near){
  double distance = sqrt(near->getDistance(robot,node,DOF_weights));
  std::vector<double> qnew;
  //std::cout << distance <<std::endl;
  if (distance < step_size){
    if(!checkCollision(node)){
      RRTNode *old = near;
      near = (new RRTNode(node,old));
      c_tree->addNode(*near);
      //std::cout <<"REACHED " << c_tree<< "  " << near->_parent <<"  "<<near << std::endl;
      return REACHED;
    }else{
      count++;
      //std::cout << "Trapped" << std::endl;
      //debug();
      return TRAPPED;
    }
  }
  for(uint i =0; i< goal.size(); i++){
    qnew.push_back(near->getConfiguration()[i] + ((node[i] - near->getConfiguration()[i])/distance)*step_size);
  }
  bool check = checkCollision(qnew);
  if(!check){
    RRTNode *old = near;

    near = (new RRTNode(qnew,old));
    c_tree->addNode(*near);
    //debug();
    //std::cout <<" ADVANCED" << c_tree<< "  " << near->_parent <<"  "<<near << std::endl;
    //debug();
    //std::cout <<"test";
    return ADVANCED;
  }else{
    //std::cout << "Trapped3" << std::endl;
    return TRAPPED;
  }

}
std::vector<double> rrt::RandomConfig(){
  double goalb = (double)rand()/RAND_MAX;
  //std::cout << "GoalBias "<< goalb << std::endl;
  //debug();
  if (goalb < goal_bias){
    isGoal = true;

    //std::cout << "Goal Selected" << std::endl;
    return g2;
  }
  else{
    isGoal = false;
    std::vector<double> R;
    do{
      for (uint i =0; i< start.size(); i++){
        double jointrange = UpperLimit[i] - LowerLimit[i];
        double r = ((double)rand()/(double)RAND_MAX)*jointrange;
        //std::cout <<LowerLimit[i] + r<< "range "<< jointrange <<  " random: " << r << std::endl;

        R.push_back( LowerLimit[i] + r);
      }
      //std::cout << "Random Selected" << std::endl;

      //debug();
      if (checkCollision(R)){
        R.clear();
      }
    }while (R.size() != goal.size());
    return R;
  }
}

bool rrt::checkCollision(std::vector<double> &config){
  // for (uint i =0; i< config.size(); i++){
  //   std::cout <<config[i]<< ";";
  // }
  // std::cout << std::endl;

  robot->SetActiveDOFValues(config);
  if(displaySearch){
    env->UpdatePublishedBodies();
  }
  bool check = robot->CheckSelfCollision() || env->CheckCollision(robot);
  //std::cout << check << std::endl;
  return (check);
}

void rrt::init(){
  std::vector<RobotBasePtr> Rptr ;

  env = GetEnv() ;
  env->GetRobots(Rptr) ;
  env->GetCollisionChecker()->SetCollisionOptions(CO_RayAnyHit);
  robot = Rptr[0] ;
  //robot->GetActiveDOFWeights(Weights) ;

}


// called to create a new plugin
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "rrt" ) {
        return InterfaceBasePtr(new rrt(penv,sinput));
    }

    return InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(PLUGININFO& info)
{
info.interfacenames[PT_Module].push_back("rrt");

}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin()
{
}
