using namespace OpenRAVE;
class RRTNode {

public:
  std::vector<double> _configuration; //Vector for the joint values in the configuration
  RRTNode *_parent;//Pointer to Parent node on RRT Tree

  RRTNode() {}
  RRTNode(std::vector<double> &configuration): _configuration(configuration){}
  RRTNode(std::vector<double> &configuration, RRTNode *parent): _configuration(configuration), _parent(parent){}
  std::vector<double> getConfiguration();
  void setConfiguration(std::vector<double> &configuration);
  double getDistance(RobotBasePtr &robot, std::vector<double> &config,std::vector<double> &w);
  RRTNode* getParent() {return _parent;}
};



std::vector<double> RRTNode::getConfiguration(){
  return _configuration;
}
void RRTNode::setConfiguration(std::vector<double> &configuration){
  _configuration = configuration;
}
double RRTNode::getDistance(RobotBasePtr &robot, std::vector<double> &config,std::vector<double> &w){

  //Gets squared Euclidean Distance on C-Space between node and a given
  double accumulated = 0.0f;
  std::vector<double> diff = _configuration;
  robot->SubtractActiveDOFValues(diff,config);
  for (uint i = 0; i< _configuration.size(); i++){
    //double dif = _configuration[i] - (config)[i];
    accumulated += (diff[i]*diff[i]*w[i]*w[i]);
  }
  return accumulated;
}
