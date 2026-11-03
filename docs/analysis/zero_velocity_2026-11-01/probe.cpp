#include <iostream>
#include <memory>
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
class Probe: public nav2_costmap_2d::InflationLayer {
public:
 void prepare(nav2_costmap_2d::LayeredCostmap * p) {
  layered_costmap_=p; enabled_=true; current_=true;
  inflation_radius_=0.5; inscribed_radius_=0.45; cost_scaling_factor_=5.0;
  inflate_unknown_=false; inflate_around_unknown_=false;
  matchSize();
 }
};
int main() {
 nav2_costmap_2d::LayeredCostmap map("map",true,false);
 map.resizeMap(140,140,0.05,-3.5,-3.5);
 auto p=std::make_shared<Probe>(); p->prepare(&map); map.addPlugin(p);
 map.updateMap(0,0,0);
 std::cout<<"before reset: layer="<<p->isCurrent()<<" map="<<map.isCurrent()<<std::endl;
 p->reset();
 for(int i=0;i<20;i++) map.updateMap(0,0,0);
 std::cout<<"after reset and 20 updates: layer="<<p->isCurrent()<<" map="<<map.isCurrent()<<std::endl;
 p->updateCosts(*map.getCostmap(),0,0,140,140);
 std::cout<<"after explicit nonempty cost update: layer="<<p->isCurrent()<<" map="<<map.isCurrent()<<std::endl;
}
