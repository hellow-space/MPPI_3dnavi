// Analysis only: calls existing query/evaluator; no production algorithm changes.
#include "height_jump_mppi_critic/terrain_map.hpp"
#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <random>
#include <set>
#include <sstream>
#include <vector>
using namespace height_jump;
volatile double observable=0;
template<class Work> void measure(const char *name,Work work) {
 std::vector<double> times;
 for(int r=0;r<35;++r){auto a=std::chrono::steady_clock::now();double sum=work();auto b=std::chrono::steady_clock::now();observable+=sum;if(r>=5)times.push_back(std::chrono::duration<double,std::milli>(b-a).count());}
 std::sort(times.begin(),times.end());
 std::cout<<"TIMING "<<name<<" p50_ms="<<times[15]<<" p95_ms="<<times[28]<<" max_ms="<<times.back()<<std::endl;
}
using Key=std::array<int,9>;
Key key(const grid_map::GridMap &m,double x,double y,double radius) {
 Key k{};int n=0;
 for(int a=-1;a<=1;++a)for(int b=-1;b<=1;++b){
  grid_map::Position p(x+a*radius,y+b*radius);grid_map::Index i;
  bool good=m.isInside(p)&&m.getIndex(p,i)&&(i>=grid_map::Index::Zero()).all()&&(i<m.getSize()).all();
  k[n++]=good?i.x()+m.getSize().x()*i.y():-1;
 }
 return k;
}
int index(const grid_map::GridMap &m,double x,double y) {
 grid_map::Position p(x,y);grid_map::Index i;
 if(!m.isInside(p)||!m.getIndex(p,i)||(i<grid_map::Index::Zero()).any()||(i>=m.getSize()).any())return -1;
 return i.x()+m.getSize().x()*i.y();
}
void reuse(const char *filename) {
 std::ifstream in(filename);if(!in)throw std::runtime_error("cannot open rollout CSV");std::string line;std::getline(in,line);
 struct Counts{int n=0,inside=0;std::set<int>cells;std::set<Key>neighborhoods;std::map<int,std::set<Key>>variants;};
 std::map<int,Counts> counts;
 while(std::getline(in,line)){
  std::replace(line.begin(),line.end(),',',' ');std::istringstream s(line);
  int f,sx,sy;double t,cx,cy,res,lx,ly,x,y;s>>f>>t>>cx>>cy>>res>>lx>>ly>>sx>>sy>>x>>y;if(!s)throw std::runtime_error("bad CSV");
  grid_map::GridMap m;m.setGeometry(grid_map::Length(lx,ly),res,grid_map::Position(cx,cy));m.setStartIndex(grid_map::Index(sx,sy));
  auto &c=counts[f];++c.n;int i=index(m,x,y);if(i<0)continue;++c.inside;c.cells.insert(i);auto k=key(m,x,y,.1);c.neighborhoods.insert(k);c.variants[i].insert(k);
 }
 for(auto &[f,c]:counts){size_t multi=0;for(const auto &v:c.variants)multi+=v.second.size()>1;
  std::cout<<"REUSE frame="<<f<<" points="<<c.n<<" inside="<<c.inside<<" unique_center_cells="<<c.cells.size()<<" unique_neighborhoods="<<c.neighborhoods.size()<<" center_cells_with_multiple_neighborhoods="<<multi<<std::endl;
 }
}
void boundaries() {
 for(double res:{.1,.13}){
  grid_map::GridMap m;m.setGeometry(grid_map::Length(7,5),res,grid_map::Position(.4,-.3));
  std::map<int,std::set<Key>> variants;
  auto center=m.getPosition();auto length=m.getLength();auto size=m.getSize();
  for(int cell=1;cell<size.x()-1;++cell){
   double edge=center.x()+length.x()/2-cell*res;
   std::vector<double>xs{std::nextafter(edge,-INFINITY),edge,std::nextafter(edge,INFINITY)};
   for(double fraction:{.01,.1,.25,.5,.75,.9,.99})xs.push_back(edge-fraction*res);
   for(double x:xs){int id=index(m,x,center.y());if(id>=0)variants[id].insert(key(m,x,center.y(),.1));}
  }
  size_t multi=0,max=0;for(const auto &[id,v]:variants){multi+=v.size()>1;max=std::max(max,v.size());}
  std::cout<<"BOUNDARY resolution="<<res<<" cells="<<variants.size()<<" center_cells_with_multiple_neighborhoods="<<multi<<" max_variants_per_cell="<<max<<std::endl;
 }
}
int main(int argc,char **argv) {
 try{
  auto snapshot=std::make_shared<TerrainSnapshot>();auto &m=snapshot->map;
  m=grid_map::GridMap({"elevation"});m.setGeometry(grid_map::Length(7,7),.1);
  snapshot->layer="elevation";auto &z=m["elevation"];
  for(int i=0;i<z.rows();++i)for(int j=0;j<z.cols();++j)z(i,j)=.001f*i+.002f*j;
  TerrainQuery query(snapshot);std::mt19937 random(20261103);std::uniform_real_distribution<double>d(-2.5,2.5);
  std::vector<grid_map::Position> points,neighbors;points.reserve(56000);neighbors.reserve(504000);
  for(int k=0;k<56000;++k){grid_map::Position p(d(random),d(random));points.push_back(p);for(int a=-1;a<=1;++a)for(int b=-1;b<=1;++b)neighbors.emplace_back(p.x()+a*.1,p.y()+b*.1);}
  std::vector<grid_map::Index> indices;indices.reserve(neighbors.size());
  for(auto &p:neighbors){grid_map::Index i;if(!m.isInside(p)||!m.getIndex(p,i)||(i<grid_map::Index::Zero()).any()||(i>=m.getSize()).any())throw std::runtime_error("benchmark needs safe interior points");indices.push_back(i);}
  std::cout<<"SCOPE centers="<<points.size()<<" neighbors="<<neighbors.size()<<" warmup=5 measured=30; components measured separately, not additive"<<std::endl;
  measure("prepared_query_504k",[&]{double sum=0;for(auto &p:neighbors){auto q=query(p.x(),p.y());sum+=q.elevation+static_cast<int>(q.state);}return sum;});
  measure("is_inside_504k",[&]{double sum=0;for(auto &p:neighbors)sum+=m.isInside(p);return sum;});
  measure("get_index_and_storage_guard_504k",[&]{double sum=0;for(auto &p:neighbors){grid_map::Index i;bool good=m.getIndex(p,i)&&(i>=grid_map::Index::Zero()).all()&&(i<m.getSize()).all();sum+=good?i.x()+i.y():0;}return sum;});
  measure("matrix_fetch_preindexed_504k",[&]{double sum=0;for(auto &i:indices){float v=z(i.x(),i.y());sum+=std::isfinite(v)?v:0;}return sum;});
  Config cfg{.1,.3f,true};size_t queries=0;
  for(auto &p:points)evaluate(p.x(),p.y(),cfg,[&](double x,double y){++queries;return query(x,y);},[](double,double){return false;});
  std::cout<<"QUERY_COUNT "<<queries<<std::endl;
  measure("nine_point_evaluate_56k",[&]{double sum=0;for(auto &p:points){auto r=evaluate(p.x(),p.y(),cfg,[&](double x,double y){return query(x,y);},[](double,double){return false;});sum+=static_cast<int>(r.decision)+r.unknown_count+r.observed_jump;}return sum;});
  z.setConstant(std::numeric_limits<float>::quiet_NaN());
  double with_exemption=0,without_exemption=0;
  measure("unknown_pass_with_startup_checks_56k",[&]{double sum=0;for(auto &p:points){auto r=evaluate(p.x(),p.y(),cfg,[&](double x,double y){return query(x,y);},[](double x,double y){return std::hypot(x,y)<=1.;});sum+=static_cast<int>(r.decision)+r.unknown_count+r.observed_jump;}with_exemption=sum;return sum;});
  measure("unknown_pass_without_startup_checks_56k",[&]{double sum=0;for(auto &p:points){auto r=evaluate(p.x(),p.y(),cfg,[&](double x,double y){return query(x,y);},[](double,double){return false;});sum+=static_cast<int>(r.decision)+r.unknown_count+r.observed_jump;}without_exemption=sum;return sum;});
  if(with_exemption!=without_exemption)throw std::runtime_error("unknown-pass decisions changed");
  boundaries();if(argc==2)reuse(argv[1]);
  std::cout<<"CHECKSUM "<<observable<<std::endl;return 0;
 }catch(const std::exception &e){std::cerr<<"ERROR "<<e.what()<<std::endl;return 1;}
}
