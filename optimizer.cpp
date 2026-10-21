#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <limits>
#include <chrono>
#include <queue>
using namespace std;
struct Point{string name;double x,y,demand;int priority;};
struct Edge{int to;double w;};
struct Result{vector<int> route;double distance=0,ms=0;};
double euclid(const Point&a,const Point&b){return hypot(a.x-b.x,a.y-b.y);} 
double trafficFactor(const string&t){return t=="heavy"?1.55:t=="medium"?1.25:1.0;}
vector<vector<Edge>> buildGraph(const vector<Point>&p,const string&traffic,const vector<pair<int,int>>&blocked){
 int n=p.size(); vector<vector<Edge>>g(n); auto isBlocked=[&](int a,int b){for(auto q:blocked)if((q.first==a&&q.second==b)||(q.first==b&&q.second==a))return true;return false;};
 // dense road graph: connect each point to its 4 nearest neighbours; fallback connectivity chain
 for(int i=0;i<n;i++){vector<pair<double,int>>v;for(int j=0;j<n;j++)if(i!=j)v.push_back({euclid(p[i],p[j]),j});sort(v.begin(),v.end());for(int k=0;k<(int)min<size_t>(4,v.size());k++){int j=v[k].second;if(!isBlocked(i,j))g[i].push_back({j,v[k].first*trafficFactor(traffic)});}}
 // make directed neighbour choices symmetric
 for(int i=0;i<n;i++)for(auto e:g[i]){bool ok=false;for(auto z:g[e.to])if(z.to==i)ok=true;if(!ok&&!isBlocked(i,e.to))g[e.to].push_back({i,e.w});}
 return g;
}
pair<double,vector<int>> shortest(const vector<Point>&p,const vector<vector<Edge>>&g,int s,int t,bool astar,int &expanded){
 int n=p.size();const double INF=1e100;vector<double>d(n,INF);vector<int>par(n,-1);using Q=pair<double,int>;priority_queue<Q,vector<Q>,greater<Q>>q;d[s]=0;q.push({0,s});expanded=0;
 while(!q.empty()){auto [f,u]=q.top();q.pop();double expect=d[u]+(astar?euclid(p[u],p[t]):0);if(f>expect+1e-9)continue;expanded++;if(u==t)break;for(auto e:g[u]){double nd=d[u]+e.w;if(nd<d[e.to]){d[e.to]=nd;par[e.to]=u;q.push({nd+(astar?euclid(p[e.to],p[t]):0),e.to});}}}
 vector<int>path;if(d[t]>=INF/2)return {INF,path};for(int v=t;v!=-1;v=par[v])path.push_back(v);reverse(path.begin(),path.end());return {d[t],path};
}
double metric(const vector<Point>&p,const vector<vector<Edge>>&g,int a,int b){int ex=0;auto r=shortest(p,g,a,b,false,ex);return r.first;}
double routeDistance(const vector<Point>&p,const vector<vector<Edge>>&g,const vector<int>&r){double d=0;for(size_t i=1;i<r.size();i++)d+=metric(p,g,r[i-1],r[i]);if(r.size()>1)d+=metric(p,g,r.back(),r[0]);return d;}
vector<int> nearestNeighbor(const vector<Point>&p,const vector<vector<Edge>>&g,const vector<int>&nodes){vector<int>r{0};vector<int>left=nodes;while(!left.empty()){int cur=r.back(),bi=-1;double bd=1e100;for(int i=0;i<(int)left.size();i++){double d=metric(p,g,cur,left[i]);if(d<bd){bd=d;bi=i;}}if(bi<0||bd>=1e99)break;r.push_back(left[bi]);left.erase(left.begin()+bi);}return r;}
vector<int> twoOpt(const vector<Point>&p,const vector<vector<Edge>>&g,vector<int>r){bool imp=true;int passes=0;while(imp&&passes++<30){imp=false;double best=routeDistance(p,g,r);for(size_t i=1;i+1<r.size();i++)for(size_t k=i+1;k<r.size();k++){auto nr=r;reverse(nr.begin()+i,nr.begin()+k+1);double nd=routeDistance(p,g,nr);if(nd+1e-8<best){r=nr;best=nd;imp=true;goto next;}}next:;}return r;}
string esc(string s){string o;for(char c:s){if(c=='"')o+="\\\"";else if(c=='\\')o+="\\\\";else o+=c;}return o;}
int main(){ios::sync_with_stdio(false);cin.tie(nullptr);int n,vehicles;double capacity;string traffic;if(!(cin>>n>>vehicles>>capacity>>traffic)||n<2)return 1;vector<Point>p(n);string line;getline(cin,line);for(int i=0;i<n;i++){getline(cin,line);stringstream ss(line);string a;getline(ss,p[i].name,'|');getline(ss,a,'|');p[i].x=stod(a);getline(ss,a,'|');p[i].y=stod(a);getline(ss,a,'|');p[i].demand=stod(a);getline(ss,a,'|');p[i].priority=stoi(a);}int bc;cin>>bc;vector<pair<int,int>>blocked(bc);for(auto &b:blocked)cin>>b.first>>b.second;auto g=buildGraph(p,traffic,blocked);
 vector<int>all;for(int i=1;i<n;i++)all.push_back(i);auto t0=chrono::high_resolution_clock::now();auto nn=nearestNeighbor(p,g,all);double nnD=routeDistance(p,g,nn);auto t1=chrono::high_resolution_clock::now();auto opt=twoOpt(p,g,nn);double optD=routeDistance(p,g,opt);auto t2=chrono::high_resolution_clock::now();double nnms=chrono::duration<double,milli>(t1-t0).count(),optms=chrono::duration<double,milli>(t2-t1).count();
 // capacity-aware multi-vehicle assignment: priority first, then first-fit by remaining capacity
 vector<int>order=all;sort(order.begin(),order.end(),[&](int a,int b){return p[a].priority>p[b].priority;});vector<vector<int>>assign(max(1,vehicles));vector<double>load(assign.size());for(int id:order){int best=-1;double rem=-1;for(int v=0;v<(int)assign.size();v++)if(load[v]+p[id].demand<=capacity&&capacity-load[v]>rem){best=v;rem=capacity-load[v];}if(best<0)best=min_element(load.begin(),load.end())-load.begin();assign[best].push_back(id);load[best]+=p[id].demand;}
 cout<<fixed<<setprecision(4)<<"{\"nearest_distance\":"<<nnD<<",\"optimized_distance\":"<<optD<<",\"saving_percent\":"<<(nnD?100*(nnD-optD)/nnD:0)<<",\"traffic\":\""<<traffic<<"\",\"algorithms\":[";
 cout<<"{\"name\":\"Nearest Neighbor\",\"distance\":"<<nnD<<",\"time_ms\":"<<nnms<<",\"complexity\":\"O(n^2)\"},{\"name\":\"NN + 2-Opt\",\"distance\":"<<optD<<",\"time_ms\":"<<optms<<",\"complexity\":\"O(n^2) per pass\"}";
 int exD=0,exA=0;auto ds=shortest(p,g,0,n-1,false,exD);auto as=shortest(p,g,0,n-1,true,exA);cout<<",{\"name\":\"Dijkstra (depot→last)\",\"distance\":"<<(ds.first>1e90?-1:ds.first)<<",\"time_ms\":0,\"complexity\":\"O((V+E) log V)\",\"expanded\":"<<exD<<"},{\"name\":\"A* (depot→last)\",\"distance\":"<<(as.first>1e90?-1:as.first)<<",\"time_ms\":0,\"complexity\":\"O(E) typical heuristic\",\"expanded\":"<<exA<<"}],\"route\":[";
 for(size_t i=0;i<opt.size();i++){if(i)cout<<",";cout<<"{\"index\":"<<opt[i]<<",\"name\":\""<<esc(p[opt[i]].name)<<"\"}";}cout<<"],\"vehicles\":[";
 for(int v=0;v<(int)assign.size();v++){auto r=nearestNeighbor(p,g,assign[v]);r=twoOpt(p,g,r);if(v)cout<<",";cout<<"{\"vehicle\":"<<v+1<<",\"load\":"<<load[v]<<",\"capacity\":"<<capacity<<",\"distance\":"<<routeDistance(p,g,r)<<",\"route\":[";for(size_t i=0;i<r.size();i++){if(i)cout<<",";cout<<r[i];}cout<<"]}";}cout<<"]}";
}
