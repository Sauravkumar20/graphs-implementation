#include<iostream>
#include<bits/stdc++.h>
#include<queue>
using namespace std;

unordered_map<int,int>parent;
typedef pair<int,int>ipair;

class Graph {
    int V;
    vector<pair<int,int>>*adj;
    public:
    Graph(int V);
    
    void addEdge(int v,int u,int w);
    void traversal();
    void bfs(int src);
    void dfs(int src);
    void dfsUtil(int src, vector<bool>&visited);
    bool isCyclic(int src);
    bool Cyclic(int V,vector<vector<int>>&edges);
    void noOfConnectedComp(int V);
    void ConnectedCompUtils(int V,vector<vector<int>>&edges);
    int Krushkal_Algorithms(int V,vector<vector<int>>&edges);
    int Prims_Algorithms(int V);
    void Dijkstra_Algorithms(int V,int src);

};

class DisjointSet{
    public:

    void MakeSet(int V){
        for (int i = 0; i < V; i++){
            parent[i]=i;

        }
    }

    int Find_Union(int v){
        if(v==parent[v]){
            return v;
        }
        return parent[v]= Find_Union(parent[v]);
    }

    void Union_Set(int a, int b){
        int x = Find_Union(a);
        int y = Find_Union(b);
        if(x!=y){
            parent[y]=x;
        }
    }

};

Graph::Graph(int V){
    this->V=V;
    adj = new vector<pair<int,int>>[V];
}

void Graph:: addEdge(int v,int u,int w){
    adj[v].push_back(make_pair(u,w));
    adj[u].push_back(make_pair(v,w));

}

void Graph::traversal(){
    for(int v=0;v<V;v++){
        cout<<v<<"-->";
        for(auto x: adj[v]){
            cout<<x.first<<" ";
        }
        cout<<endl;
    }
}

void Graph:: bfs(int src){
    vector<int>visited(V,false);
    queue<int>q;
    q.push(src);
    visited[src]=true;
    while(!q.empty()){
        int u = q.front();
        q.pop();
        cout<<u<<" ";
        for(int i=0;i<adj[u].size();i++){
            int v = adj[u][i].first;
            if(visited[v]==false){
                q.push(v);
                visited[v]= true;
            }
        }
    }
}

void Graph::dfsUtil(int src,vector<bool>&visited){
    cout<<src<<" ";
    for(int i=0;i<adj[src].size();i++){
        int v = adj[src][i].first;
        if(!visited[v]){
            visited[v]= true;
            dfsUtil(v,visited);
        }
    }

}
void Graph::dfs(int src){
    vector<bool>visited(V,false);
    visited[src]=true;
    dfsUtil(src,visited);
}

bool Graph::isCyclic(int src){  // checking cyclic or not using bfs
    vector<int>visited(V,-1);
    queue<int>q;
    q.push(src);
    visited[src]=0;
    while(!q.empty()){
        int u = q.front();
        visited[u]=1;
        q.pop();
        for(int i=0;i<adj[u].size();i++){
            int v = adj[u][i].first;
            if(visited[v]==-1){
                q.push(v);
                visited[v]=0;
            }
            else if(visited[v]==0){
                return true;
            }
        }
    }
    return false;
}

bool Graph:: Cyclic(int V,vector<vector<int>>& edges){
    DisjointSet ds;
    bool cyclic = false;
    ds.MakeSet(V);
    for(auto i: edges){
        int u = i[1];
        int v = i[2];
        int x = ds.Find_Union(u);
        int y = ds.Find_Union(v);
        if(x==y){
            cout<<"saurav"<<endl;
            cyclic = true;
        }else{
            ds.Union_Set(u,v);
        }
    }
    return cyclic;
}

void Graph::noOfConnectedComp(int V){
    DisjointSet ds;
    set<int>s;
    for(int i=0;i<V;i++){
        s.insert(ds.Find_Union(parent[i]));
    }
    cout<<s.size()<<endl;
}

void Graph::ConnectedCompUtils(int V,vector<vector<int>>&edges){
    DisjointSet ds;
    ds.MakeSet(V);
    for(int i=0;i<edges.size();i++){
        ds.Union_Set(edges[i][1],edges[i][2]);
    }

    noOfConnectedComp(V);
}

int Graph::Krushkal_Algorithms(int V,vector<vector<int>>&edges){
    int cost =0;
    DisjointSet ds;
    ds.MakeSet(V);
    sort(edges.begin(),edges.end());
    for(int i=0;i<edges.size();i++){
    }
    for(auto i: edges){
        int u = i[1];
        int v = i[2];
        int w = i[0];
        if(ds.Find_Union(u)!=ds.Find_Union(v)){
            cost = cost+w;
            ds.Union_Set(u,v);
        }else{
            cout<<"dadadd"<<endl;
        }
    }
    return cost;
}

int Graph::Prims_Algorithms(int V){
    int cost=0;
    priority_queue<ipair,vector<ipair>,greater<ipair>>pq;
    int src = 0;
    vector<int>key(V,INT_MAX);
    vector<int>parent(V,-1);
    vector<bool>inMST(V,false);

    pq.push(make_pair(0,src));
    key[src]=0;
    while(!pq.empty()){
        int u = pq.top().second;
        pq.pop();
        if(inMST[u]==true){
            continue;
        }
        inMST[u]= true;
        for(auto x: adj[u]){
            int v = x.first;
            int weight = x.second;
            if(inMST[v]==false && key[v]>weight){
                key[v]= weight;
                pq.push(make_pair(key[v],v));
                parent[v] = u;
            }
        }
    }
    cout<<"minimum spanning tree of graph"<<endl;
    for(int i=1;i<V;i++){
        cout<<parent[i]<<" "<<i<<endl;
    }

    for(auto x:key){
        cost = cost+x;
    }
    return cost;
}

void Graph:: Dijkstra_Algorithms(int V,int src){

    priority_queue<ipair,vector<ipair>,greater<ipair>>pq;
    vector<int>dist(V,INT_MAX);
    pq.push(make_pair(0,src));
    dist[src]=0;
    while(!pq.empty()){
        int u = pq.top().second;
        pq.pop();
        for(auto x: adj[u]){
            int v = x.first;
            int weight = x.second;
            if(dist[v]>dist[u]+weight){
                dist[v]= dist[u]+weight;
                pq.push(make_pair(dist[v],v));
            }
        }
    }
    cout<<"shortest path from the given source to all the vertex "<<endl;
    for(int i=0;i<V;i++){
        cout<<i<<" -->"<<dist[i]<<endl;
    }
}


int main() {
    int V = 9;
    vector<vector<int>> edges = {{5,0,1},{3,0,2},{1,1,3},{2,2,4},{4,3,5},{2,3,6},{3,4,7},{1,5,8},{2,5,6}};
    Graph g(V);
    g.addEdge(0, 1, 5);
    g.addEdge(0, 2, 3);
    g.addEdge(1, 3, 1);
    g.addEdge(2, 4, 2);
    g.addEdge(3, 5, 4);
    g.addEdge(3, 6, 2);
    g.addEdge(4, 7, 3);
    g.addEdge(5, 8, 1);
    g.addEdge(5, 6, 2);


    g.traversal();
    cout<<"breadth first Search"<<endl;
    g.bfs(0); // basically we can start bfs from anypoint;
    cout<<endl;
    cout<<"depth first Search"<<endl;
    g.dfs(0);
    cout<<endl;

    if(g.isCyclic(0)){
        cout<<"contain cycle"<<endl;
    }else{
        cout<<"not contain cycle"<<endl;
    }

    if(g.Cyclic(V,edges)){
        cout<<"contain cycle"<<endl;
    }else{
        cout<<"not contain cycle"<<endl;
    }

    cout<<"Number of Connected Component:";
    g.ConnectedCompUtils(V,edges);

    cout<<"cost of the minimum spanning tree(MST) is: "<<g.Krushkal_Algorithms(V,edges)<<endl;

    cout<<"cost of the minimum spanning tree(MST) is: ";
    cout<<g.Prims_Algorithms(V)<<endl;

    cout<<"Dijkstra Algorithm for shortest distance from the source to the all vertex is"<<endl;
    g.Dijkstra_Algorithms(V,0);

    return 0;
}