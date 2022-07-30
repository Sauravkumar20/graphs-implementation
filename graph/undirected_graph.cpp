#include<iostream>
#include<bits/stdc++.h>
#include<queue>
using namespace std;

unordered_map<int,int>parent;
class Graph {
    int V;
    vector<int>*adj;
    public:
    Graph(int V);
    
    void addEdge(int v,int u);
    void traversal();
    void bfs(int src);
    void dfs(int src);
    void dfsUtil(int src, vector<bool>&visited);
    bool isCyclic(int src);
    bool Cyclic(int V,vector<vector<int>>&edges);
    void noOfConnectedComp(int V);
    void ConnectedCompUtils(int V,vector<vector<int>>&edges);
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
    adj = new vector<int>[V];
}

void Graph:: addEdge(int v,int u){
    adj[v].push_back(u);
    adj[u].push_back(v);

}

void Graph::traversal(){
    for(int v=0;v<V;v++){
        cout<<v<<"-->";
        for(auto x: adj[v]){
            cout<<x<<" ";
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
            int v = adj[u][i];
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
        int v = adj[src][i];
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
            int v = adj[u][i];
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
        int u = i[0];
        int v = i[1];
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
        ds.Union_Set(edges[i][0],edges[i][1]);
    }

    noOfConnectedComp(V);
}


int main() {
    int V = 9;
    vector<vector<int>> edges = {{0,1},{0,2},{1,3},{2,4},{3,5},{3,6},{4,7},{5,8}};
     Graph g(V);
    g.addEdge(0, 1);
    g.addEdge(0, 2);
    g.addEdge(1, 3);
    g.addEdge(2, 4);
    g.addEdge(3, 5);
    g.addEdge(3, 6);
    g.addEdge(4, 7);
    g.addEdge(5, 8);

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
    
    return 0;


}