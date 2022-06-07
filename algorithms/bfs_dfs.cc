
#include <iostream>
#include <queue>
#include <stack>
#define Max 20
using namespace std;

class Vertex {
 public:
  Vertex(char lab) {
    Label = lab;
    wasVisited = false;
  }

 public:
  bool wasVisited;
  char Label;
};

class Graph {
 public:
  Graph();                           //构造函数
  ~Graph();                          //析构函数
  void addVertex(char lab);          //增加一个节点
  void addEdge(int start, int end);  //增加一条边，起点到终点
  void printMatrix();                //打印出矩阵
  void showVertex(int v);
  void DFS();
  void BFS();

 private:
  Vertex* vertexList[Max];  //存放每个节点的指针的数组
  int nVerts;               //实际数量
  int adjMat[Max][Max];     //矩阵
  int getAdjUnvisitedVertex(int v);  //获得其相邻的节点 在邻接矩阵里找其最近的
};

void Graph::DFS() {
  stack<int> gStack;
  vertexList[0]->wasVisited = true;
  showVertex(0);
  gStack.push(0);
  int v;
  while (gStack.size() > 0) {
    v = getAdjUnvisitedVertex(gStack.top());
    if (v == -1) {
      cout << "出:" << gStack.top() << endl;  //查看出栈情况
      gStack.pop();
    } else {
      vertexList[v]->wasVisited = true;
      showVertex(v);
      gStack.push(v);
    }
  }

  for (int j = 0; j < nVerts; j++)  //重新置为未访问.
    vertexList[j]->wasVisited = false;
}

void Graph::BFS() {
  queue<int> gQueue;
  vertexList[0]->wasVisited = true;
  showVertex(0);
  gQueue.push(0);
  int vert1, vert2;

  while (gQueue.size() > 0) {
    vert1 = gQueue.front();
    gQueue.pop();
    vert2 = getAdjUnvisitedVertex(vert1);
    while (vert2 != -1) {
      vertexList[vert2]->wasVisited = true;
      showVertex(vert2);
      gQueue.push(vert2);
      vert2 = getAdjUnvisitedVertex(vert1);
    }
  }

  cout << endl;
  for (int j = 0; j < nVerts; j++)  //重新置为未访问
    vertexList[j]->wasVisited = false;
}

//得到其相邻节点
int Graph::getAdjUnvisitedVertex(int v) {
  for (int j = 0; j < nVerts; j++) {
    if ((adjMat[v][j] == 1) &&
        (vertexList[j]->wasVisited ==
         false))  //找其第一个相邻（邻接）的且没有被访问过的
      return j;
  }
  return -1;
}

void Graph::showVertex(int v)  //展示该下标对应的节点
{
  cout << vertexList[v]->Label << " ";
}

Graph::Graph() {
  nVerts = 0;
  for (int i = 0; i < Max; i++)
    for (int j = 0; j < Max; j++) adjMat[i][j] = 0;
}

void Graph::addVertex(char lab) {
  vertexList[nVerts++] = new Vertex(lab);  //
}

void Graph::addEdge(int start, int end) {
  adjMat[start][end] = adjMat[end][start] = 1;
}

void Graph::printMatrix() {
  for (int i = 0; i < nVerts; i++) {
    for (int j = 0; j < nVerts; j++) {
      cout << adjMat[i][j] << " ";
    }
    cout << endl;
  }
}

Graph::~Graph() {
  for (int i = 0; i < nVerts; i++) {
    delete vertexList[i];
  }
}

int main() {
  Graph g;
  g.addVertex('A');  // 0
  g.addVertex('B');  // 1
  g.addVertex('C');  // 2
  g.addVertex('D');  // 3
  g.addVertex('E');  // 4
  g.addEdge(0, 1);   // A-B
  g.addEdge(1, 4);   // B-E
  g.addEdge(2, 4);   // C-E

  g.addEdge(0, 3);  // A-D
  g.addEdge(3, 0);
  g.addEdge(3, 4);

  g.printMatrix();

  cout << "DFS搜索" << endl;
  g.DFS();
  cout << endl;
  cout << "BFS搜索" << endl;
  g.BFS();
  return 0;
}