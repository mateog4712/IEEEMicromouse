/*BFS_MM.ino: Breadth First Search implementation for micromouse with Arduino using Standard C++ Library
 * 
 * Created By: Michael Menaker 10/16/18
 * 
 * Objectives:
 * -Use sense data to accurately determine position, velocity and acceleration of MM
 * -Traverse maze
 * -Map maze data using adjancency matrix
 * -Condense graph by omitting nodes which are dead ends
 * -Find shortest path/paths using Breadth First Search
 * -If more than one path, find path with least # of turns
 * -Convert paths to list of states
 * -Use state machine to convert states to actions performed by MM
 * -Control high speed motion using onboard sensors
 * 
 * Note:
 * -Need to put dynamically allocated global path variables into methods to conserve memory
 */
#include <StandardCplusplus.h>
#include <vector>
#include <queue>

using namespace std;

//Macros
#define VERTICES 16*16
#define FWD    0
#define L_90   15
#define R_90   16
#define L_45   17
#define R_45   18
#define L_135  19
#define R_135  20

//Global Variables
vector<vector<int>> adj;//adjacency list for cells in maze
vector<vector<int>> paths;//possible solutions to maze
vector<vector<int>> commands;//movement commands for robot
int chosen = 0;//chosen commands from command vector

//Functions

//overloaded graph condensing method
void condense_graph(vector<vector<int>> &g){
  int size = g.size();
  for(int i = 0; i < size; i++){
      condense_graph(g,i);
    }
}

//recursive method that condenses graph by removing all dead ends
void condense_graph(vector<vector<int>> &g, int i){
  if(g[i].size() == 1){
    int adj = g[i][0];
    g[i][0] = i;
    condense_graph(g, adj);
  }
}

//helper method to print a path vector<int> object
void print_path(vector<int>& path) { 
    int size = path.size(); 
    for (int i = 0; i < size; i++){
    Serial.print(path[i]);
    Serial.println(" ");
    }
  Serial.println();
} 

//checks if cell in path has been visited
int is_not_visited(int x, vector<int>& path){
  int size = path.size();
  for(int i = 0; i < size; i++){
    if(path[i] == x)
      return 1;
    }
  return 0; 
}

// method to find path for a given graph, g
void findpaths(vector<vector<int>>& g, int src, int dst){
  queue<vector<int>> q; //queue for visiting
  int shortestPath = VERTICES;
  
  vector<int> path;
  path.push_back(src);
  q.push(path);

  while(!q.empty()){
    path = q.front();
    q.pop();
    
    //we only need to evaluate path if it is shorter than previous solutions
    while(path.size() >= shortestPath) {
      path = q.front();
      q.pop();
    }
    
    int last = path[path.size() - 1];

    if(last == dst){
      //print_path(path);
      paths.push_back(path);//if last node is dst, push into paths
      shortestPath = path.size();
    }

    for(int i = 0; i < g[last].size(); i++){
      if(!is_not_visited(g[last][i], path)){ 
        vector<int> newpath(path);
        newpath.push_back(g[last][i]);
        q.push(newpath);
       }
     }
   }
}

/* pushes states into path
 * heading is represented as int values:
 * N = 0
 * NE = 1
 * E = 2
 * SE = 3
 * SW = 4
 * S = 5
 * SW = 6
 * W = 7
 * NW = 8
 */
void path_to_actions(vector<vector<int>> &paths){
  int size = paths.size();
  for(int i = 0; i < size; i++){
      int size2 = paths[i].size();
      int heading = 0;
      for(int j = 0; j < size2 - 1; j++){
        //adjacent N
        if(paths[i][j] == paths[i][j+1] - 16){
          switch(heading){
            case 0:
              commands[i].push_back(FWD);
              break;
              
            case 1:
              commands[i].push_back(L_45);
              break;
            
            case 2:
              commands[i].push_back(L_90);
              break;
            
            case 3:
              commands[i].push_back(L_135);
              break;
            
            //case 4: this couldn't happen
            case 5:
              commands[i].push_back(R_135);
              break;
            
            case 6:
              commands[i].push_back(R_90);
              break;
            
            case 7:
              commands[i].push_back(R_45);
              break;
              
            heading = 0;
          }
        }
        //adjacent S
        if(paths[i][j] == paths[i][j+1] + 16){
          switch(heading){
            case 4:
              commands[i].push_back(FWD);
              break;
              
            case 5:
              commands[i].push_back(L_45);
              break;
            
            case 6:
              commands[i].push_back(L_90);
              break;
            
            case 7: 
              commands[i].push_back(L_135);
              break;
            
            //case(0){}, this couldn't happen
            case 1:
              commands[i].push_back(R_135);
              break;
             
            case 2:
              commands[i].push_back(R_90);
              break;
            
            case 3:
              commands[i].push_back(R_45);
              break;
          }
          heading = 4;
        }
        //adjacent E
        if(paths[i][j] == paths[i][j+1] - 1){
          switch(heading){
            case 3:
              commands[i].push_back(L_45);
              break;
            
            case 4: 
              commands[i].push_back(L_90);
              break;
            
            case 5:
              commands[i].push_back(L_135);
              break;
            
            //case(6){}, this couldn't happen
            case 7:
              commands[i].push_back(R_135);
              break;
            
            case 0:
              commands[i].push_back(R_90);
              break;
            
            case 1:
              commands[i].push_back(R_45);
              break;
            
            case 2:
              commands[i].push_back(FWD);
          }          
          heading = 2;
        }
        //adjacent W
        if(paths[i][j] == paths[i][j+1] + 1){
          switch(heading){    
            case 6:
              commands[i].push_back(FWD);
              break;
              
            case 7:
              commands[i].push_back(L_45);
              break;
            
            case 0:
             commands[i].push_back(L_90);
              break;
            
            case 1:
              commands[i].push_back(L_135);
              break;
            
            //case(2){}, this couldn't happen
            case 3:
              commands[i].push_back(R_135);
              break;
            
            case 4:
              commands[i].push_back(R_90);
              break;
            
            case 5:
              commands[i].push_back(R_45);
              break;
          }
          heading = 6;
        }
        //adjacent NE diagonal
        if(paths[i][j] == paths[i][j+1] - 17){
           switch(heading){
            case 1:
              commands[i].push_back(FWD);
            
            case 2:
              commands[i].push_back(L_45);
              break;
            
            case 3:
              commands[i].push_back(L_90);
              break;
            
            case 4:
              commands[i].push_back(L_135);
              break;
           
            //case(5){}, this couldn't happen
            case 6:
              commands[i].push_back(R_135);
              break;
            
            case 7:
              commands[i].push_back(R_90);
              break;
            
            case 0:
              commands[i].push_back(R_45);
              break;
            }
            heading = 1;
          }
        
        //NW diagonal
        if(paths[i][j] == paths[i][j+1] - 15){
           switch(heading){
            case 7:
              commands[i].push_back(FWD);
              break;
              
            case 0:
              commands[i].push_back(L_45);
              break;

            case 1:
              commands[i].push_back(L_90);
              break;
 
            case 2:
              commands[i].push_back(L_135);
              break;
    
            //case(3){}, this couldn't happen
            case 4:
              commands[i].push_back(R_135);
              break;
         
            case 5:
              commands[i].push_back(R_90);
              break;
         
            case 6:
              commands[i].push_back(R_45);
              break;
          }
          heading = 7;
        }
        //SW diagonal
        if(paths[i][j] == paths[i][j+1] + 17){
           switch(heading){
            case 6:
              commands[i].push_back(L_45);
              break;
           
            case 7:
              commands[i].push_back(L_90);
              break;
            
            case 0:
              commands[i].push_back(L_135);
              break;
            
            //case(1){}, this couldn't happen
            case 2:
              commands[i].push_back(R_135);
              break;
          
            case 3:
              commands[i].push_back(R_90);
              break;
            
            case 4:
              commands[i].push_back(R_45);
              break;
            
            case 5:
              commands[i].push_back(FWD);
              break;
          }
          heading = 6;
        }
        //SE diagonal
        if(paths[i][j] == paths[i][j+1] + 15){
           switch(heading){
            case 3:
              commands[i].push_back(FWD);
            
            case 4:
              commands[i].push_back(L_45);
              break;
           
            case 5:
              commands[i].push_back(L_90);
              break;
        
            case 6:
              commands[i].push_back(L_135);
              break;
          
            //case(7){}, this couldn't happen
            case 0:
              commands[i].push_back(R_135);
              break;
           
            case 1:
              commands[i].push_back(R_90);
              break;
           
            case 2:
              commands[i].push_back(R_45);
              break;
          }
          heading = 3;          
        }
      }
    }
}

int least_turns(vector<vector<int>> &commands){
  int size = commands.size();
  for(int i = 0; i < size; i++){
    int size2 = commands[i].size();
    int turns = 0;
    int best = 128;
    int chosen;
    for(int j = 0; j< size2; j++){
      if(commands[i][j] != 1)
        turns++;
      if (turns > best)
        j = size2;
      }
    if(turns < best)
      best = turns;
      chosen = i;
    }
  return chosen;
}

void consolidate_fwd(vector<vector<int>> &commands){
  int size = commands.size();
  int prev = 15;
  for(int i = 0; i < size; i++){
      if(commands[chosen][i] < 15 && prev == 1){
        commands[chosen][i-1]++;
        commands[chosen].erase(commands[chosen].begin() + i);
        size--;
      }
    }
}


void setup(){}
void loop(){}
