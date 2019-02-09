#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <climits>
#include <cmath>
using namespace std;

class Point {
public:
    int X, Y;
    Point(int x = 0, int y = 0) {
        X = x; Y = y;
    }
    Point(const Point &obj)
    {      
        X = obj.X;
        Y = obj.Y;
    }
};


class PathNode {
private:
	//Координата точки
    Point* position;
	//Соседние клетки
    vector<Point*> neighbours;
public:
	//Мой вид лабиринта
    static string* myMaze;
	//Размер лабиринта
	static int row;
	static int cell;
public:
    PathNode(Point* p) {
        position = p;
    }
	void addNeighbours(vector<Point*> n) {
        neighbours = n;
    }
    void goNext() {
		myMaze[position->X][position->Y] = 'y';
        for(int i = 0; i < size;++i){
            int nX = neighbours[i]->X;
    		int nY = neighbours[i]->Y;
            if(myMaze[nX][nY]!='y'){
                if(position->X>nX) cout << "UP"<< endl;
                else if(position->X<nX) cout << "DOWN"<< endl;
                else if(position->Y>nY) cout << "LEFT"<< endl;
                else cout << "RIGHT" << endl;
            }
        }
    }

    ~PathNode() {
        for(int i = 0; i < neighbours.size();++i)
            delete neighbours[i];
        neighbours.clear();
        delete cameFrom;
    }
};

vector<Point*> getNeighbours(Point* cur, string* M) {
    vector<Point*> Neighbours;
    int X = cur->getX();
    int Y = cur->getY();
    if(M[X][Y+1]=='.') {
        Neighbours.push_back(new Point(X,Y+1));
    }else if(M[X][Y-1]=='.') {
        Neighbours.push_back(new Point(X,Y-1));
    }else if(M[X+1][Y]=='.') {
        Neighbours.push_back(new Point(X+1,Y));
    }else if(M[X-1][Y]=='.') {
        Neighbours.push_back(new Point(X-1,Y));
    }
    return Neighbours;
}

void compareMaze(string* real, string* my, int R, int C) {
    for(int i = 0; i < R; ++i) {
        for(int j = 0; j < C; ++j){
            if(real[i][j]!=my[i][j] && my[i][j]!='y')
                my[i][j] = real[i][j];
        }
    }
}

int PathNode::cell = 0;
int PathNode::row = 0;
string* PathNode::myMaze = NULL;

int main()
{
    int R; // number of rows.
    int C; // number of columns.
    int A; // number of rounds between the time the alarm countdown is activated and the time the alarm goes off.
    cin >> R >> C >> A; cin.ignore();
    PathNode::cell = C;
    PathNode::row = R;
    PathNode::myMaze = new string[R];
    for(int i = 0;i < R;++i){
        PathNode::myMaze[i].resize(C,'?');
    }
    string* myMaze;  
    string* maze = new string[R];
    PathNode* cur;
    Point* curPoint;
	// game loop
    while (1) {
        int KR; // row where Kirk is located.
        int KC; // column where Kirk is located.
        cin >> KR >> KC; cin.ignore();
        myMaze =  PathNode::myMaze;
        for (int i = 0; i < R; i++) {
            string row; // C of the characters in '#.TC?' (i.e. one line of the ASCII maze).
            cin >> row; cin.ignore();
            maze[i] = row;
        }
        for(int i = 0;i < R;++i) {
            cerr << myMaze[i] << endl;
        }
        compareMaze(maze, myMaze, R, C);
        PathNode::myMaze = myMaze;
        
		curPoint = new Point(KR,KC);
		cur = new PathNode(curPoint);
		cur->addNeighbours(getNeighbours(cur, passedMaze)); 
		cur->goNext();

        // Write an action using cout. DON'T FORGET THE "<< endl"
        // To debug: cerr << "Debug messages..." << endl;
        for (int i = 0; i < R; i++) {
            cerr << maze[i] <<endl;
        }
    }
}