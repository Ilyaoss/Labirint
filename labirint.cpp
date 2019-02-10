#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
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
    friend bool operator== (Point& p1, Point& p2);
};
bool operator== (Point& p1, Point& p2) {
    return p1.X == p2.X && p1.Y == p2.Y;
}

class PathNode {
private:
	//Координата точки
    Point* position;
	//Соседние клетки
    vector<Point*> neighbours;
	//Откуда пришли
	PathNode* cameFrom;
	//Длина от начала пути
	int pathLengthFromStart;
    //Эвристическая длина
    int heuristicEstimatePathLength;
public:
	//Стек точек развилок(более 1 варианта пути)
    static vector<Point*> branches;
	//Мой вид лабиринта
    static string* myMaze;
	//Реальный вид лабиринта
    static string* realMaze;
	//Размер лабиринта
	static int row;
	static int cell;
	//Счётчик, для приостановки бесконечного цикла в main
	//Приостанавливаем, чтобы предотвратить создание новый точек, пока движемся
	static int counter;
public:
    PathNode(Point* p, PathNode* cf = NULL) {
        position = p;
        cameFrom = cf;
        pathLengthFromStart = 0;
        heuristicEstimatePathLength = 0;
		neighbours = getNeighbours(p,myMaze);
    }

    void goNext() {
		int size = neighbours.size();
		myMaze[position->X][position->Y] = 'y';
		//Если больше 1 пути, то помечаем развилку(b-branch)
		if(size > 1) {
		    myMaze[position->X][position->Y] = 'b';
		    branches.push_back(this->position);
		}
		//Если нет соседей, возращаемся к последнему разветвлению
		else if(size < 1) {
		    Point* branch;
    	    while (branches.size() > 0) {
    		    branch = branches.back();
    		    branches.pop_back();
    		    //Проверяем осталось ли там разветвление
    		    if(isBrunch(branch)){ 
					for (int i = 0; i < row; i++) {
						cerr << myMaze[i] <<endl;
					}
					//cerr << "cur pos" << position->X << " " << position->Y << endl;
					//cerr << "brunch pos" << branch->X << " " << branch->Y << endl;
					//Вычисляем путь до последнего разветвления
					vector<string> pathToLastBranch = findPath(position,branch);
					//Выставляем счётсчик и движемся
        			counter = pathToLastBranch.size();
					for(int i = 0; i < pathToLastBranch.size(); ++i) {
						cout << pathToLastBranch[i] << endl;
					}
					return;
				}    			
				//Если не осталось изменяем метку
    			else myMaze[branch->X][branch->Y] = 'y';
    	    }
			//Если разветвлений не осталось, то лабиринт изучен и можно идти к точке C и Т
    	    cerr << "current pos" << position->X << " " << position->Y << endl;
			Point* contorPoint = findC();
			cerr << "C pos" << contorPoint->X << " " << contorPoint->Y << endl;
			vector<string> pathToC = findPath(position,contorPoint);
			counter = pathToC.size();
			for(int i = 0;i < pathToC.size();++i){
				cout << pathToC[i] << endl;
			}
			Point* startPoint = findT();
			vector<string> pathToT = findPath(contorPoint,startPoint);
			counter += pathToT.size();
			for(int i = 0;i < pathToT.size();++i){
				cout << pathToT[i] << endl;
			}
		}
		
        for(int i = 0; i < size;++i){
            int nX = neighbours[i]->X;
    		int nY = neighbours[i]->Y;
            if(myMaze[nX][nY]!='y'){
                if(position->X>nX) cout << "UP"<< endl;
                else if(position->X<nX) cout << "DOWN"<< endl;
                else if(position->Y>nY) cout << "LEFT"<< endl;
                else cout << "RIGHT" << endl;
                break;
            }
        }
    }
    int estimateFullPathLength() {
        return pathLengthFromStart + heuristicEstimatePathLength;
    }
    int getDistanceBetweenNeighbours() {
        return 1;
    }
    int getHeuristicPathLength(Point* from, Point* to) {
        return abs(from->X - to->X) + abs(from->Y - to->Y);
    }
    
    vector<string> findPath(Point* start, Point* goal) {
        // Результируеющий массив
        vector<string> result;
        // Массив ожидающих рассмотрения и пройденных точек
        vector<PathNode*> closedSet;
        vector<PathNode*> openSet; 
        // Начинаем искать
        PathNode* startNode = new PathNode(start);
        startNode->pathLengthFromStart = 0,
        startNode->heuristicEstimatePathLength = getHeuristicPathLength(start, goal);
        openSet.push_back(startNode);
        while (openSet.size() > 0)
        {
            // Берём точку ближайшую к цели
            sort(openSet.begin(),openSet.end());
            auto currentNode = openSet.front();
            // Если совапли, то пути найден
            //cerr << currentNode->position->X << " " << currentNode->position->Y << endl;
            //cerr << goal->X << " " << goal->Y << endl;
            if (*(currentNode->position) == *(goal))
                return getPathForNode(currentNode);
            // Переносим точку из списка ожидающий в список пройденных
            openSet.erase(openSet.begin());
            closedSet.push_back(currentNode);
            // Ищем соседей
            vector<PathNode*> neighbourNodes = getNeighbours(currentNode, goal);
            for(int i = 0; i < neighbourNodes.size();++i) {
                PathNode* neighbourNode = neighbourNodes[i];
				// Если сосед уже есть в списке пройденных - пропускаем
                int SamePosition = count_if(closedSet.begin(),closedSet.end(),[neighbourNode](PathNode* closed)
                {
                    return *(neighbourNode->position) == *(closed->position);
                });
                if (SamePosition > 0)
                    continue;
				// Ищем в списке не пройденных
                auto openNode = find_if(openSet.begin(),openSet.end(),[neighbourNode](PathNode* opened){
                    return *(neighbourNode->position) == *(opened->position);
                });
                // Если не нашли - то добавляем
                if (openNode == openSet.end())
                    openSet.push_back(neighbourNode);
				// Если же сосед в списке на рассмотрение — проверяем,
				// Если X.G + расстояние от X до Y < Y.G, значит мы пришли в точку Y более коротким путем, 
				// Заменяем Y.G на X.G + расстояние от X до Y, а точку, из которой пришли в Y на X.
                else if (openNode[0]->pathLengthFromStart > neighbourNode->pathLengthFromStart)
                {
                    // 
                    openNode[0]->cameFrom = currentNode;
                    openNode[0]->pathLengthFromStart = neighbourNode->pathLengthFromStart;
                }
            }
        }
		// Если список точек на рассмотрение пуст, а до цели мы так и не дошли — значит маршрут не существует
        return result;
    }
    // Возвращаем комманды движения к точке
	vector<string> getPathForNode(PathNode* pathNode) {
	    vector<string> path;
		vector<PathNode*> result;
		PathNode* currentNode = pathNode;
		while (currentNode != NULL)
		{	
			result.push_back(currentNode);
			currentNode = currentNode->cameFrom;
		}
		reverse(result.begin(), result.end());
		for(int i = 0; i < result.size() - 1 ; ++i) {
		    if(result[i]->position->X < result[i+1]->position->X) path.push_back("DOWN");
		    else if(result[i]->position->X > result[i+1]->position->X) path.push_back("UP");
		    else if(result[i]->position->Y < result[i+1]->position->Y) path.push_back("RIGHT");
		    else path.push_back("LEFT");
		}
		return path;
	}
	
	//Получаем соседние точки
	vector<Point*> getNeighbours(Point* cur, string* M) {
		vector<Point*> Neighbours;
		int X = cur->X;
		int Y = cur->Y;
		if(M[X][Y+1]=='.') {
			Neighbours.push_back(new Point(X,Y+1));
		} 
		if(M[X][Y-1]=='.') {
			Neighbours.push_back(new Point(X,Y-1));
		} 
		if(M[X+1][Y]=='.') {
			Neighbours.push_back(new Point(X+1,Y));
		} 
		if(M[X-1][Y]=='.') {
			Neighbours.push_back(new Point(X-1,Y));
		}
		return Neighbours;
	}
	
	vector<PathNode*> getNeighbours(PathNode* pathNode, Point* goal) {
    	vector<PathNode*> result;
    
    	vector<Point*> neighbourPoints;
    	neighbourPoints.push_back(new Point(pathNode->position->X + 1, pathNode->position->Y));
    	neighbourPoints.push_back(new Point(pathNode->position->X - 1, pathNode->position->Y));
    	neighbourPoints.push_back(new Point(pathNode->position->X, pathNode->position->Y + 1));
    	neighbourPoints.push_back(new Point(pathNode->position->X, pathNode->position->Y - 1));
    
    	for(int i = 0; i < neighbourPoints.size(); ++i)
    	{
    		// Проверяем, что по клетке можно ходить.
    		if ((myMaze[neighbourPoints[i]->X][neighbourPoints[i]->Y] != '?') && (myMaze[neighbourPoints[i]->X][neighbourPoints[i]->Y] != '#'))
    		{
        		// Заполняем данные для точки маршрута.
        		PathNode* neighbourNode = new PathNode(neighbourPoints[i], pathNode);
        		neighbourNode->pathLengthFromStart = pathNode->pathLengthFromStart + getDistanceBetweenNeighbours();
        		neighbourNode->heuristicEstimatePathLength = getHeuristicPathLength(neighbourPoints[i], goal);
        		result.push_back(neighbourNode);
    		}
    	}
    	return result;
    }
    
	Point* findC() {
    	for(int i = 0; i < row; ++i) {
            for(int j = 0; j < cell; ++j){
                if(myMaze[i][j]=='C')
                    return new Point(i,j);
            }
        }
    	return NULL;
    }
    
	Point* findT() {
    	for(int i = 0; i < row; ++i) {
            for(int j = 0; j < cell; ++j){
                if(realMaze[i][j]=='T')
                    return new Point(i,j);
            }
        }
    	return NULL;
    }
    
	bool isBrunch(Point* p) {
        int X = p->X;
        int Y = p->Y;
        if(myMaze[X][Y+1] == '.' || myMaze[X][Y-1] == '.'
            || myMaze[X+1][Y] == '.' || myMaze[X-1][Y] == '.')
            return true;
        return false;
    }
	
	bool operator< (PathNode* a) {
        return (this->estimateFullPathLength() < a->estimateFullPathLength());
    }
    
    ~PathNode() {
        for(int i = 0; i < neighbours.size();++i)
            delete neighbours[i];
        neighbours.clear();
        delete cameFrom;
    }
};

void compareMaze(string* real, string* my, int R, int C) {
    for(int i = 0; i < R; ++i) {
        for(int j = 0; j < C; ++j){
            if(real[i][j]!=my[i][j] && my[i][j]!='y'
									&& my[i][j]!='b')
                my[i][j] = real[i][j];
        }
    }
}

int PathNode::cell = 0;
int PathNode::row = 0;
string* PathNode::myMaze = NULL;
string* PathNode::realMaze = NULL;
vector<Point*> PathNode::branches;
int PathNode::counter = 0;

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
        PathNode::realMaze = maze;
		
		//Пока движемся к точке, игнорируем цикл
        if(PathNode::counter - 1 > 0){
            PathNode::counter--;
            cerr << PathNode::counter << endl;
        }
        else {
            curPoint = new Point(KR,KC);
            cur = new PathNode(curPoint);
            cur->goNext();
        }

        // Write an action using cout. DON'T FORGET THE "<< endl"
        // To debug: cerr << "Debug messages..." << endl;
        for (int i = 0; i < R; i++) {
            cerr << maze[i] <<endl;
        }
    }
}