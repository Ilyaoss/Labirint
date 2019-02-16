#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
using namespace std;

class Point {
private:
    int x;
    int y;
public:
    Point(int X = 0, int Y = 0) {
        x = X; y = Y;
    }

    Point(const Point &obj) {      
        x = obj.x;
        y = obj.y;
    }

	int getX() const {
		return x;
	}

	int getY() const {
		return y;
	}

	Point& operator= (const Point& p1) {
        //проверка на самоприсваивание
        if (this == &p1) {
            return *this;
        }
        x = p1.x;
		y = p1.y;
        return *this;
    }

    bool operator== (const Point& p2) {
        return x == p2.x && y == p2.y;
    }

	bool operator!= (const Point& p2) {
        return x != p2.x || y != p2.y;
    }

};

class PathNode {
private:
	//Координата точки
    Point position;
    //Откуда пришли
    PathNode* cameFrom;
    //Длина от начала пути
    int pathLengthFromStart;
    //Эвристическая длина
    int heuristicEstimatePathLength;
public:
	PathNode(Point p, int hEP = 0, PathNode* cF = NULL, int pLFS = 0) {
		position = p;
		cameFrom = cF;
		pathLengthFromStart = pLFS;
		heuristicEstimatePathLength = hEP;
	}
	
	static vector<string> findPath(const Point start, const Point goal, const string* maze) {
        // Результируеющий массив
        vector<string> result;
        // Массив ожидающих рассмотрения и пройденных точек
        vector<PathNode*> closedSet;
        vector<PathNode*> openSet; 
        
        cerr << "s " << start.getX() << " " << start.getY() << endl;
        cerr << "e " << goal.getX() << " " << goal.getY() << endl;
        
        // Начинаем искать
        PathNode* startNode = new PathNode(start, getHeuristicPathLength(start, goal));
        openSet.push_back(startNode);
		
        while (openSet.size() > 0) {
            // Берём точку ближайшую к цели
            sort(openSet.begin(),openSet.end());
            auto currentNode = openSet.front();
			
			cerr << "cur " << currentNode->position.getX() << " " << currentNode->position.getY() << endl;
			// Если совапли, то пути найден
            if (currentNode->position == goal) {
				vector<string> path = getPathForNode(currentNode);
				
				for (int i = 0; i < closedSet.size(); ++i) {
                    delete closedSet[i];
				}
				
				for (int i = 0; i < openSet.size(); ++i) {
					delete openSet[i];
				}
				
				openSet.clear();
				closedSet.clear();
				
                return path;
			}
			
            // Переносим точку из списка ожидающий в список пройденных
            openSet.erase(openSet.begin());
            closedSet.push_back(currentNode);
            // Ищем соседей
            vector<PathNode*> neighbourNodes = getNeighbours(currentNode, goal, maze);
		    
            for (int i = 0; i < neighbourNodes.size(); ++i) {
                PathNode* neighbourNode = neighbourNodes[i];
				
				// Если сосед уже есть в списке пройденных - пропускаем
                int SamePosition = count_if(closedSet.begin(),closedSet.end(),[neighbourNode](PathNode* closed) {
                    return (neighbourNode->position) == (closed->position);
                });
				
                if (SamePosition > 0) {
                    delete neighbourNode;
                    continue;
                }
				
				// Ищем в списке не пройденных
                auto openNode = find_if(openSet.begin(),openSet.end(),[neighbourNode](PathNode* opened) {
                    return (neighbourNode->position) == (opened->position);
                });
				
                // Если не нашли - то добавляем
                if (openNode == openSet.end())
                    openSet.push_back(neighbourNode);
				
				// Если же сосед в списке на рассмотрение — проверяем,
				// Если X.pathLengthFromStart + расстояние от X до Y < Y.pathLengthFromStart, значит мы пришли в точку Y более коротким путем, 
				// Заменяем Y.pathLengthFromStart на X.pathLengthFromStart + расстояние от X до Y, а точку, из которой пришли в Y на X.
                else {
                    if (openNode[0]->pathLengthFromStart > neighbourNode->pathLengthFromStart) {
                        openNode[0]->cameFrom = currentNode;
                        openNode[0]->pathLengthFromStart = neighbourNode->pathLengthFromStart;
                    }
					
                    delete neighbourNode;
                }
            }
			
			neighbourNodes.clear();
        }
        
        for (int i = 0; i < closedSet.size(); ++i) {
			delete closedSet[i];
		}
		
		for (int i = 0; i < openSet.size(); ++i) {
			delete openSet[i];
		}
		
		openSet.clear();
		closedSet.clear();
		openSet.clear();
		closedSet.clear();
        
		// Если список точек на рассмотрение пуст, а до цели мы так и не дошли — значит маршрут не существует
        return result;
    }

    // Возвращаем комманды движения к точке
    static vector<string> getPathForNode(PathNode* pathNode) {
	    vector<string> path;
		vector<PathNode*> result;
		PathNode* currentNode = pathNode;
		
		while (currentNode != NULL) {	
			result.push_back(currentNode);
			currentNode = currentNode->cameFrom;
		}
		
		reverse(result.begin(), result.end());
		int size = result.size();
		
		for (int i = 0; i < size - 1 ; ++i) {
		    if (result[i]->position.getX() < result[i+1]->position.getX()) path.push_back("DOWN");
		    else if (result[i]->position.getX() > result[i+1]->position.getX()) path.push_back("UP");
		    else if (result[i]->position.getY() < result[i+1]->position.getY()) path.push_back("RIGHT");
		    else path.push_back("LEFT");
		}
		
		result.clear();
		delete currentNode;
		
		return path;
	}

    static vector<PathNode*> getNeighbours(PathNode* pathNode, const Point goal, const string* maze) {
    	vector<PathNode*> result;

		int x = pathNode->position.getX();
		int y = pathNode->position.getY();
		vector<Point> points = {Point(x, y + 1), Point(x, y - 1), Point(x + 1, y), Point(x - 1, y)};
		
    	for (int i = 0; i < points.size(); ++i) {
			x = points[i].getX();
			y = points[i].getY();
            
			// Проверяем, что по клетке можно ходить.
			if ((maze[x][y] == '.') || (maze[x][y] == 'C') || (maze[x][y] == 'T')) {
				Point p(x,y);
				// Заполняем данные для точки маршрута.
				PathNode* neighbourNode = new PathNode(p, 
											getHeuristicPathLength(p, goal), 
											pathNode, 
											pathNode->pathLengthFromStart + pathNode->getDistanceBetweenNeighbours());
				result.push_back(neighbourNode);
			}
    	}
        
    	return result;
    }
	
	int estimateFullPathLength() const {
        return pathLengthFromStart + heuristicEstimatePathLength;
    }

	int getDistanceBetweenNeighbours() const {
		return 1;
	}

	static int getHeuristicPathLength(const Point from, const Point to) {
        return abs(from.getX() - to.getX()) + abs(from.getY() - to.getY());
    }

    bool operator< (PathNode* a) {
        return (this->estimateFullPathLength() < a->estimateFullPathLength());
    }

};

class Path {
private:
    //Координата точки
    Point position;
    //Соседние клетки
    vector<Point> neighbours;
    //Стек точек развилок(более 1 варианта пути)
    static vector<Point> branches;
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
    //Время на обратный путь
    static int timeBack;
    //Контрольная комната
    static Point controlPoint;
public:
    Path(Point p) {
        position = p;
        neighbours = getNeighbours(p, myMaze);
    }

    void goNext() {
		
        for (int i = 0; i < row; ++i) {
            cerr << realMaze[i] << endl;
        }
        
        int size = neighbours.size();
        myMaze[position.getX()][position.getY()] = 'y';
        
        // Если нашли С
        if (controlPoint != Point(-1,-1)) {
            Point startPoint = findT();
            vector<string> pathToT = PathNode::findPath(controlPoint,startPoint,realMaze);
            
			// И обратный путь меньше обратного отсчёта, то идём к С и Т
            if (pathToT.size() <= timeBack && pathToT.size() > 0) {
                vector<string> pathToC = PathNode::findPath(position,controlPoint,realMaze);
                counter = pathToC.size() + pathToT.size();
                
				for (int i = 0; i < pathToC.size(); ++i) {
				    cerr << pathToC[i] << endl;
                    cout << pathToC[i] << endl;
                }
                
				for (int i = 0; i < pathToT.size(); ++i) {
				    cerr << pathToT[i] << endl;
                    cout << pathToT[i] << endl;
                }
                
                branches.clear();
				//delete myMaze;
				//delete realMaze;
            }
			pathToT.clear();
        }
        else controlPoint = findC();
        
        //Если больше 1 пути, то помечаем развилку(b-branch)
        if (size > 1) {
            myMaze[position.getX()][position.getY()] = 'b';
            branches.push_back(position);
        }
		//Если нет соседей, возращаемся к последнему разветвлению
        else if (size < 1) {
            while (branches.size() > 0) {
                Point branch = branches.back();
                branches.pop_back();
                
				//Проверяем осталось ли там разветвление
                if (isBrunch(branch)) { 
                    //Вычисляем путь до последнего разветвления
                    vector<string> pathToLastBranch = PathNode::findPath(position,branch,realMaze);
                    //Выставляем счётсчик и движемся
                    counter = pathToLastBranch.size();
                    
					for (int i = 0; i < pathToLastBranch.size(); ++i) {
                        cout << pathToLastBranch[i] << endl;
                    }
					
                    return;
                }
                //Если не осталось изменяем метку
                else myMaze[branch.getX()][branch.getY()] = 'y';
            }
        }
        
        for(int i = 0; i < size; ++i) {
		    int nX = neighbours[i].getX();
            int nY = neighbours[i].getY();
			
            if (myMaze[nX][nY] != 'y') {
                if (position.getX() > nX) cout << "UP"<< endl;
                else if (position.getX() < nX) cout << "DOWN"<< endl;
                else if (position.getY() > nY) cout << "LEFT"<< endl;
                else cout << "RIGHT" << endl;
                break;
            }
        }
    }

    //Получаем соседние точки
    vector<Point> getNeighbours(const Point cur, const string* M) const {
		vector<Point> neighbours;
		int x = cur.getX();
		int y = cur.getY();
		
		vector<Point> points = {Point(x, y + 1), Point(x, y - 1), Point(x + 1, y), Point(x - 1, y)};
		
		for (int i = 0; i < points.size(); ++i) {
		    x = points[i].getX();
		    y = points[i].getY();
			if (M[x][y] == '.') {
			    neighbours.push_back(points[i]);
		    } 
		}

		return neighbours;
	}

    Point findC() const {
    	for (int i = 0; i < row; ++i) {
            for (int j = 0; j < cell; ++j) {
                if (realMaze[i][j] == 'C')
                    return Point(i,j);
            }
        }
    	return Point(-1,-1);
    }

    Point findT() const {
    	for (int i = 0; i < row; ++i) {
            for (int j = 0; j < cell; ++j) {
                if (realMaze[i][j] == 'T')
                    return Point(i,j);
            }
        }
    	return Point(-1,-1);
    }

    bool isBrunch(const Point p) const {
        int X = p.getX();
        int Y = p.getY();
        
        if (myMaze[X][Y+1] == '.' || myMaze[X][Y-1] == '.'
            || myMaze[X+1][Y] == '.' || myMaze[X-1][Y] == '.')
            return true;
        return false;
    }

	static void setMyMaze(string* m) {
		myMaze = m;
	}

    static string* getMyMaze() {
		return myMaze;
	}

	static void setRealMaze(string* m) {
		realMaze = m;
	}

	static void setTimer(int t) {
		timeBack = t;
	}

	static void setRow(int r) {
		row = r;
	}

	static void setCell(int c) {
		cell = c;
	}

	static int getCounter() {
		return counter;
	}

	static void decrementCounter() {
		--counter;
	}

};

void compareMaze(const string* real, string* my, const int R, const int C) {
    for (int i = 0; i < R; ++i) {
        for (int j = 0; j < C; ++j) {
            if (real[i][j] != my[i][j] && my[i][j] != 'y'
									&& my[i][j] != 'b')
                my[i][j] = real[i][j];
        }
    }
}

int Path::cell = 0;
int Path::row = 0;
string* Path::myMaze = NULL;
string* Path::realMaze = NULL;
Point Path::controlPoint(-1,-1);
vector<Point> Path::branches;
int Path::counter = 0;
int Path::timeBack = 0;

int main()
{
    int R; // number of rows.
    int C; // number of columns.
    int A; // number of rounds between the time the alarm countdown is activated and the time the alarm goes off.
    cin >> R >> C >> A; cin.ignore();
    
	Path::setRow(R);
	Path::setCell(C);
    Path::setTimer(A);
	
    string* myMaze = new string[R];
	string* maze = new string[R];
	
    for (int i = 0; i < R; ++i) {
        myMaze[i].resize(C,'?');
    }

	Path::setMyMaze(myMaze);

	// game loop
    while (1) {
        int KR; // row where Kirk is located.
        int KC; // column where Kirk is located.
        cin >> KR >> KC; cin.ignore();
        myMaze =  Path::getMyMaze();
        
		for (int i = 0; i < R; ++i) {
            string row; // C of the characters in '#.TC?' (i.e. one line of the ASCII maze).
            cin >> row; cin.ignore();
            maze[i] = row;
        }
		
        compareMaze(maze, myMaze, R, C);
        Path::setMyMaze(myMaze);
        Path::setRealMaze(maze);
		
		//Пока движемся к точке, игнорируем цикл
        if (Path::getCounter() - 1 > 0){
            Path::decrementCounter();
        } else {
            Point curPoint(KR,KC);
            Path curPath(curPoint);
            curPath.goNext();
        }
    }
    delete maze;
    delete myMaze;
}