#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <memory>

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

    Point& operator= (const Point& right) {
        //Проверка на самоприсваивание
        if (this == &right) {
            return *this;
        }
        x = right.x;
        y = right.y;
        return *this;
    }

    bool operator== (const Point& right) {
        return x == right.x && y == right.y;
    }

    bool operator!= (const Point& right) {
        return x != right.x || y != right.y;
    }

};

class PathNode {
private:
    //Координата точки
    Point position;
    //Откуда пришли
    shared_ptr<PathNode> cameFrom;
    //Длина от начала пути
    int pathLengthFromStart;
    //Эвристическая длина
    int heuristicEstimatePathLength;
public:
    PathNode(Point _position, int _heuristicEstimatePathLength = 0, 
            shared_ptr<PathNode> _cameFrom = nullptr, int _pathLengthFromStart = 0) {
        if (_position.getX() < 0 || _position.getY() < 0) {
            throw out_of_range{"Position out of range"};
        }
        if (_heuristicEstimatePathLength < 0) {
            throw length_error{"Heuristic Estimate Path Length must be nonnegative"};
        }
        if (_pathLengthFromStart < 0) {
            throw length_error{"Path Length From Start must be nonnegative"};
        }
        position = _position;
        cameFrom = _cameFrom;
        pathLengthFromStart = _pathLengthFromStart;
        heuristicEstimatePathLength = _heuristicEstimatePathLength;
    }
    
    static vector<string> findPath(const Point start, const Point goal, const shared_ptr<string[]> maze) {
        vector<string> result;
        // Массив ожидающих рассмотрения и пройденных точек
        vector<shared_ptr<PathNode>> closedSet;
        vector<shared_ptr<PathNode>> openSet; 
        
        cerr << "s " << start.getX() << " " << start.getY() << endl;
        cerr << "e " << goal.getX() << " " << goal.getY() << endl;
        
        // Начинаем искать
        auto startNode = make_shared<PathNode>(start, getHeuristicPathLength(start, goal));
        openSet.push_back(startNode);
        
        while (openSet.size() > 0) {
            // Берём точку ближайшую к цели
            sort(openSet.begin(),openSet.end());
            auto currentNode = openSet.front();
            
            // Если совапли, то путь найден
            if (currentNode->position == goal) {
                vector<string> path = getPathForNode(currentNode);
                
                openSet.clear();
                closedSet.clear();
                
                return path;
            }
            
            openSet.erase(openSet.begin());
            closedSet.push_back(currentNode);
            
            auto neighbourNodes = getNeighbours(currentNode, goal, maze);
            
            for (int i = 0; i < neighbourNodes.size(); ++i) {
                auto neighbourNode = neighbourNodes[i];
                
                // Если сосед уже есть в списке пройденных - пропускаем
                int SamePosition = count_if(closedSet.begin(),closedSet.end(),[neighbourNode](shared_ptr<PathNode> closed) {
                    return (neighbourNode->position) == (closed->position);
                });
                
                if (SamePosition > 0) {
                    continue;
                }
                
                // Ищем в списке не пройденных
                auto openNode = find_if(openSet.begin(),openSet.end(),[neighbourNode](shared_ptr<PathNode> opened) {
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
                }
            }
            
            neighbourNodes.clear();
        }
        
        closedSet.clear();
        
        // Если список точек на рассмотрение пуст, а до цели мы так и не дошли
        // Значит маршрут не существует
        return result;
    }

    // Возвращаем комманды движения к точке
    static vector<string> getPathForNode(shared_ptr<PathNode> pathNode) {
        vector<string> path;
        vector<shared_ptr<PathNode>> result;
        auto currentNode = pathNode;
        
        while (currentNode != nullptr) {    
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
        return path;
    }

    static vector<shared_ptr<PathNode>> getNeighbours(shared_ptr<PathNode> pathNode, const Point goal, const shared_ptr<string[]> maze) {
        vector<shared_ptr<PathNode>> result;

        int x = pathNode->position.getX();
        int y = pathNode->position.getY();
        vector<Point> points = {Point(x, y + 1), Point(x, y - 1), Point(x + 1, y), Point(x - 1, y)};
        
        for (int i = 0; i < points.size(); ++i) {
            x = points[i].getX();
            y = points[i].getY();
            
            // Проверяем, что по клетке можно ходить.
            if ((maze[x][y] == '.') || (maze[x][y] == 'C') || (maze[x][y] == 'T')) {
                Point neighbourPosition(x,y);
                // Заполняем данные для точки маршрута.
                shared_ptr<PathNode> neighbourNode = make_shared<PathNode>(neighbourPosition, 
                                            getHeuristicPathLength(neighbourPosition, goal), 
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

    bool operator< (const PathNode* a) {
        return (this->estimateFullPathLength() < a->estimateFullPathLength());
    }

};

class Path {
private:
    //Соседние клетки
    vector<Point> neighbours;
    //Стек точек развилок(более 1 варианта пути)
    vector<Point> branches;
    //Мой вид лабиринта
    shared_ptr<string[]> myMaze;
    //Реальный вид лабиринта
    shared_ptr<string[]> realMaze;
    //Размер лабиринта
    int row;
    int cell;
    //Счётчик, для приостановки бесконечного цикла в main
    //Приостанавливаем, чтобы предотвратить создание новый точек, пока движемся
    int counter;
    //Время на обратный путь
    int alarm;
    //Контрольная комната
    Point controlPoint;
public:
    Path(int Row = 0, int Cell = 0, int Alarm = 0) {
        realMaze = nullptr;
        myMaze = nullptr;
        row = Row;
        cell = Cell;
        alarm = Alarm;
        counter = 0;
        controlPoint = Point(-1,-1);
    }

    void goNext(Point position) noexcept(false) {
        try {
            if(position.getX() < 0 || position.getX() >= row 
                || position.getY() < 0 || position.getY() >= cell) {
                throw out_of_range{"Position is out of maze range"};
            }
            
            int size = neighbours.size();
            
            for(int i = 0; i < size; ++i) {
                neighbours.pop_back();
            }
            
            neighbours.clear();
            neighbours = getNeighbours(position, myMaze);
            size = neighbours.size();
            
            for (int i = 0; i < row; ++i) {
                cerr << realMaze[i] << endl;
            }
            
            myMaze[position.getX()][position.getY()] = 'y';
            
            // Если нашли С
            if (controlPoint != Point(-1,-1)) {
                Point startPoint = findT();
                vector<string> pathToT = PathNode::findPath(controlPoint,startPoint,realMaze);
                
                // И обратный путь меньше обратного отсчёта, то идём к С и Т
                if (pathToT.size() <= alarm && pathToT.size() > 0) {
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
                int x = neighbours[i].getX();
                int y = neighbours[i].getY();
                
                if (myMaze[x][y] != 'y') {
                    if (position.getX() > x) cout << "UP"<< endl;
                    else if (position.getX() < x) cout << "DOWN"<< endl;
                    else if (position.getY() > y) cout << "LEFT"<< endl;
                    else cout << "RIGHT" << endl;
                    break;
                }
            }
        }
        catch (...) {
            throw;
        }
    }

    vector<Point> getNeighbours(const Point current, const shared_ptr<string[]> maze) const noexcept(false) {
        vector<Point> neighbours;
        int x = current.getX();
        int y = current.getY();
        
        if(x < 0 || x >= row || y < 0 || y >= cell) {
            throw out_of_range{"Position is out of maze range"};
        }
        
        vector<Point> points = {Point(x, y + 1), Point(x, y - 1), Point(x + 1, y), Point(x - 1, y)};
        
        for (int i = 0; i < points.size(); ++i) {
            x = points[i].getX();
            y = points[i].getY();
            
            if(x < 0 || x >= row || y < 0 || y >= cell) {
                continue;
            }
        
            if (maze[x][y] == '.') {
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

    bool isBrunch(const Point point) const noexcept(false) {
        int x = point.getX();
        int y = point.getY();
        
        if(x < 0 || x >= row || y < 0 || y >= cell) {
            throw out_of_range{"Position is out of maze range"};
        }
            
        if (myMaze[x][y+1] == '.' || myMaze[x][y-1] == '.'
            || myMaze[x+1][y] == '.' || myMaze[x-1][y] == '.')
            return true;
        return false;
    }

    void setMyMaze(shared_ptr<string[]> maze) {
        myMaze = maze;
    }

    shared_ptr<string[]> getMyMaze() {
        return myMaze;
    }

    void setRealMaze(shared_ptr<string[]> maze) {
        realMaze = maze;
    }

    void setAlarm(int _alarm) noexcept(false) {
        if (_alarm < 0) {
            throw length_error{"Alarm must be nonnegative"};
        }
        alarm = _alarm;
    }

    void setRow(int _row) noexcept(false) {
        if (_row < 0) {
            throw length_error{"Row must be nonnegative"};
        }
        row = _row;
    }

    void setCell(int _cell) noexcept(false) {
        if (_cell < 0) {
            throw length_error{"Cell must be nonnegative"};
        }
        cell = _cell;
    }

    int getCounter() {
        return counter;
    }

    void decrementCounter() {
        --counter;
    }

};

void compareMaze(const shared_ptr<string[]> real, shared_ptr<string[]> my, const int Row, const int Cell) {
    for (int i = 0; i < Row; ++i) {
        for (int j = 0; j < Cell; ++j) {
            if (real[i][j] != my[i][j] && my[i][j] != 'y'
                                    && my[i][j] != 'b')
                my[i][j] = real[i][j];
        }
    }
}

int main()
{
    try {
        int Row; // number of rows.
        int Cell; // number of columns.
        int Alarm; // number of rounds between the time the alarm countdown is activated and the time the alarm goes off.
        cin >> Row >> Cell >> Alarm; cin.ignore();
        
        Path path(Row, Cell, Alarm);
        
        shared_ptr<string[]> myMaze(new string[Row]);
        shared_ptr<string[]> maze(new string[Row]);
        
        for (int i = 0; i < Row; ++i) {
            myMaze[i].resize(Cell,'?');
        }

        path.setMyMaze(myMaze);

        // game loop
        while (1) {
            int KirkRow; // row where Kirk is located.
            int KirkCell; // column where Kirk is located.
            cin >> KirkRow >> KirkCell; cin.ignore();
            myMaze =  path.getMyMaze();
            
            for (int i = 0; i < Row; ++i) {
                string row; // C of the characters in '#.TC?' (i.e. one line of the ASCII maze).
                cin >> row; cin.ignore();
                maze[i] = row;
            }
            
            compareMaze(maze, myMaze, Row, Cell);
            path.setMyMaze(myMaze);
            path.setRealMaze(maze);
            
            //Пока движемся к точке, игнорируем цикл
            if (path.getCounter() - 1 > 0){
                path.decrementCounter();
            } else {
                Point curPoint(KirkRow,KirkCell);
                path.goNext(curPoint);
            }
        }
    }
    catch(...) {
        cerr << "Ошибка!" << endl;
    }
    
    return 0;
}