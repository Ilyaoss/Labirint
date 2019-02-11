// ReverseList.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//

#include "pch.h"
#include <iostream>
class ListNode {
public:
	int _num;
	ListNode* _next;
	ListNode(int num, ListNode* next = NULL) {
		_num = num;
		_next = next;
	}
};
ListNode* reverse(ListNode* current, ListNode* previous = NULL)
{
	if (current == NULL) {
		return previous;
	}
	ListNode* next = current->_next;
	current->_next = previous;
	return reverse(next, current);
}

int main()
{
	const int N = 20;
	ListNode* start = new ListNode(0);
	ListNode* current = start;
	for (int i = 1; i < N; ++i) {
		ListNode* next = new ListNode(i);
		current->_next = next;
		current = next;
	}
	current = start;
	while (current != NULL) {
		std::cout << current->_num << " !\n";
		current = current->_next;
	}
	std::cout << std::endl;
	current = reverse(start);
	while (current != NULL) {
		std::cout << current->_num << " !\n";
		current = current->_next;
	}
	return 0;
}

// Запуск программы: CTRL+F5 или меню "Отладка" > "Запуск без отладки"
// Отладка программы: F5 или меню "Отладка" > "Запустить отладку"

// Советы по началу работы 
//   1. В окне обозревателя решений можно добавлять файлы и управлять ими.
//   2. В окне Team Explorer можно подключиться к системе управления версиями.
//   3. В окне "Выходные данные" можно просматривать выходные данные сборки и другие сообщения.
//   4. В окне "Список ошибок" можно просматривать ошибки.
//   5. Последовательно выберите пункты меню "Проект" > "Добавить новый элемент", чтобы создать файлы кода, или "Проект" > "Добавить существующий элемент", чтобы добавить в проект существующие файлы кода.
//   6. Чтобы снова открыть этот проект позже, выберите пункты меню "Файл" > "Открыть" > "Проект" и выберите SLN-файл.
