#include <bits/stdc++.h>
#include <queue>

/*
Properties:
    Priority queues are a type of container adapters, specifically designed such that the first element of the queue is the greatest of all elements in the queue
    and elements are in nonincreasing order (hence we can see that each element of the queue has a priority {fixed order}).
*/

using namespace std;

void showpq(priority_queue<int, vector<int>, greater<int>> gq)
{
    priority_queue<int, vector<int>, greater<int>> g = gq;
    while (!g.empty())
    {
        cout << '\t' << g.top();
        g.pop();
    }
    cout << '\n';
}

int main(int argc, char const *argv[])
{
    priority_queue<int, vector<int>, greater<int>> gquiz;
    gquiz.push(10);
    gquiz.push(30);
    gquiz.push(20);
    gquiz.push(5);
    gquiz.push(1);
    cout << "The priority queue gquiz is : ";
    showpq(gquiz);

    cout << "\ngquiz.size() : " << gquiz.size();
    cout << "\ngquiz.top() : " << gquiz.top();

    cout << "\ngquiz.pop() : ";
    gquiz.pop();

    showpq(gquiz);

    return 0;
}
