
/**
 * @file example_mutex.cpp
 * @author Dr. -Ing. Ahmad Kamal Nasir (dringakn@gmail.com)
 * @brief C++ Mutex example
 * @version 0.1
 * @date 2023-03-20
 * @note g++ example_mutex.cpp -lpthread -o mutex --std=c++11
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <bits/stdc++.h>

int main(int argc, char const *argv[])
{
    std::mutex m;

    auto print = [&m](int id)
    {
        if (m.try_lock())
        {
            std::cout << "Printing from thread-" << id << std::endl;
            m.unlock();
        }
    };

    std::vector<std::thread> threads;
    for (int i = 0; i < 10; ++i)
        threads.emplace_back(print, i);

    for (int i = 0; i < 10; ++i)
        threads[i].join();

    return 0;
}
