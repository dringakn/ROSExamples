
/**
 * @file example_atomic.cpp
 * @author Dr. -Ing. Ahmad Kamal Nasir (dringakn@gmail.com)
 * @brief C++ atomic example
 * @version 0.1
 * @date 2023-03-20
 * @note g++ example_atomic.cpp -lpthread -o atomic --std=c++17
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <bits/stdc++.h>

int main(int argc, char const *argv[])
{
    std::atomic<int> counter = 0; // For c++11 or c++14: counter(0)

    auto increment = [&counter]()
    {
        for (int i = 0; i < 50000; i++)
            counter++; // Protect read-modify-write
    };

    std::vector<std::thread> threads;
    for (int i = 0; i < 10; ++i)
        threads.emplace_back(increment);

    for (int i = 0; i < 10; ++i)
        threads[i].join();

    std::cout << "Counter: " << counter << std::endl;

    return 0;
}
