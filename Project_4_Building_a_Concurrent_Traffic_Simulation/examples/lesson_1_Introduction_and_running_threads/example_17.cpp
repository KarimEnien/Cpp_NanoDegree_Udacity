#include <iostream>
#include <thread>
#include <chrono>
#include <random>
#include <vector>

/*
A First Concurrency Bug
Let us adjust the program code from the previous example and use a Lambda instead of the function printHello().
Also, we will pass the loop counter i into the Lambda to enforce an individual wait time for each thread.
The idea is to prevent the interleaving of text on the command line which we saw in the previous example.
*/
int main()
{
    // create threads
    std::vector<std::thread> threads;
    for (size_t i = 0; i < 10; ++i)
    {
        // create new thread from a Lambda
        threads.emplace_back([i]() { //In order to ensure the correct view on the counter variable i, pass it to the Lambda function by value and not by reference

            // wait for certain amount of time
            std::this_thread::sleep_for(std::chrono::milliseconds(10 * i));

            // perform work
            std::cout << "Hello from Worker thread #" << i << std::endl;
        });
    }

    // do something in main()
    std::cout << "Hello from Main thread" << std::endl;

    // call join on all thread objects using a range-based loop
    for (auto &t : threads)
        t.join();

    return 0;
}
