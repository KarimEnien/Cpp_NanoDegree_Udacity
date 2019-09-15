#include <iostream>
#include <thread>


/*

Quiz: Starting your own threads
In the code on the right, you will find a thread function called threadFunctionEven, which is passed to a thread t. In this example, the thread is immediately detached after creation. To ensure main does not quit before the thread is finished with its work, there is a sleep_for call at the end of main.

Please create a new function called threadFunctionOdd that outputs the string "Odd threadn". Then write a for-loop that starts 6 threads and immediately detaches them. Based on wether the increment variable is even or odd, you should pass the respective function to the thread.

Run the program several times and look the console output. What do you observe? As a second experiment, comment out the sleep_for function in the main thread. What happens to the detached threads in this case?

Solution:
The order in which even and odd threads are executed changes. Also, some threads are executed after the main function reaches its end. When sleep_for is removed, threads will not finish before the program terminates.

*/
void threadFunctionEven()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // simulate work
    std::cout << "Even thread\n";
}

/* Student Task START */
void threadFunctionOdd()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // simulate work
    std::cout << "Odd thread\n";
}
/* Student Task END */

int main()
{
    /* Student Task START */
    for (int i = 0; i < 6; ++i)
    {
        if (i % 2 == 0)
        {
            std::thread t(threadFunctionEven);
            t.detach();
        }
        else
        {
            std::thread t(threadFunctionOdd);
            t.detach();
        }
    }
    /* Student Task END */

    // ensure that main does not return before the threads are finished
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // simulate work

    std::cout << "End of main is reached" << std::endl;
    return 0;
}
