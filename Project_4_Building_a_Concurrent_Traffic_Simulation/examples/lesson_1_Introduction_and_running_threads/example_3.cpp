#include <iostream>
#include <thread>


/*
Starting a second thread
In this section, we will start a second thread in addition to the main thread of our program.
To do this, we need to construct a thread object and pass it the function we want to be executed by the thread.
Once the thread enters the runnable state, the execution of the associated thread function may start at any point in time.

After the thread object has been constructed, the main thread will continue and execute the remaining instructions until it reaches the end and returns.
It is possible that by this point in time, the thread will also have finished. But if this is not the case, the main program will terminate and the resources of the associated process will be freed by the OS.
As the thread exists within the process, it can no longer access those resources and thus not finish its execution as intended.

To prevent this from happening and have the main program wait for the thread to finish the execution of the thread function,
we need to call join() on the thread object. This call will only return when the thread reaches the end of the thread function and block the main thread until then.

The code on the right shows how to use join() to ensure that main() waits for the thread t to finish its operations before returning.
It uses the function sleep_for(), which pauses the execution of the respective threads for a specified amount of time.
The idea is to simulate some work to be done in the respective threads of execution.

To compile this code with g++, you will need to use the -pthread flag. pthread adds support for multithreading with the pthreads library, and the option sets flags for both the preprocessor and linker:


g++ example_3.cpp -pthread

Not surprisingly, the main function finishes before the thread because the delay inserted into the thread function is much larger than in the main path of execution.
The call to join() at the end of the main function ensures that it will not prematurely return.
As an experiment, comment out t.join() and execute the program. What do you expect will happen?
*/
void threadFunction()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // simulate work
    std::cout << "Finished work in thread\n"; 
}

int main()
{
    // create thread
    std::thread t(threadFunction);

    // do something in main()
    std::this_thread::sleep_for(std::chrono::milliseconds(50)); // simulate work
    std::cout << "Finished work in main\n";

    // wait for thread to finish
    t.join();

    return 0;
}