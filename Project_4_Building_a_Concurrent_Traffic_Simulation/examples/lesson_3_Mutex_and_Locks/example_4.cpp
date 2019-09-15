#include <iostream>
#include <thread>
#include <mutex>
 
 /*
 Deadlock 2
A second type of deadlock is a state in which two or more threads are blocked because each thread waits for the resource of the other thread to be released before releasing its resource.
The result of the deadlock is a complete standstill.
The thread and therefore usually the whole program is blocked forever. 
*/

std::mutex mutex1, mutex2;
 
void ThreadA()
{
    // Creates deadlock problem
    mutex2.lock();
    std::cout << "Thread A" << std::endl;
    mutex1.lock();
    mutex2.unlock();
    mutex1.unlock();
}
 
void ThreadB()
{
    // Creates deadlock problem
    mutex1.lock();
    std::cout << "Thread B" << std::endl;
    mutex2.lock();
    mutex1.unlock();
    mutex2.unlock();
}
 
void ExecuteThreads()
{
    std::thread t1( ThreadA );
    std::thread t2( ThreadB );
 
    t1.join();
    t2.join();
 
    std::cout << "Finished" << std::endl;
}
 
int main()
{
    ExecuteThreads();
 
    return 0;
}