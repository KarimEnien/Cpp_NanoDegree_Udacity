#include <iostream>
#include <thread>
#include <mutex>
 
/*
Avoiding deadlocks with std::lock()
In most cases, your code should only hold one lock on a mutex at a time.
Occasionally you can nest your locks, for example by calling a subsystem that protects its internal data with a mutex while holding a lock on another mutex,
but it is generally better to avoid locks on multiple mutexes at the same time, if possible. Sometimes, however, it is necessary to hold a lock on more than one mutex because you need to perform an operation on two different data elements, each protected by its own mutex.

In the last section, we have seen that using several mutexes at once can lead to a deadlock,
if the order of locking them is not carefully managed. To avoid this problem, the system must be told that both mutexes should be locked at the same time,
so that one of the threads takes over both locks and blocking is avoided. That's what the std::lock() function is for - you provide a set of lock_guard or unique_lock objects and the system ensures that they are all locked when the function returns.

In the following example, which is a version of the code we saw in the last section were std::mutex has been replaced with std::lock_guard.

Note that when executing this code, it still produces a deadlock, despite the use of std::lock_guard.
*/ 
std::mutex mutex1, mutex2;
 
void ThreadA()
{
    // Creates deadlock problem
    std::lock_guard<std::mutex> lock2(mutex2);
    std::cout << "Thread A" << std::endl;
    std::lock_guard<std::mutex> lock1(mutex1);
    
}
 
void ThreadB()
{
    // Creates deadlock problem
    std::lock_guard<std::mutex> lock1(mutex1);
    std::cout << "Thread B" << std::endl;
    std::lock_guard<std::mutex> lock2(mutex2);
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