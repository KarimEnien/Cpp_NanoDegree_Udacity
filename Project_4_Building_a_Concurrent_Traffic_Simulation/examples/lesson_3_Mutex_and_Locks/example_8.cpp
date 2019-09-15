#include <iostream>
#include <thread>
#include <mutex>

/*
In the following deadlock-free code, std::lock is used to ensure that the mutexes are always locked in the same order,
regardless of the order of the arguments. Note that std::adopt_lock option allows us to use std::lock_guard on an already locked mutex.

As a rule of thumb, programmers should try to avoid using several mutexes at once.
Practice shows that this can be achieved in the majority of cases. For the remaining cases though, using std::lock is a safe way to avoid a deadlock situation.
*/ 
std::mutex mutex1, mutex2;
 
void ThreadA()
{
    // Ensure that locks are always executed in the same order
    std::lock(mutex1, mutex2);
    std::lock_guard<std::mutex> lock2(mutex2, std::adopt_lock);
    std::cout << "Thread A" << std::endl;
    std::lock_guard<std::mutex> lock1(mutex1, std::adopt_lock);
    
}
 
void ThreadB()
{
    std::lock(mutex1, mutex2);
    std::lock_guard<std::mutex> lock1(mutex1, std::adopt_lock);
    std::cout << "Thread B" << std::endl;
    std::lock_guard<std::mutex> lock2(mutex2, std::adopt_lock);
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