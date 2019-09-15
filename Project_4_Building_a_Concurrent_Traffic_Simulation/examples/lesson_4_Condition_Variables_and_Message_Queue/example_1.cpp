#include <iostream>
#include <thread>
#include <vector>
#include <future>
#include <mutex>
#include <algorithm>

/*
Writing a vehicle counter
Note that the program in the example above did not terminate - even though no new Vehicles are added to the queue,
the infinite while-loop will not exit.

One possible solution to this problem would be to integrate a vehicle counter into the WaitingVehicles class,
that is incremented each time a Vehicle object is added and decremented when it is removed.
The while-loop could then be terminated as soon as the counter reaches zero.
Please go ahead and implement this functionality - but remember to protect the counter as it will also be accessed by several threads at once.
Also, it will be a good idea to introduce a small delay between spawning threads and collecting results.
Otherwise, the queue will be empty by default and the program will terminate prematurely.
At the end of main(), please also print the number of remaining Vehicle objects in the vecto
*/
class Vehicle
{
public:
    Vehicle(int id) : _id(id) {}
    int getID() { return _id; }

private:
    int _id;
};

class WaitingVehicles
{
public:
    WaitingVehicles() {}

    void printIDs()
    {
        std::lock_guard<std::mutex> myLock(_mutex); // lock is released when myLock goes out of scope
        for(auto &v : _vehicles)
            std::cout << "   Vehicle #" << v.getID() << " is now waiting in the queue" << std::endl;
        
    }

    void pushBack(Vehicle &&v)
    {
        // perform vector modification under the lock
        std::lock_guard<std::mutex> uLock(_mutex);
        std::cout << "   Vehicle #" << v.getID() << " will be added to the queue" << std::endl; 
        _vehicles.emplace_back(std::move(v));

        // simulate some work
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

private:
    std::vector<Vehicle> _vehicles; // list of all vehicles waiting to enter this intersection
    std::mutex _mutex;
};

int main()
{
    // create monitor object as a shared pointer to enable access by multiple threads
    std::shared_ptr<WaitingVehicles> queue(new WaitingVehicles);

    std::cout << "Spawning threads..." << std::endl;
    std::vector<std::future<void>> futures;
    for (int i = 0; i < 10; ++i)
    {
        // create a new Vehicle instance and move it into the queue
        Vehicle v(i);
        futures.emplace_back(std::async(std::launch::async, &WaitingVehicles::pushBack, queue, std::move(v)));
    }

    std::for_each(futures.begin(), futures.end(), [](std::future<void> &ftr) {
        ftr.wait();
    });

    std::cout << "Collecting results..." << std::endl;
    queue->printIDs();

    return 0;
}