#include <iostream>
#include <thread>
#include <string>

/*
Starting a Thread with Variadic Templates and Member Functions
Passing Arguments using a Variadic Template
In the previous section, we have seen that one way to pass arguments in to the thread function is to package them in a class using the function call operator.
Even though this worked well, it would be very cumbersome to write a special class every time we need to pass data to a thread.
We can also use a Lambda that captures the arguments and then calls the function.
But there is a simpler way: The thread constructor may be called with a function and all its arguments.
That is possible because the thread constructor is a variadic template that takes multiple arguments.

Before C++11, classes and functions could only accept a fixed number of arguments,
which had to be specified during the first declaration. With variadic templates it is possible to include any number of arguments of any type.
*/
void printID(int id)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::cout << "ID = " << id << std::endl;
    
}

void printIDAndName(int id, std::string name)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "ID = " << id << ", name = " << name << std::endl;
}

int main()
{
    int id = 0; // Define an integer variable

    // starting threads using variadic templates
    std::thread t1(printID, id);
    std::thread t2(printIDAndName, ++id, "MyString");
    std::thread t3(printIDAndName, ++id); // this procudes a compiler error

    // wait for threads before returning
    t1.join();
    t2.join();
    //t3.join();


    return 0;
}