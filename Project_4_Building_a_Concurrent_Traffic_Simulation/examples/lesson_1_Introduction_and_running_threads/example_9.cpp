#include <iostream>

/*
 Lambda is a function object (a "functor"), so it has a type and can be stored and passed around.
Its result object is called a "closure", which can be called using the operator () as we will see shortly.

A lambda formally consists of three parts: a capture list [] , a parameter list ()
and a main part {}, which contains the code to be executed when the Lambda is called. Note that in principal all parts could be empty.

The capture list []: By default, variables outside of the enclosing {} around the main part of the Lambda can not be accessed.
By adding a variable to the capture list however, it becomes available within the Lambda either as a copy or as a reference.
The captured variables become a part of the Lambda.

By default, variables in the capture block can not be modified within the Lambda.
Using the keyword "mutable" allows to modify the parameters captured by copy,
and to call their non-const member functions within the body of the Lambda.
The following code examples show several ways of making the external variable "id" accessible within a Lambda.
*/
int main()
{
    // create lambdas
    int id = 0; // Define an integer variable

    //auto f0 = []() { std::cout << "ID = " << id << std::endl; }; // Error: 'id' cannot be accessed

    id++;
    auto f1 = [id]() { std::cout << "ID = " << id << std::endl; }; // OK, 'id' is captured by value

    id++;
    auto f2 = [&id]() { std::cout << "ID = " << id << std::endl; }; // OK, 'id' is captured by reference

    //auto f3 = [id]() { std::cout << "ID = " << ++id << std::endl; }; // Error, 'id' may not be modified

    auto f4 = [id]() mutable { std::cout << "ID = " << ++id << std::endl; }; // OK, 'id' may be modified

    // execute lambdas
    f1();
    f2();
    f4();

    return 0;
}