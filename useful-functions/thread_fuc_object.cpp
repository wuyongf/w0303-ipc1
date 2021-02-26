//function object

#if 0

#include <iostream>
#include <thread>

class Vehicle
{
public:
    Vehicle(int id) : _id(id) {}
    void operator()()
    {
        std::cout << "Vehicle #" << _id << " has been created" << std::endl;
    }

private:
    int _id;
};

int main()
{
    //sequency: 
    //Q: how to pass value into the thread? 
    //A: by funcntion object, construct func will do, pass the value to the member variable
    // and then it will call the overload operator()????

    // create thread
    //std::thread t((Vehicle(1)));
    std::thread t = std::thread(Vehicle(2)); // Use copy initialization

    // do something in main()
    std::cout << "Finished work in main \n";

    // wait for thread to finish
    t.join();

    return 0;
}

#endif

//Lambda capture[]
#if 0

#include <iostream>

int main() {
    // create lambdas

    //define a variable outside
    int id = 1;

    auto f0 = [id]() mutable {std::cout << "id in lambda: " << ++id << std::endl; };
    f0();

    std::cout << "id in main(): " << id << std::endl ;
    std::cin.get();
}

#endif

//Lambda quiz
#if 1

//b) cout id 1
//c) cout id 0
//d) cout id 1
//e) cout id 1
//f) cout id 2
//e) cout id 2
#include <iostream>

int main()
{
    int id = 0; // Define an integer variable

    // capture by reference (immutable)
    auto f0 = [&id]() { std::cout << "a) ID in Lambda = " << id << std::endl; };

    // capture by value (mutable)
    auto f1 = [id]() mutable { std::cout << "b) ID in Lambda = " << ++id << std::endl; };
    f1(); // call the closure and execute the code witin the Lambda
    std::cout << "c) ID in Main = " << id << std::endl;

    // capture by reference (mutable)
    auto f2 = [&id]() mutable { std::cout << "d) ID in Lambda = " << ++id << std::endl; };
    f2();
    std::cout << "e) ID in Main = " << id << std::endl;

    // pass parameter
    auto f3 = [](const int id) { std::cout << "f) ID in Lambda = " << id << std::endl; };
    f3(++id);

    // observe the effect of capturing by reference at an earlier point in time
    f0();

    return 0;
}

#endif

// lambda in thread
#if 0
#include <iostream>
#include <thread>

int main(){

    int id = 0;

    //starting the first thread
    auto f0 = [&id](){
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        std::cout << "a) ID in Thread (call-by-reference) = " << id << std::endl;
    };
    std::thread t1(f0);

    //starting the second thread(by value)
    std::thread t2([id]() mutable {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        std::cout << "b) ID in Thread (call-by-value) = " << id << std::endl;
    });

    // increment and print id in main
    ++id;
    std::cout << "c) ID in Main (call-by-value) = " << id << std::endl;

    // wait for threads before returning
    t1.join();
    t2.join();

    return 0;
}
#endif

// thread with shared pointer 1
#if 0

#include <iostream>
#include <thread>

class Vehicle
{
public:
    Vehicle() : _id(0) {}
    void addID(int id) { _id = id; }
    void printID()
    {
        std::cout << "Vehicle ID=" << _id << std::endl;
    }

private:
    int _id;
};

int main()
{
    // create thread
        std::shared_ptr<Vehicle> v(new Vehicle);
    std::thread t = std::thread(&Vehicle::addID, v, 1); // call member function on object v

    // wait for thread to finish
    t.join();

    // print Vehicle id
    v->printID();

    return 0;
}
#endif

// thread with shared pointer 2
#if 0
#include <iostream>
#include <thread>

class Vehicle
{
public:
    Vehicle() : _id(0), _name("haha"){}
    void addID(int id) { _id = id; }
    void printID()
    {
        std::cout << "Vehicle ID=" << _id << std::endl;
    }
    // TODO: Modify the Vehicle class as indicated in the instructions on the left.
    void setName(const std::string& name){
        _name = name;
    }
    void printName(){
        std::cout << "Vehicle Name=" << _name << std::endl;
    }
private:
    int _id;
    std::string _name;
};

int main()
{
    // create thread
    std::shared_ptr<Vehicle> v(new Vehicle);
    std::thread t = std::thread(&Vehicle::addID, v, 1); // call member function on object v

    // TODO: Modify the main to start setName as a thread.
    // Also, add code to main to print the name of the vehicle.
    std::thread t1 = std::thread(&Vehicle::setName, v, "Porche");
    // wait for thread to finish
    t.join();
    t1.join();
    // print Vehicle id
    v->printID();
    v->printName();
    return 0;
}
#endif

// multiple threads - bug1?
#if 0
#include <iostream>
#include <thread>
#include <vector>

void printHello()
{
    // perform work
    std::cout << "Hello from Worker thread #" << std::this_thread::get_id() << std::endl;
}

int main()
{
    // create threads
    std::vector<std::thread> threads;
    for (size_t i = 0; i < 5; ++i)
    {
        // copying thread objects causes a compile error
        /*
        std::thread t(printHello);
        threads.push_back(t);
        */

        // moving thread objects will work
        threads.emplace_back(std::thread(printHello));
    }

    // do something in main()
    std::cout << "Hello from Main thread #" << std::this_thread::get_id() << std::endl;

    // call join on all thread objects using a range-based loop
    for (auto &t : threads)
        t.join();

    return 0;
}
#endif

//#include <iostream>
//#include <thread>
//#include <chrono>
//#include <random>
//#include <vector>
//
//int main()
//{
//    // create threads
//    std::vector<std::thread> threads;
//    for (size_t i = 0; i < 10; ++i)
//    {
//
//        // create new thread from a Lambda
//        threads.emplace_back(std::thread([i]() {
//
//            // wait for certain amount of time
//            std::this_thread::sleep_for(std::chrono::milliseconds(10 * i));
//
//            // perform work
//            std::cout << "Hello from Worker thread #" << i << std::endl;
//        }));
//    }
//
//    // do something in main()
//    std::cout << "Hello from Main thread #" << std::this_thread::get_id() <<std::endl;
//
//    // call join on all thread objects using a range-based loop
//    for (auto &t : threads)
//        t.join();
//
//    return 0;
}

















































