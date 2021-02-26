#include <iostream>
#include <thread>
#include <future>

class Vehicle
{
public:
    //default constructor
    Vehicle() : _id(0), _name(new std::string("Default Name"))
    {
        std::cout << "Vehicle #" << _id << " Default constructor called" << std::endl;
    }

    //initializing constructor
    Vehicle(int id, std::string name) : _id(id), _name(new std::string(name))
    {
        std::cout << "Vehicle #" << _id << " Initializing constructor called" << std::endl;
    }

    // copy constructor
    Vehicle(Vehicle const&src)
    {
        // todo: a deep copy.
        // _name = src._name;  // this is shallow copy.
        std::cout << "haha2" << std::endl;
        _id = src._id;
        if (src._name != nullptr )
        {
            // how to let _name point to a new space?
            _name = new std::string;
            *_name = *src._name; // deep copy.
        }
        std::cout << "Vehicle #" << _id << " copy constructor called" << std::endl;
    }

    // setter and getter
    void setID(int id) { _id = id; }
    int getID() { return _id; }
    void setName(std::string name) { *_name = name; }
    std::string getName() { return *_name; }

private:
    int _id;
    std::string *_name;
};

int main()
{
    // create instances of class Vehicle
    Vehicle v0;    // default constructor
    Vehicle v1(1, "Vehicle 1"); // initializing constructor

    Vehicle v4(v0);

    // launch a thread that modifies the Vehicle name
    std::cout << "--haha--" << std::endl;

    std::future<void> ftr = std::async([](Vehicle v) {
        std::cout << "haha1" << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); // simulate work
        v.setName("Vehicle 2");
    },v0);

    v0.setName("Vehicle 3");

    ftr.wait();
    std::cout << v0.getName() << std::endl;

    return 0;
}