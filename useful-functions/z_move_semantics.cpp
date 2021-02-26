#pragma region Move semantics Move constructors Move assignment operator

#if 0 


#include <stdlib.h>
#include <iostream>

class MyMovableClass
{
private:
    int _size;
    int* _data;

public:
    MyMovableClass(size_t size) // constructor
    {
        _size = size;
        _data = new int[_size];
        std::cout << "CREATING instance of MyMovableClass at " << this << " allocated with size = " << _size * sizeof(int) << " bytes" << std::endl;
    }

    MyMovableClass(const MyMovableClass& source) // 2 : copy constructor
    {
        _size = source._size;
        _data = new int[_size];
        *_data = *source._data;
        std::cout << "COPYING content of instance " << &source << " to instance " << this << std::endl;
    }

    MyMovableClass& operator=(const MyMovableClass& source) // 3 : copy assignment operator
    {
        std::cout << "ASSIGNING content of instance " << &source << " to instance " << this << std::endl;
        if (this == &source)
            return *this;
        delete[] _data;
        _data = new int[source._size];
        *_data = *source._data;
        _size = source._size;
        return *this;
    }

    MyMovableClass(MyMovableClass&& source) // 4 : move constructor
    {
        std::cout << "MOVING (c¡¯tor) instance " << &source << " to instance " << this << std::endl;
        _data = source._data;
        _size = source._size;
        source._data = nullptr;
        source._size = 0;
    }

    MyMovableClass& operator=(MyMovableClass&& source) // 5 : move assignment operator
    {
        std::cout << "MOVING (assign) instance " << &source << " to instance " << this << std::endl;
        if (this == &source)
            return *this;

        delete[] _data;

        _data = source._data;
        _size = source._size;

        source._data = nullptr;
        source._size = 0;

        return *this;
    }

    ~MyMovableClass() // 1 : destructor
    {
        std::cout << "DELETING instance of MyMovableClass at " << this << std::endl;
        delete[] _data;
    }
};

MyMovableClass createObject(int size) {
    MyMovableClass obj(size); // regular constructor
    return obj; // return MyMovableClass object by value
}

void useObject(MyMovableClass obj)
{
    std::cout << "using object " << &obj << std::endl;
}

//int main()
//{
//    MyMovableClass obj1(10); // regular constructor
//    MyMovableClass obj2(obj1); // copy constructor
//    obj2 = obj1; // copy assignment operator
//
//
//
//    return 0;
//}

int main() {
    
    #if 0
    MyMovableClass obj1(10);
    // call to copy constructor, (alternate syntax)
    //MyMovableClass obj3 = obj1;
    // Here, we are instantiating obj3 in the same statement; hence the copy assignment operator would not be called.

    MyMovableClass obj4 = createObject(10);
    // createObject(10) returns a temporary copy of the object as an rvalue, which is passed to the copy constructor.

    std::cout << "already delete the temperary rvalue?  YES!!!!" << std::endl;
    
     //You can try executing the statement below as well
     //MyMovableClass obj4(createObject(10));

    return 0;
    #endif
    
    #if 0
    MyMovableClass obj1(100), obj2(200); // constructor

    MyMovableClass obj3(obj1); // copy constructor

    MyMovableClass obj4 = obj1; // copy constructor

    obj4 = obj2; // copy assignment operator

    return 0;
    #endif
    
    #if 0
    MyMovableClass obj1(100); // constructor
    
    obj1 = MyMovableClass(200); // move assignment operator

    MyMovableClass obj2 = MyMovableClass(300); // move constructor 

    return 0;
    #endif

    MyMovableClass obj1(100); // constructor

    useObject(obj1);

    return 0;


}

#endif

#pragma endregion



#pragma region smart pointers! finally.

#if 0
#pragma region unique pointers

#include <iostream>
#include <memory>
#include <string>

//std::unique_ptr<int> unique(new int);
/*std::unique_ptr<int> unique2 = std::make_unique<int>();*/

class MyClass
{
private:
    std::string _text;

public:
    MyClass() {}
    MyClass(std::string text) { _text = text; }
    ~MyClass() { std::cout << _text << " destroyed" << std::endl; }
    void setText(std::string text) { _text = text; }
};

int main()
{
    // create unique pointer to proprietary class
    std::unique_ptr<MyClass> myClass1(new MyClass());
    std::unique_ptr<MyClass> myClass2(new MyClass("String 2"));

    // call member function using ->
    myClass1->setText("String 1");

    // use the dereference operator * 
    *myClass1 = *myClass2;

    // use the .get() function to retrieve a raw pointer to the object
    std::cout << "Objects have stack addresses " << myClass1.get() << " and " << myClass2.get() << std::endl;

    return 0;
}

#pragma endregion
#endif

#if 0
#pragma region shared pointers

#include <iostream>
#include <memory>

#include <iostream>
#include <memory>

class MyClass
{
public:
    ~MyClass() { std::cout << "Destructor of MyClass called" << std::endl; }
};

int main()
{
    std::shared_ptr<MyClass> shared(new MyClass);
    std::cout << "shared pointer count = " << shared.use_count() << std::endl;

    std::shared_ptr<MyClass> shared1 = shared;
    std::cout << "shared pointer count = " << shared.use_count() << std::endl;

    shared.reset(new MyClass);
    std::cout << "shared pointer count = " << shared.use_count() << std::endl;

    return 0;
}


#pragma endregion
#endif

#if 0
#pragma region weak pointers
#include <iostream>
#include <memory>

int main()
{
    std::shared_ptr<int> mySharedPtr(new int);

    std::weak_ptr<int> myWeakPtr(mySharedPtr);

    mySharedPtr.reset(new int);

    if (myWeakPtr.expired() == true)
    {
        std::cout << "Weak pointer expired!" << std::endl;
    }


    return 0;
}

#pragma endregion
#pragma endregion
#endif

#include <iostream>
#include <thread>

#include <iostream>

int main()
{
    int id = 0; // Define an integer variable

    // capture by reference (immutable) 
    auto f0 = [&id]() { std::cout << "a) ID in Lambda = " << id << std::endl; };
    // capture by value (mutable) 

    auto f1 = [id]() mutable { std::cout << "b) ID in Lambda = " << ++id << std::endl; };
    f1(); // call the closure and execute the code witin the Lambda 1
    std::cout << "c) ID in Main = " << id << std::endl;

    // capture by reference (mutable) 
    auto f2 = [&id]() mutable { std::cout << "d) ID in Lambda = " << ++id << std::endl; };
    f2(); 
    std::cout << "e) ID in Main = " << id << std::endl;

    // pass parameter  
    auto f3 = [](const int id) { std::cout << "f) ID in Lambda = " << id << std::endl; };
    f3(++id); 
    std::cout << "f) ID in Main = " << id << std::endl;
    // observe the effect of capturing by reference at an earlier point in time
    f0(); 

    return 0;
}
