#include <iostream>
#include <thread>
#include <vector>

void threadFunctionEven()
{
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // simulate work
    std::cout << "Even thread\n";
}

/* Student Task START */
void threadFunctionOdd() {
    std::this_thread::sleep_for(std::chrono::milliseconds(1)); // simulate work
    std::cout << "Odd threadn\n";
}
/* Student Task END */

int main()
{
    //todo: how to create n threads?
    // thread in vector
    // https://thispointer.com/c11-how-to-create-vector-of-thread-objects/
    // vector ref.
    // https://www.geeksforgeeks.org/vector-in-cpp-stl/
    // https://stackoverflow.com/questions/8221702/accessing-elements-of-a-vector-in-c
    
    std::vector<std::thread> threads;

    int nthreads = 6;

    for (int i = 0; i < nthreads; i++) {

//        threads.push_back(std::thread(threadFunctionEven));

// internally uses move semantics to move our thread object into the vector without making a copy.
        threads.emplace_back(std::thread(threadFunctionEven));
        threads.at(i).detach();
    }


    // ensure that main does not return before the threads are finished
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // simulate work

    std::cout << "End of main is reached" << std::endl;
    return 0;
}
