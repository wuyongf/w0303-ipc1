#include <iostream>
#include <thread>

using namespace std;

void foo(int i) { cout << "foo() i = " << i << endl; }

struct task_struct
{
	int& i;

	task_struct(int& ii) :i(ii) {
		cout << "task_struct constructor i = " << i << endl;
	}

	void operator()()
	{
		for (unsigned j = 0; j < INT_MAX; ++j)
		{
			cout << j << ": foo()\n";
			cout << "foo() i = " << i << endl;
			//foo(i);
		}
	}
};

void A_function_creating_a_thread_within(int& state)
{
	
	task_struct task(state);
	cout << "Launching a thread\n";
	std::thread t(task);
	cout << "detaching the thread\n";
	// do not wait for the thread to finish
	t.detach();

	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

}

int main()
{
	int state = 99;
	A_function_creating_a_thread_within(state);
	cout << "END OF PROGRAM\n";
	return 0;
}