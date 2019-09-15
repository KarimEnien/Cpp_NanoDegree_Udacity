#include <thread>
#include <vector>
#include<algorithm>

/*
    std::cout << "Hello concurrent world from main! Thread id = " << std::this_thread::get_id() << std::endl;
	
	unsigned int nCores = std::thread::hardware_concurrency();
    std::cout << "This machine supports concurrency with " << nCores << " cores available" << std::endl;

*/

int main(int argc, char const *argv[])
{
	std::vector<std::thread> threads;
	
	// Start up n threads

	int nThreads = 4;
	for(int i = 0; i < nThreads; ++i)
	{
		threads.emplace_back(std::thread([]() {
			while (true);
		}));
	}

	// wait for threads to finish before leaving main
	std::for_each(threads.begin(), threads.end(), [](std::thread &t) {
		t.join();
	});

	return 0;
}