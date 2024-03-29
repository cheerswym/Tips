#include <stdio.h>
#include <stdlib.h>

#include <chrono>    // std::chrono::seconds
#include <iostream>  // std::cout
#include <thread>    // std::thread, std::this_thread::sleep_for

void thread_task(int n) {
    std::this_thread::sleep_for(std::chrono::seconds(n));
    std::cout << "hello thread "
        << std::this_thread::get_id()
        << " paused " << n << " seconds" << std::endl;
}

int main(int argc, const char *argv[])
{
    std::thread threads[5];
    std::cout << "Spawning 5 threads...\n";
    int j = 6;
    for (int i = 0; i < 5; i++) {
        threads[i] = std::thread(thread_task, j--);
    }
    std::cout << "Done spawning threads! Now wait for them to join\n";
    for (auto& t: threads) {
		std::cout << "one joined.\n";
        t.join();
    }
    std::cout << "All threads joined.\n";

    return EXIT_SUCCESS;
}
