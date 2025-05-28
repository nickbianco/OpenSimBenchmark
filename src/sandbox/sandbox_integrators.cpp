#include <vector>
#include <chrono>
#include <iostream>

void integrateWithoutPreallocation(int steps, double dt, double k) {
    double y = 1.0;  // initial condition
    for (int i = 0; i < steps; ++i) {
        std::vector<double> dydt(1); // allocates on each step
        dydt[0] = -k * y;
        y += dt * dydt[0];
    }
}

void integrateWithPreallocation(int steps, double dt, double k) {
    double y = 1.0;
    std::vector<double> dydt(1); // allocated once
    for (int i = 0; i < steps; ++i) {
        dydt[0] = -k * y;
        y += dt * dydt[0];
    }
}

int main() {
    const int steps = 10000000;
    const double dt = 0.001;
    const double k = 0.1;

    auto start = std::chrono::high_resolution_clock::now();
    integrateWithoutPreallocation(steps, dt, k);
    auto end = std::chrono::high_resolution_clock::now();
    auto durationWithout = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cout << "Without preallocation: " << durationWithout << " nanoseconds\n";

    start = std::chrono::high_resolution_clock::now();
    integrateWithPreallocation(steps, dt, k);
    end = std::chrono::high_resolution_clock::now();
    auto durationWith = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::cout << "With preallocation:    " << durationWith << " nanoseconds\n";

    return 0;
}

