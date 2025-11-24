#include <drake/common/drake_assert.h>
#include <iostream>

int main() {
    std::cout << "Drake is installed and linked correctly!" << std::endl;
    DRAKE_DEMAND(1 + 1 == 2);  // Simple runtime check
    return 0;
}

