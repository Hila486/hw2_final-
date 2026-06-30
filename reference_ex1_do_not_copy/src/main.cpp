#include <string>

#include "simulator.h"

int main(int argc, char* argv[]) {
    std::string inputOutputPath = "."; // if input files in our folder

    if (argc > 1) {
        inputOutputPath = argv[1]; // different folder
    }

    Simulator simulator(inputOutputPath);
    return simulator.run();

}


