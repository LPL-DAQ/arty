#include <iostream>
#include <boost/unordered_set.hpp>
#include <LabJackM.h>

int main() {
    std::cout << "Hello :) whatsup!" << std::endl;

    // Boost works
    boost::unordered_set<std::string> boost_test_set;
    boost_test_set.insert("hello");
    std::cout << boost_test_set.size() << std::endl;
    std::cout << "Found hello: " << boost_test_set.contains("hello") << std::endl;

    // Labjack works
    int error, handle;
    error = LJM_OpenS("T7", "USB", "ANY", &handle);
    std::cout << "Got error from labjack: " << error << std::endl;
}
