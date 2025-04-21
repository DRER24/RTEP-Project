#include "SignalHandler.h"
#include <iostream>
#include <thread>
#include <chrono>

// Initialize static member
SignalHandler* SignalHandler::instance_ = nullptr;

SignalHandler& SignalHandler::getInstance() {
    if (instance_ == nullptr) {
        instance_ = new SignalHandler();
    }
    return *instance_;
}

void SignalHandler::registerCleanupCallback(CleanupCallback callback) {
    cleanup_callback_ = callback;
}

void SignalHandler::setupSignalHandlers() {
    // Register signal handlers
    std::signal(SIGINT, signalHandlerFunc);   // Ctrl+C
    std::signal(SIGTERM, signalHandlerFunc);  // kill
    std::signal(SIGSEGV, signalHandlerFunc);  // Segmentation fault
    std::signal(SIGABRT, signalHandlerFunc);  // Abnormal termination
    
    std::cout << "[INFO] Signal handlers set up" << std::endl;
}

void SignalHandler::cleanup() {
    std::cout << std::endl << "[INFO] Signal received, performing cleanup..." << std::endl;
    
    if (cleanup_callback_) {
        cleanup_callback_();
    }
    
    // Add a delay to ensure I2C commands and other cleanup operations complete
    std::cout << "[INFO] Ensuring all cleanup operations complete..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    
    std::cout << "[INFO] Cleanup complete, exiting" << std::endl;
    
    // This will terminate the program
    std::exit(0);
}

void SignalHandler::signalHandlerFunc(int signal) {
    std::cout << std::endl << "[INFO] Received signal " << signal << ", preparing to exit" << std::endl;
    
    if (instance_) {
        instance_->cleanup();
    } else {
        std::cout << "[WARNING] No signal handler instance available" << std::endl;
        std::exit(1);  // Exit with error
    }
}
