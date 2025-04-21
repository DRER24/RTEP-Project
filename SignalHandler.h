#ifndef SIGNAL_HANDLER_H
#define SIGNAL_HANDLER_H

#include <functional>
#include <csignal>

/**
 * @class SignalHandler
 * @brief Handles system signals for graceful program termination
 * 
 * This class sets up handlers for system signals like SIGINT (Ctrl+C),
 * SIGTERM, etc., to allow for proper cleanup before program exit.
 */
class SignalHandler {
public:
    /**
     * @brief Callback function type for cleanup
     */
    using CleanupCallback = std::function<void()>;
    
    /**
     * @brief Get the singleton instance
     * @return Reference to the singleton SignalHandler
     */
    static SignalHandler& getInstance();
    
    /**
     * @brief Register a cleanup callback
     * @param callback Function to call when a signal is received
     */
    void registerCleanupCallback(CleanupCallback callback);
    
    /**
     * @brief Setup signal handlers
     */
    void setupSignalHandlers();
    
    /**
     * @brief Call the cleanup callback
     * 
     * This is public so the static signal handler function can call it.
     */
    void cleanup();

private:
    /**
     * @brief Construct a new SignalHandler (private for singleton)
     */
    SignalHandler() = default;
    
    /**
     * @brief Deleted copy constructor
     */
    SignalHandler(const SignalHandler&) = delete;
    
    /**
     * @brief Deleted assignment operator
     */
    SignalHandler& operator=(const SignalHandler&) = delete;
    
    /**
     * @brief Static signal handler function
     * @param signal Signal number
     */
    static void signalHandlerFunc(int signal);
    
    CleanupCallback cleanup_callback_; ///< Callback for cleanup
    static SignalHandler* instance_;   ///< Singleton instance
};

#endif // SIGNAL_HANDLER_H
