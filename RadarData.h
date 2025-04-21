#ifndef RADAR_DATA_H
#define RADAR_DATA_H

#include <cstdint>
#include <cstddef>
#include <vector>
#include "Target.h"

/**
 * @class RadarData
 * @brief Class for managing radar data including targets and fault information
 * 
 * This class encapsulates the data received from the radar sensor, including
 * any fault codes, the count of targets, and the target information.
 */
class RadarData {
public:
    /**
     * @brief Construct a new RadarData object with default values
     */
    RadarData();
    
    /**
     * @brief Get the fault code
     * @return The fault code from the radar
     */
    uint8_t getFault() const;
    
    /**
     * @brief Set the fault code
     * @param fault The new fault code
     */
    void setFault(uint8_t fault);
    
    /**
     * @brief Get the count of targets
     * @return The number of targets detected
     */
    uint8_t getCount() const;
    
    /**
     * @brief Set the count of targets
     * @param count The new target count
     */
    void setCount(uint8_t count);
    
    /**
     * @brief Get the vector of targets
     * @return Reference to the vector of targets
     */
    const std::vector<Target>& getTargets() const;
    
    /**
     * @brief Add a target to the data
     * @param target The target to add
     */
    void addTarget(const Target& target);
    
    /**
     * @brief Clear all targets
     */
    void clearTargets();
    
    /**
     * @brief Parse radar frame data into this object
     * @param data Pointer to the frame data
     * @param len Length of the frame data
     * @return Reference to this object for method chaining
     */
    RadarData& parseFrame(const uint8_t* data, size_t len);
    
    /**
     * @brief Find the best target (closest one) from the data
     * @return The best target, or an invalid target if none found
     */
    Target findBestTarget() const;

private:
    uint8_t fault_;              ///< Fault code from the radar
    uint8_t count_;              ///< Number of targets detected
    std::vector<Target> targets_; ///< Vector of target objects
};

#endif // RADAR_DATA_H
