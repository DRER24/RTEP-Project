#ifndef TARGET_H
#define TARGET_H

#include <cstdint>

/**
 * @class Target
 * @brief Represents a radar target with position and angle information
 * 
 * This class encapsulates target data including ID, distance, angles, and coordinates.
 * It also tracks validity of the target data.
 */
class Target {
public:
    /**
     * @brief Construct a new Target object with default values
     */
    Target();
    
    /**
     * @brief Get the target ID
     * @return The target ID
     */
    uint8_t getId() const;
    
    /**
     * @brief Set the target ID
     * @param id The new target ID
     */
    void setId(uint8_t id);
    
    /**
     * @brief Get the target distance
     * @return The distance in meters
     */
    float getDistance() const;
    
    /**
     * @brief Set the target distance
     * @param dist The new distance in meters
     */
    void setDistance(float dist);
    
    /**
     * @brief Get the pitch angle
     * @return The pitch angle in degrees
     */
    int8_t getPitch() const;
    
    /**
     * @brief Set the pitch angle
     * @param pitch The new pitch angle in degrees
     */
    void setPitch(int8_t pitch);
    
    /**
     * @brief Get the yaw angle
     * @return The yaw angle in degrees
     */
    int8_t getYaw() const;
    
    /**
     * @brief Set the yaw angle
     * @param yaw The new yaw angle in degrees
     */
    void setYaw(int8_t yaw);
    
    /**
     * @brief Get the X coordinate
     * @return The X coordinate in meters
     */
    float getX() const;
    
    /**
     * @brief Set the X coordinate
     * @param x The new X coordinate in meters
     */
    void setX(float x);
    
    /**
     * @brief Get the Y coordinate
     * @return The Y coordinate in meters
     */
    float getY() const;
    
    /**
     * @brief Set the Y coordinate
     * @param y The new Y coordinate in meters
     */
    void setY(float y);
    
    /**
     * @brief Check if the target is valid
     * @return true if the target is valid, false otherwise
     */
    bool isValid() const;
    
    /**
     * @brief Set the validity of the target
     * @param valid The new validity status
     */
    void setValid(bool valid);

private:
    uint8_t id_;    ///< Target ID
    float   dist_;  ///< Distance in meters
    int8_t  pitch_; ///< Pitch angle in degrees
    int8_t  yaw_;   ///< Yaw angle in degrees
    float   x_;     ///< X coordinate in meters
    float   y_;     ///< Y coordinate in meters
    bool    valid_; ///< Indicates if the target is valid
};

#endif // TARGET_H
