

/**
 * @brief Private struture for storing car state
 * @param speed             Car speed
 * @param steeringWheelDuty Car steering wheel duty (%)
 * @param roadCondition         Road Condition
 * @param pedalState        Brake pedal toggle
 * @param brakePressureDuty     Brake pressure (%)
 */
typedef struct {
    uint8_t speed; //m
    uint8_t steeringWheelDuty; //km/h
    uint8_t roadCondition; 
    bool pedalState; 
    uint8_t brakePressureDuty;
} Car_t;

// Define local car state object
static Car_t carState = {0,0,0,0};

// Getters
uint8_t getCarSpeed(void)
{
    return carState.speed;
}

uint8_t getSteeringDuty(void)
{
    return carState.steeringWheelDuty;
}

uint8_t getRoadCondition(void)
{
    return carState.roadCondition;
}

bool getPedalState(void)
{
    return carState.pedalState;
}

uint8_t getBrakePressureDuty(void)
{
    return carState.brakePressureDuty;
}

// Setters
void setCarSpeed(uint8_t speed)
{
    carState.speed = speed;
}

void setSteeringDuty(uint8_t duty)
{
    carState.steeringWheelDuty = duty;
}

void setRoadCondition(uint8_t condition)
{
    carState.roadCondition = condition;
}

void setPedalState(bool state)
{
    carState.pedalState = state;
}

void setBrakePressureDuty(uint8_t brakePressureDuty)
{
    carState.brakePressureDuty = brakePressureDuty;
}

