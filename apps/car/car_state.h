#ifndef CAR_STATE_H_
#define CAR_STATE_H_


uint8_t getCarSpeed(void);
uint8_t getSteeringDuty(void);
uint8_t getRoadCondition(void);
bool getPedalState(void);
uint8_t getBrakePressureDuty(void);

void setCarSpeed(uint8_t speed);
void setSteeringDuty(uint8_t duty);
void setRoadCondition(uint8_t condition);
void setPedalState(bool state);
void setBrakePressureDuty(uint8_t brakePressureDuty);

#endif