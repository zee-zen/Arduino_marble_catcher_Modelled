#ifndef sensormodule_h
#define sensormodule_h

// Flag definitions that detect the status of the ball at various stages
volatile bool marble_detected=0;
volatile bool checkpoint_US1=0, checkpoint_US2=0, checkpoint_US3=0;
volatile bool LS_flag=0;
volatile bool catcher_success=0;

uint8_t pos_US1_cm, pos_US2_cm;
uint8_t pos_catch;


#endif
