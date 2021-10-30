using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor front_L;
extern motor front_R;
extern motor back_L;
extern motor back_R;
extern motor indexer;
extern motor sorter;
extern motor left_intake;
extern motor right_intake;
extern inertial left_inertial;
extern inertial right_inertial;
extern encoder left_encoder;
extern encoder right_encoder;
extern encoder back_encoder;
extern line ballPos1;
extern line ballPos2;
extern controller Controller1;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );