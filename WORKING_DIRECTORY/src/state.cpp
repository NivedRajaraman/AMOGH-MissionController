
#include "state.hpp"


STATE::STATE()
{
  _taskID = 'n';
  _startTime = std::chrono::steady_clock::now();
  _timeDiff = 0.01;
  _r = 0;
  _theta = 0;
  _phi = 0;
  _roll = 0;
  _pitch = 0;
  _yaw = 0;
  _gyroX = 0;
  _gyroY = 0;
  _gyroZ = 0;
  _pressure = 0;
  _PWM_Side[0] = 1500;
  _PWM_Side[1] = 1500;
  _PWM_Back[0] = 1500;
  _PWM_Back[1] = 1500;
  _PWM_Bottom[0] = 1500;
  _PWM_Bottom[1] = 1500;
  _Port = 0;
};

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&      FLOW CONTROL FUNCTIONS     &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

/*Set and get the value of the flag (flow control variable)*/

  void STATE::set_taskID(char C) {
    this->_taskID = C;
  }
  char STATE::return_taskID() {
    return this->_taskID;
  }

/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&      ARDUINO PORT FUNCTIONS     &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

  void STATE::set_port(int port) {
    this->_Port = port;
  }

  void STATE::thruster_update()
  {
    int size = snprintf(NULL, 0, "%04d,%04d,%04d,%04d,%04d,%04d\n", _PWM_Side[0],_PWM_Side[1],_PWM_Back[0],_PWM_Back[1],_PWM_Bottom[0],_PWM_Bottom[1]);
    //NOTE :: \n is a single character

    char transmit[size + 1];      // holds the string that will be transmitted to Arduino
    sprintf(transmit, "%04d,%04d,%04d,%04d,%04d,%04d\n", _PWM_Side[0],_PWM_Side[1],_PWM_Back[0],_PWM_Back[1],_PWM_Bottom[0],_PWM_Bottom[1]);

    int wr = write(this->_Port, transmit, size);  // don't allow final \0 to pass through
    if (wr == -1) {
        printf("error in write");
    }
  }


/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&        DATA FUNCTIONS       &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/
/*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&*/

// ---------------------------------------------------TIME FUNCTIONS-----------------------------------------------------

  double STATE::global_time() {
    std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
    std::chrono::steady_clock::duration timeSpan = currentTime - _startTime;
    return double(timeSpan.count())*std::chrono::steady_clock::period::num/std::chrono::steady_clock::period::den;
  }

  void STATE::set_timeDiff(float t) {
    this->_timeDiff = t;
  }

  float STATE::return_timeDiff() {
    return this->_timeDiff;
  }


// -----------------------------------------------POSITION & IP FUNCTIONS-------------------------------------------------

// IP (camera) readings, r, theta and phi

  void STATE::set_r(float r) {
    this->_r = r;
  }
  void STATE::set_theta(float theta){
    this->_theta = theta;
  }
  void STATE::set_phi(float phi) {
    this->_phi = phi;
  }


  float STATE::return_r() {
    return this->_r;
  }
  float STATE::return_theta() {
    return this->_theta;
  }
  float STATE::return_phi() {
    return this->_phi;
  }

//-----------------------------------------------------------------------------------------------------------------------------
// Roll, pitch and yaw readings from the IMU

  void STATE::set_roll(float roll) {
    this->_roll = roll;
  }
  void STATE::set_pitch(float pitch) {
    this->_pitch = pitch;
  }
  void STATE::set_yaw(float yaw) {
    this->_yaw = yaw;
  }


  float STATE::return_roll() {
    return this->_roll;
  }
  float STATE::return_pitch() {
    return this->_pitch;
  }
  float STATE::return_yaw() {
    return this->_yaw;
  }

//-----------------------------------------------------------------------------------------------------------------------------
// gyroX gyroY gyroZ readings

  void STATE::set_gyroX(float gyroX) {
    this->_gyroX = gyroX;
  }
  void STATE::set_gyroY(float gyroY) {
    this->_gyroY = gyroY;
  }
  void STATE::set_gyroZ(float gyroZ) {
    this->_gyroZ = gyroZ;
  }


  float STATE::return_gyroX() {
    return this->_gyroX;
  }
  float STATE::return_gyroY() {
    return this->_gyroY;
  }
  float STATE::return_gyroZ() {
    return this->_gyroZ;
  }

//-----------------------------------------------------------------------------------------------------------------------------
// Pressure readings given by the pressure sensor

  void STATE::set_pressure(float pressure) {
    this->_pressure = pressure;
  }
  float STATE::return_pressure() {
    return this->_pressure;
  }

//-----------------------------------------------------------------------------------------------------------------------------
// PWM values that are pushed to thrusters

  void STATE::set_PWM_Side(int* PWM) {
    this->_PWM_Side[0] = *(PWM);
    this->_PWM_Side[1] = *(PWM + 1); 
  }
  void STATE::set_PWM_Back(int* PWM) {
    this->_PWM_Back[0] = *(PWM);
    this->_PWM_Back[1] = *(PWM + 1);  
  }
  void STATE::set_PWM_Bottom(int* PWM) {
    this->_PWM_Bottom[0] = *(PWM);
    this->_PWM_Bottom[1] = *(PWM + 1);  
  }

  // Get PWM readings

  void STATE::return_PWM_Side(int* PWM) {
    *(PWM) = *(this->_PWM_Side);
    *(PWM + 1) = *(this->_PWM_Side + 1);
  }
  void STATE::return_PWM_Back(int* PWM) {
    *(PWM) = *(this->_PWM_Back);
    *(PWM + 1) = *(this->_PWM_Back + 1);
  }
  void STATE::return_PWM_Bottom(int* PWM) {
    *(PWM) = *(this->_PWM_Bottom);
    *(PWM + 1) = *(this->_PWM_Bottom + 1);
  }

// Call boundary check just before these values are read by the Arduino and given to the thrusteres via ESC
  void STATE::boundaryCheck(int* PWM) {    // IF between 1525 and 1475 make it 1500. If it is more than 1900 make it 1900 (here I conservatively chose 1875 as the max instead of 1900)
      *PWM = *PWM > 1875 ? 1875 : *PWM;
      *(PWM+1) = *(PWM+1) > 1875 ? 1875 : *(PWM+1);
      
      *(PWM) = *PWM < 1125 ? 1125 : *(PWM);
      *(PWM+1) = *(PWM+1) < 1125 ? 1125 : *(PWM+1);

      *PWM = std::abs(*PWM - 1500) < 25 ? 1500 : *PWM;
      *(PWM+1) = std::abs(*(PWM+1) - 1500) < 25 ? 1500 : *(PWM+1);

  }
