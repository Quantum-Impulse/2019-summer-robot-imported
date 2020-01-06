/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>

#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include "AHRS.h"

#include "controller.hpp"
#include "DriveTrain.hpp"

#include "rev/CANSparkMax.h"

class Robot : public frc::TimedRobot {

/* Creates the controllers objects Driver and Operator
The Drivers ID is set to 0 while the Operator is set to 1*/
 FRC5572Controller Driver{0}; 
 FRC5572Controller Operator{1};

/*Creates the navx-XMP object*/
 AHRS ahrs{SPI::Port::kMXP};


/*The NeoSparks SpeedController instantiation with their CAN ID
 and the type of motor its wired with respectively*/
rev::CANSparkMax m_leftTopMotor{leftLeadDeviceID, 
  rev::CANSparkMax::MotorType::kBrushless};

rev::CANSparkMax m_rightTopMotor{rightLeadDeviceID, 
  rev::CANSparkMax::MotorType::kBrushless};

rev::CANSparkMax m_leftBottomMotor{leftFollowDeviceID, 
  rev::CANSparkMax::MotorType::kBrushless};

rev::CANSparkMax m_rightBottomMotor{rightFollowDeviceID, 
  rev::CANSparkMax::MotorType::kBrushless};


/*instantiation of the double Solenoids with 
their PCM, forwardChannel, reverseChannel  */
frc::DoubleSolenoid Left_Solenoid{PCM1, Left_Solenoid1, Left_Solenoid2};
frc::DoubleSolenoid Right_Solenoid{PCM1, Right_Solenoid1, Right_Solenoid2};


/*instantiation of the compressor with its CAN ID*/ 
Compressor compressor{0};

/*DriveTrain Object  */
DriveTrain driveTrain{
  m_leftTopMotor, 
  m_rightTopMotor, 
  m_leftBottomMotor, 
  m_rightBottomMotor, 
  Driver,
  ahrs, 
  Left_Solenoid, 
  Right_Solenoid};

 /* ID Numbers for Compressors, Speedcontrollers,
 DoubleSoleniods. Lead means front and follow are back  */
static const int 
leftLeadDeviceID = 1, 
leftFollowDeviceID = 2,
rightLeadDeviceID = 3,
rightFollowDeviceID = 4,

PCM1 = 5,
PCM2 = 6,

/* Solenoid port numbers */
Right_Solenoid1 = 1,
Right_Solenoid2 = 6,
Left_Solenoid1 = 0,
Left_Solenoid2 = 7;

 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  
};




