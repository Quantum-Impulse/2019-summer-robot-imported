/*----------------------------------------------------------------------------*/
/* FRC Team --> 5572                                                                            */
/* Enrique`s Translation of java code from 2019 FRC game season               */
/*----------------------------------------------------------------------------*/

#include "Robot.h"
#include <iostream>
#include <string>
#include <frc/smartdashboard/SmartDashboard.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include "cameraserver/CameraServer.h"
#include "DriveTrain.hpp"


// USB Camera Libraries
#include <cscore_oo.h>

//Quick camera thread, Using cs(opencv) to make camera setting changeable 
//r=e^{\sin\left(\theta\right)}-2\cos\left(4\theta\right)+\sin\left(\frac{2\theta-\pi}{24}\right)^{5}
static void VisionThread()
  {
      frc::CameraServer *cameraServer;
      cs::UsbCamera camera = cameraServer->GetInstance()->StartAutomaticCapture();
      camera.SetResolution(240, 120);
      camera.SetFPS(15);
      cs::CvSink cvSink = cameraServer->GetInstance()->GetVideo();
      cs::CvSource outputStreamStd = cameraServer->GetInstance()->PutVideo("DriverStaionCam", 240, 120);
      cv::Mat source;
      cv::Mat output;
      while(true) {
          cvSink.GrabFrame(source);
          cvtColor(source, output, cv::COLOR_BGR2RGB);
          outputStreamStd.PutFrame(output);
      }
  }

void cameraThread() {
  cs::UsbCamera camera = cs::UsbCamera("default_cam", 0);
  cs::CvSource outputStream = frc::CameraServer::GetInstance()->PutVideo(
      "DriveStationVideo", 240, 120);
  cs::CvSink front = frc::CameraServer::GetInstance()->GetVideo(camera);
  cv::Mat mat;
  while (1) {
    front.GrabFrame(mat);
    outputStream.PutFrame(mat);
  }
}

void Robot::RobotInit() {
  
}

void Robot::RobotPeriodic() {
  driveTrain.Drive();
}

void Robot::AutonomousInit() {}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {
  
}

void Robot::TeleopPeriodic() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
