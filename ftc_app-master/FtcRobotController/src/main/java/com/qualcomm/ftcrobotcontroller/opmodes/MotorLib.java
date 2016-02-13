/* Copyright (c) 2016, Security Administration

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Security Administration nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
package com.qualcomm.ftcrobotcontroller.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class MotorLib {

  DcMotor motor;
  double motorPosition = 0;
  double motorMin = -1000000;
  double motorMax = 1000000;

  public MotorLib(Motor inputMotor, double inputMotorPosition, double inputMotorMin, double inputMotorMax) {
    motor = inputMotor;
    motorPosition = inputMotorPosition;
    motorMin = inputMotorMin;
    motorMax = inputMotorMax;

  }

  public double getMotor() {
    return motorPosition;
  }

  public void setMotor(double degree, double power) {

      double finalMove = motorPosition + degree;

      if (power > 1 || power < 0) return 1;
      if (degree < 0) power = -power;

      for (int i = 0; 0 < degree + 1; i++)
      {
        motor.setPower(power);
        if (power < 0) {
          motorPosition -= 1;
        } else {
          motorPostition += 1;
        }
        try {
          Thread.sleep(1);
        } catch(InteruptedException e) {
          Thread.currentThread().interrupt();
        }
        motor.setPower(0);
        }
      }
  }

  pubic void setMotorPower(double inputPower) {
    motor.setPower(inputPower);
  }

}
