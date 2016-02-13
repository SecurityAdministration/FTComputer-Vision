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

public class FrankOpOne extends OpMode {

  //final values need to be measured after robot has finished completion
  final static double clawLeftMinimumRange = 0;
  final static double clawLeftMaximumRange = 0;
  final static double clawRightMinimumRange = 0;
  final static double clawRightMaximumRange = 0;

  final static double motorRightMinimum = 0;
  final static double motorRightMaximum = 0;
  final static double motorLeftMinimum = 0;
  final static double motorLeftMaximum = 0;

  final static double armBaseMinimum = 0;
  final static double armBaseMaximum = 0;
  final static double armTopMinimum = 0;
  final static double armTopMaximum = 0;

  double clawLeftPosition = 0;
  double clawLeftDelta = 0;
  double clawRightPosition = 0;
  double clawRightDelta = 0;

  double motorRightPosition = 0;
  double motorLeftPotition = 0;

  double armBasePosition = 0;
  double armTopPosition = 0;

  DcMotor motorRightRaw;
  DcMotor motorLeftRaw;
  DcMotor armBaseRaw;
  DcMotor armTopRaw;

  Servo clawRightRaw;
  Servo clawLeftRaw;

  public FrankOpOne() {

  }

  @Override
  public void init() {

    motorRightRaw = hardwareMap.dcMotor.get("motorRightRaw");
    motorLeftRaw = hardwareMap.dcMotor.get("motorLeftRaw");

    armBaseRaw = hardwareMap.dcMotor.get("armBaseRaw");
    armTopRaw = hardwareMap.dcMotor.get("armTopRaw");

    motorLeftRaw.setDirection(DcMotor.Direction.REVERSE);

    clawRightRaw = hardwareMap.servo.get("clawRightRaw");
    clawLeftRaw = hardwareMap.servo.get("clawLeftRaw");

    ServoLib clawRight = new ServoLib(clawRightRaw, clawRightPosition, clawRightMinimumRange, clawRightMaximumRange);
    ServoLib clawLeft = new ServoLib(clawLeftRaw, clawLeftPosition, clawLeftMinimumRange, clawLeftMaximumRange);

    MotorLib motorRight = new MotorLib(motorRightRaw, motorRightPosition, motorRightMinimum, motorRightMaximum);
    MotorLib motorLeft = new MotorLib(motorLeftRaw, motorLeftPotition, motorLeftMinimum, motorLeftMaximum);

    MotorLib armBase = new MotorLib(armBaseRaw, armBasePosition, armBaseMinimum, armBaseMaximum);
    MotorLib armTop = new MotorLib(armTopRaw, armTopPosition, armTopMinimum, armTopMaximum);

  }

  @Override
  public void loop() {

    float left = -gamepad1.left_stick_y;
    float right = -gamepad1.right_stick_y;

    float armBasePad = -gamepad2.left_stick_x;
    float armTopPad = -gamepad2.right_stick_x;

    right = Range.clip(right, -1, 1);
    left = Range.clip(left, -1, 1);

    right = (float)scaleInput(right);
    left = (float)scaleInput(left);

    motorRight.setMotorPower(right);
    motorLeft.setMotorPower(left);

    armBase.setMotorPower(armBasePad);
    armTop.setMotorPower(armTopPad);

    if (gamepad2.left_bumper) {
      clawRightPosition += clawRightDelta;
      clawLeftPosition += clawLeftDelta;
    }
    if (gamepad2.right_bumper) {
      clawRightPosition -= clawRightDelta;
      clawLeftPosition -= clawLeftDelta;
    }

    clawRightPosition = Range.clip(clawRightPosition, clawRightMinimumRange, clawRightMaximumRange);
    clawLeftPosition = Range.clip(clawLeftPosition, clawLeftMinimumRange, clawLeftMaximumRange);

    clawRight.setServo(clawRightPosition);
    clawLeft.setServo(clawLeftPosition);

  }

  @Override
  public void stop() {

  }

  double scaleInput(double dVal) {
    double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
				0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

		int index = (int) (dVal * 16.0);

		if (index < 0) {
			index = -index;
		}

		if (index > 16) {
			index = 16;
		}

		double dScale = 0.0;
		if (dVal < 0) {
			dScale = -scaleArray[index];
		} else {
			dScale = scaleArray[index];
		}

		return dScale;
	}


}
