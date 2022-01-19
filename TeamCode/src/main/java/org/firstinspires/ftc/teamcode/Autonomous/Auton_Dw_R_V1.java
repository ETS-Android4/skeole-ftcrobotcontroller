package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Systems.RobotHardware;

import java.util.Set;

//@Disabled
@Autonomous(name = "Red Duckwheel Auton (V1)")
public class Auton_Dw_R_V1 extends LinearOpMode implements Auton_Values{

    private ElapsedTime runtime = new ElapsedTime();
    private RobotHardware robot = new RobotHardware();

    private int elementPosition = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap, telemetry);


        waitForStart();
        while (opModeIsActive()) {

//======================================================
//---------------Main Code Starts Here------------------
//======================================================

//Read Barcode

            Drive(7.5);
            checkPos(1);

            if(elementPosition == 1) {
                Drive(distance_between_squares*2, "Right");
            }else{
                Drive(distance_between_squares, "Right");
                checkPos(2);
                if(elementPosition == 0) {
                    elementPosition = 3;
                }
                Drive(distance_between_squares, "Right");
            }
            //ends up by 1st square

            robot.telemetry.addData("Barcode: ", elementPosition);
            robot.telemetry.update();

//END Read Barcode

//Drop Block in Tower

            Drive(8, "Right");
            Drive(3.5);

            robot.IN.setPower(0.4);
            robot.Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SetArm(0);
            robot.rServo.setPosition(servoM);
            robot.lServo.setPosition(1-servoM);
            robot.IN.setPower(0);
            sleep(100);
            SetArm(elementPosition);
            robot.rServo.setPosition(servoBM);
            robot.lServo.setPosition(1-servoBM);
            sleep(100);
            robot.rServo.setPosition(servoD);
            robot.lServo.setPosition(1-servoD);

            sleep(2000);

//END Drop Block in Tower

//Reset Arm

            robot.rServo.setPosition(servoB);
            robot.lServo.setPosition(1-servoB);
            SetArm(0);
            sleep(1000);

//END Reset Arm

//Spin Duck Wheel

            Drive(-3.1);
            Drive(25.5, "Left");

            Drive(-4);
            Turn(130, "Right");
            Drive(5.2, "Right", 0.4);

            robot.Duck.setPower(duckMotorPowerA);
            sleep(3000);
            robot.Duck.setPower(duckMotorPowerB); // ***VERY IMPORTANT TO CHECK***
            sleep(1000);
            robot.Duck.setPower(0);

            Drive(2, "Left");

//END Spin Duck Wheel

//Park

            Turn(130, "Left");
            Drive(11);
            Drive(3, "Left");

//END Park



            stop();

//======================================================
//-----------------Main Code Ends Here------------------
//======================================================

        }
    }

    private void ExecuteEncoders() {
        robot.SpeedSet(0.75);
        while (robot.MotorsBusy() && opModeIsActive()) {
            idle();
        }
        robot.SpeedSet(0.2);
        sleep(100);
        robot.SpeedSet(0);
        sleep(200);
    }
    private void ExecuteEncoders(double Speed) {
        robot.SpeedSet(Speed);
        while (robot.MotorsBusy() && opModeIsActive()) {
            idle();
        }
        robot.SpeedSet(0.2);
        sleep(100);
        robot.SpeedSet(0);
        sleep(200);
    }

    private void checkPos(int position){
        //robot.getGreen() > 100 &&
        elementPosition = 0;
        if(robot.getDistInch() < 4){
            elementPosition = position;
        }
    }

    private void Drive(double Dist){
        robot.DriveDistance(-Dist);
        ExecuteEncoders();
    }

    private void Drive(double Dist, String Direction){
        robot.DriveDistance(Dist, Direction);
        ExecuteEncoders();
    }
    private void Drive(double Dist, String Direction, double Speed){
        robot.DriveDistance(Dist, Direction);
        ExecuteEncoders(Speed);
    }

    private void Turn(int Degrees, String Direction){
        if(Direction.equals("Right")){
            robot.turnEncoderDegree(Degrees);
        }else if(Direction.equals("Left")){
            robot.turnEncoderDegree(-Degrees);
        }
        ExecuteEncoders();
    }
    private void SetArm(int pos){

        switch(pos){
            case 1:

                robot.Arm.setTargetPosition(low_goal);
                break;
            case 2:

                robot.Arm.setTargetPosition(mid_goal);
                break;
            case 3:

                robot.Arm.setTargetPosition(high_goal);
                break;
            case 0:
                robot.Arm.setTargetPosition(reset_arm);
                break;
        }
        robot.Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.Arm.setPower(armSpeed);
        while(robot.Arm.isBusy() && opModeIsActive()){
            idle();
        }
        robot.Arm.setPower(0.1);

    }
}
