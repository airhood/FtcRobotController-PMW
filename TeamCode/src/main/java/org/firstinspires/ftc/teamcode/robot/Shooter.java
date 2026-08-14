package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {

    private final DcMotor shooterMotor;

    public Shooter(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotor.class, "motor4");
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        shooterMotor.setPower(power);
    }

    public void stop() {
        shooterMotor.setPower(0);
    }

    public void setPowerForDistance(double distanceMM) {
        // TODO: 거리 기반 파워 계산 미구현. 항상 고정값 반환.
        setPower(1.0);
    }
}