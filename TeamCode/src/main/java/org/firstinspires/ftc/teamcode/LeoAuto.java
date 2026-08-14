package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.robot.Drive;
import org.firstinspires.ftc.teamcode.robot.Intake;
import org.firstinspires.ftc.teamcode.robot.Localizer;
import org.firstinspires.ftc.teamcode.robot.Shooter;
import org.firstinspires.ftc.teamcode.robot.Vision;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

/**
 * Autonomous 상태머신.
 *
 * *** 대회 당일 필수 체크: START_X_MM / START_Y_MM / START_HEADING_DEG (약 111줄 부근)는
 * *** 연습 장소 기준값이라 실제 경기장 배치와 다름. 로봇을 실제 시작 위치/방향에 놓은 직후
 * *** 이 세 값을 실측/계산해서 반드시 수정 후 재빌드할 것. ***
 *
 * 흐름 (2026-08-01 논의 기준):
 *   INIT_ALLIANCE      : 동맹 색상 선택 대기
 *   SEARCH_ARTIFACT     : 카메라로 아티팩트(보라/초록) 블롭 탐색, 헤딩 회전하며 찾기
 *   APPROACH_ARTIFACT   : 블롭 방향으로 헤딩 정렬 + 전진 (이동 중 자동 흡입)
 *   AIM_AT_GOAL          : 고정 슈팅 구역으로 이동 + AprilTag(골대)로 헤딩/거리 정렬
 *   SHOOT                : 캘리브레이션 룩업테이블로 파워 설정 후 발사
 *   PARK                 : 발사 라인 밖으로 이동 (G206 리더십 3점 확보)
 *   DONE                 : 정지
 *
 * 확정된 것 (대회 규칙 PDF 확인 완료):
 *   - G206: AUTO 종료 시 로봇이 발사 라인 위/관중 쪽에 있으면 안 됨 (리더십 3점 조건)
 *
 * 아직 미정이라 플레이스홀더/TODO로 남긴 것:
 *   - 인테이크 흡입 완료 판단: robot/Intake.java. 색+근접 겸용 센서(REV Color Sensor V3급)
 *     90% 가까이 유력(2026-08-01)하나 확정 아님. 임계값(근접 30mm) 실측 필요.
 *   - 인테이크 모터(motor3): on/off 단순 제어로 확정(2026-08-01), robot/Intake.java에 구현됨.
 *     INTAKE_POWER 값은 실측 필요.
 *   - 슈팅 모터(motor4): encoder 기반 RPM 제어 필요 여부 아직 미정(2026-08-01). 우선
 *     RUN_WITHOUT_ENCODER + 직접 파워로 시작(robot/Shooter.java). RPM 제어 필요해지면
 *     Shooter 내부만 교체하면 됨.
 *   - 슈팅 거리→파워 룩업테이블 미구현. Shooter.setPowerForDistance()는 항상 고정값(0.8)
 *     반환하는 플레이스홀더.
 *   - 슈팅 위치 전략: 고정 슈팅으로 확정(2026-08-01). 슈팅 자체 정확도가 아직 미검증이라
 *     이동 슈팅 복잡도는 나중으로 미룸. 확장성을 위해 "슈팅 구역"을 좌표가 아니라
 *     AprilTag 거리 범위(SHOOT_SPOT_RANGE_MIN/MAX_MM)로 정의해둠 - 나중에 이동 슈팅으로
 *     바꿀 때 이 범위만 넓히거나 없애면 됨 (handleAimAtGoal() 주석 참고).
 *   - 슈팅 반복 횟수 / 종료 조건 - 미정. 현재는 1회 슈팅 후 바로 PARK로 이동하는 최소 버전.
 *   - 발사 완료 판정 - 아직 없어 SHOOT_DURATION_SEC 시간 대기로만 처리.
 *   - Drive의 회전/직진 제어 로직 - 이전에 미검증 PID를 걷어냈고, 지금은 전부 임시
 *     비례제어(게인값 미검증). 실제 로봇에서 재조정 필요.
 *   - 위치추정(EKF, robot/Localizer.java): 상태벡터[x,y,theta], predict(encoder+IMU)/
 *     update(AprilTag) 구조는 구현됨. 필드 좌표계는 필드 중앙 원점(0,0), X축 오른쪽,
 *     Y축 안쪽, heading 0=+X 방향으로 확정(2026-08-01). 단, 로봇 시작 위치, 골대 태그의
 *     정확한 필드 좌표(GOAL_*_FIELD_*_MM)는 공식 필드 CAD 확인 전까지 임시값. Q/R
 *     노이즈 파라미터, encoder tick→mm 변환 계수도 전부 미검증 플레이스홀더.
 *     현재 상태머신에서는 텔레메트리 확인용으로만 predict/update를 매 틱 호출하고
 *     있고, 아직 상태머신의 판단 로직(예: 태그 안 보일 때 마지막 추정 위치 활용)에는
 *     연결 안 함 - 이후 확장 여지.
 *   - 목표 모티프(패턴) 인식: robot/Vision.java에 getTargetMotif()/PATTERN_* 상수 추가함
 *     (2026-08-01). 대기 중 오벨리스크 태그를 읽어 targetMotif에 저장하고 텔레메트리로
 *     확인 가능. 단, 이번 AUTO 전략은 아직 패턴 매칭을 시도하지 않고 색 무관하게 먼저
 *     보이는 아티팩트를 먹는 것으로 확정(2026-08-01) - 목표가 특정 색을 얻기 어려우면
 *     아무 색이나 먹어서 점수부터 확보하는 게 나을 수 있다는 판단. targetMotif는 지금
 *     상태머신 로직에는 전혀 반영 안 되고, 나중에 패턴 전략을 추가할 때 바로 쓸 수
 *     있도록 준비만 해둔 상태. 오벨리스크는 "위치"가 아니라 "어느 태그가 보이는지"만
 *     사용하므로 PDF의 오벨리스크 위치 사용 금지 경고와는 무관.
 *
 * 이 파일은 "상태 흐름의 뼈대"이며, 표시된 TODO 항목들은 결정되는 대로 채워야 함.
 */
@Autonomous(name = "Leo Auto")
public class LeoAuto extends LinearOpMode {

    private enum State {
        SEARCH_ARTIFACT,
        APPROACH_ARTIFACT,
        AIM_AT_GOAL,
        SHOOT,
        PARK,
        DONE
    }

    private Alliance alliance = Alliance.BLUE; // 기본값, 대기 중 선택으로 덮어씀

    // 시작 전 대기 중 오벨리스크를 읽어서 저장. 현재는 로직에 반영 안 하고 텔레메트리
    // 확인 및 향후 패턴 전략 추가를 위한 준비 용도(2026-08-01, 패턴 전략 자체는 미정).
    private Vision.Motif targetMotif = null;
    private State state = State.SEARCH_ARTIFACT;

    // APPROACH_ARTIFACT 중 블롭 인식이 매 틱 불안정(카메라 FPS 낮음, 6~10FPS 확인됨,
    // 2026-08-12)해서 한 프레임 놓쳤다고 바로 SEARCH로 돌아가면 "가끔만 직진, 대부분
    // 회전"하는 증상이 발생함. 연속 몇 틱 이상 놓쳤을 때만 포기하도록 디바운스 카운터 추가.
    private int consecutiveBlobMissCount = 0;
    private static final int BLOB_MISS_TOLERANCE_TICKS = 8; // TODO: 실측 후 재조정

    private Drive drive;
    private Vision vision;
    private Intake intake;
    private Shooter shooter;
    private Localizer localizer;

    private final ElapsedTime stateTimer = new ElapsedTime();

    // TODO: 웹캠 이름, 카메라 장착 위치/각도 확정되면 교체
    private static final String WEBCAM_NAME = "Webcam 1";
    private static final Position CAMERA_POSITION =
            new Position(DistanceUnit.MM, 0, 0, 0, 0);
    private static final YawPitchRollAngles CAMERA_ORIENTATION =
            new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    // 필드 좌표계 (2026-08-01 확정): 필드 중앙이 원점(0,0), X축 오른쪽, Y축 안쪽(오디언스 반대),
    // heading 0도 = +X 방향, 반시계 방향이 양(+). 단위 mm.
    //
    // *** 중요: 아래 세 값은 지금 테스트하던 연습 장소 기준이며 실제 경기장 값이 아님! ***
    // *** 대회 당일 로봇을 실제 AUTO 시작 위치/방향에 놓은 직후, 반드시 이 세 값을 ***
    // *** 그 실제 배치에 맞게 다시 계산해서 수정하고 재빌드해야 함. ***
    // START_HEADING_DEG=90은 "로봇이 두 골대를 잇는 선분의 수선 방향(두 골대 중앙을 정면으로
    // 보는 방향)으로 시작한다"는 가정 하에 나온 값이며, 실제 경기장에서 로봇이 이 방향으로
    // 배치되지 않는다면 이 숫자도, START_X_MM/START_Y_MM도 전부 다시 계산해야 함.
    // 하드코딩 자체가 문제가 아니라 "이 장소, 이 배치에서만 맞는 값"이라는 점이 문제이므로,
    // 대회 시작 직전 로봇 배치를 확인한 뒤 여기 세 값만 수정 -> 재빌드 -> 업로드하는 절차가
    // 반드시 필요함 (2026-08-15).
    private static final double START_X_MM = 0;
    private static final double START_Y_MM = 0;
    private static final double START_HEADING_DEG = 90;

    // 골대 태그의 필드 좌표 및 방향. 공식 FIRST 필드 CAD STEP 파일
    // (ftc-resources.firstinspires.org/ftc/archive/2026/field/field-cad-step,
    // am-5700_Full.step, 2026-08-11 직접 다운로드 및 형상 분석으로 확인)에서
    // 골대 정면 패널(태그 부착면)의 정확한 중심 좌표와 법선 벡터를 추출한 실측값.
    // 이전의 타일 격자 기반 근사값(±1524, 1524)을 이 정밀값으로 교체함.
    //
    // 중요: 골대 정면은 필드 대각선(45도)과 평행하지 않음 - 법선 벡터 확인 결과
    // X축 기준 약 35.95도(레드), 대칭으로 144.05도(블루)로, 대각선과 약 9도 차이가 남.
    // 이 각도 차이 때문에 태그의 yaw(정면이 향하는 방향)를 고려해야 정확한 위치추정이
    // 가능함 - Localizer.update()가 현재 이 태그 방향을 반영하지 않고 있다면 오차 요인이 됨.
    private static final double GOAL_RED_FIELD_X_MM = 1383.69;
    private static final double GOAL_RED_FIELD_Y_MM = 1486.32;
    private static final double GOAL_RED_FIELD_NORMAL_DEG = 35.95; // 정면이 향하는 방향(X축 기준)

    private static final double GOAL_BLUE_FIELD_X_MM = -1382.11;
    private static final double GOAL_BLUE_FIELD_Y_MM = 1486.32;
    private static final double GOAL_BLUE_FIELD_NORMAL_DEG = 144.05;

    // 상태 전이/제어 임시 임계값 (전부 미검증 플레이스홀더, 실측 후 재조정 필요)
    private static final double SEARCH_TURN_POWER = 0.25;
    private static final double APPROACH_DRIVE_POWER = 0.35;
    private static final double GOAL_HEADING_TOLERANCE_DEG = 3.0;

    // 고정 슈팅 구역 판정: 태그까지 거리가 이 범위 안이면 "슈팅 구역 도달"로 간주.
    // TODO: 완전히 미검증인 플레이스홀더 값. 로봇/필드 실측 후 재조정 필수.
    // 이동 슈팅으로 확장할 때는 이 범위를 넓히거나(예: 0 ~ 매우 큰 값) 없애면 됨.
    private static final double SHOOT_SPOT_RANGE_MIN_MM = 800;
    private static final double SHOOT_SPOT_RANGE_MAX_MM = 1200;

    private static final double SHOOT_DURATION_SEC = 1.5; // TODO: Shooter 완성되면 발사 완료 판정으로 교체
    private static final double PARK_DRIVE_POWER = 0.4;

    // 발사 라인 Y좌표 (필드 중앙 원점 좌표계, mm). 발사 지역(골대 측) 깊이가 3타일이고
    // 필드가 6타일이므로 발사 지역 경계가 정확히 필드 중앙과 일치 → Y=0.
    // (2026-08-01: 청중측 2x1타일/골대측 6x3타일 확인, 3타일=필드 절반이라 계산상 0이 나옴)
    // TODO: 이건 "발사 지역 직사각형 경계"로 단순화한 근사치이며, 실제 발사선은 그림 2-2에서
    // 보듯 삼각형 형태 경계라 이 직선 근사가 완벽히 정확하진 않을 수 있음. 또한 동맹마다
    // Y+/Y- 어느 쪽이 발사 라인 안쪽인지도 좌표계 방향과 실제 로봇 시작 위치에 따라
    // 달라지므로 실측 검증 필요.
    private static final double LAUNCH_LINE_Y_MM = 0;

    // 발사 라인을 완전히 넘었다고 판단하기 위한 여유 마진(mm). 로봇 길이/EKF 불확실성을
    // 고려해 정확히 경계선이 아니라 좀 더 들어간 지점을 목표로 함. TODO: 실측 후 조정.
    private static final double PARK_SAFETY_MARGIN_MM = 300;

    // PARK 최대 대기 시간(안전장치). EKF가 이상하게 튀거나 태그를 못 잡아 판정이 안 되는
    // 경우에도 무한정 움직이지 않도록 하는 타임아웃. TODO: 실측 후 여유있게 조정.
    private static final double PARK_TIMEOUT_SEC = 3.0;

    @Override
    public void runOpMode() {
        drive = new Drive(hardwareMap);
        vision = new Vision(hardwareMap, WEBCAM_NAME, CAMERA_POSITION, CAMERA_ORIENTATION);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        localizer = new Localizer(START_X_MM, START_Y_MM, START_HEADING_DEG);

        // 동맹 선택 대기. 이 시간 동안 오벨리스크가 보이면 목표 모티프를 계속 갱신해서 저장.
        // (경기 시작 전 오벨리스크가 시야에 있을 가능성이 높은 시점 - 시작 후에는
        // 로봇이 움직이며 오벨리스크가 시야 밖으로 나갈 수 있음)
        while (!isStarted() && !isStopRequested()) {
            if (gamepad1.x && !gamepad1.b) alliance = Alliance.BLUE;
            else if (!gamepad1.x && gamepad1.b) alliance = Alliance.RED;

            updateTargetMotifIfDetected();

            telemetry.addData("Alliance (X=Blue, B=Red)", alliance);
            telemetry.addData("Target Motif", targetMotif != null ? targetMotif : "(감지 안 됨)");
            telemetry.update();
        }

        waitForStart();
        stateTimer.reset();

        while (opModeIsActive()) {
            localizer.predict(drive);
            updateLocalizerFromVision();
            updateTargetMotifIfDetected();

            runStateMachine();

            telemetry.addData("State", state);
            telemetry.addData("Target Motif", targetMotif != null ? targetMotif : "(감지 안 됨)");
            telemetry.addData("Heading", "%.1f", drive.getCurrentHeadingDeg());
            telemetry.addData("Localizer (x,y,theta)", "%.0f, %.0f, %.1f",
                    localizer.getX(), localizer.getY(), localizer.getHeadingDeg());
            telemetry.addData("Localizer uncertainty (x,y)", "%.1f, %.1f",
                    localizer.getUncertaintyX(), localizer.getUncertaintyY());
            telemetry.update();

            sleep(20);
        }

        drive.stop();
        vision.close();
    }

    private void runStateMachine() {
        switch (state) {
            case SEARCH_ARTIFACT:
                handleSearchArtifact();
                break;
            case APPROACH_ARTIFACT:
                handleApproachArtifact();
                break;
            case AIM_AT_GOAL:
                handleAimAtGoal();
                break;
            case SHOOT:
                handleShoot();
                break;
            case PARK:
                handlePark();
                break;
            case DONE:
                drive.stop();
                break;
        }
    }

    private void transitionTo(State next) {
        state = next;
        stateTimer.reset();
    }

    /**
     * 시야에 들어온 골대 태그로 Localizer를 업데이트한다. 매 틱 호출.
     * 오벨리스크는 위치추정에 사용하지 않음(PDF 명시 경고 - "정확한 배치 위치는 시합에
     * 따라 다를 수 있으므로 로봇 주행에 사용하지 않는 것이 좋음", 2026-08-01 확인).
     * 골대 태그만 사용.
     */
    private void updateLocalizerFromVision() {
        AprilTagDetection goalTag = vision.getGoalTagDetection(alliance);
        if (goalTag == null) return;

        double tagFieldX = (alliance == Alliance.RED) ? GOAL_RED_FIELD_X_MM : GOAL_BLUE_FIELD_X_MM;
        double tagFieldY = (alliance == Alliance.RED) ? GOAL_RED_FIELD_Y_MM : GOAL_BLUE_FIELD_Y_MM;

        localizer.update(goalTag, tagFieldX, tagFieldY);
    }

    /**
     * 오벨리스크 태그가 보이면 목표 모티프를 갱신한다. 매 틱(대기 중 + AUTO 실행 중 모두) 호출.
     * 한 번 감지되면(targetMotif != null) 그 이후 안 보여도 값을 유지한다 - 로봇 시작 위치에
     * 따라 대기 중에는 오벨리스크가 안 보이다가 AUTO 시작 후 이동하며 뒤늦게 보일 수 있고
     * (그림 4-1 확인, 시작 위치가 여러 곳), 반대로 한 번 읽은 후 이동하며 시야에서 벗어날
     * 수도 있으므로 - 두 경우 모두 마지막으로 확인된 값을 계속 신뢰하는 것이 안전함.
     */
    private void updateTargetMotifIfDetected() {
        Vision.Motif detectedMotif = vision.getTargetMotif();
        if (detectedMotif != null) {
            targetMotif = detectedMotif;
        }
    }

    // =========================================================
    //  SEARCH_ARTIFACT: 제자리 회전하며 아티팩트 블롭 탐색
    // =========================================================
    private void handleSearchArtifact() {
        ColorBlobLocatorProcessor.Blob blob = vision.getLargestArtifactBlobAnyColor();

        if (blob != null) {
            drive.stop();
            transitionTo(State.APPROACH_ARTIFACT);
            return;
        }

        // TODO: 회전 방향/범위가 미정. 지금은 한 방향으로만 계속 회전.
        drive.setPowerRaw(-SEARCH_TURN_POWER, SEARCH_TURN_POWER);
    }

    // =========================================================
    //  APPROACH_ARTIFACT: 블롭 방향으로 헤딩 정렬 + 전진, 이동 중 자동 흡입
    //  흡입 완료 여부는 카메라(Vision)가 아니라 Intake(색+근접 센서)로 판단.
    //  인테이크 모터(motor3)는 이 상태 진입 시 켜고, 흡입 완료 시 끈다.
    // =========================================================
    // 블롭을 놓친 틱 동안에도 마지막으로 알려진 방향으로 계속 진행하기 위한 기억값
    private double lastKnownBlobX = 0.0;

    // AIM_AT_GOAL 중 태그가 보일 때마다 갱신되는 IMU 기준 목표 방향. 카메라의 순간 bearing이
    // 아니라 이 고정된 목표를 IMU로 추적하며 이동하므로, 태그 인식이 프레임마다 흔들려도
    // 조향 자체는 안정적으로 유지된다. (2026-08-15: 매 틱 bearing으로 직접 조향하던 방식은
    // 인식 불안정 시 로봇이 지그재그로 움직이는 문제가 있어 이 방식으로 교체)
    private Double targetHeadingLock = null;
    private int consecutiveTagMissCount = 0;
    private static final int TAG_MISS_TOLERANCE_TICKS = 8;

    private void handleApproachArtifact() {
        intake.start();

        if (intake.isArtifactLoaded()) {
            intake.stop();
            drive.stop();
            transitionTo(State.AIM_AT_GOAL);
            return;
        }

        ColorBlobLocatorProcessor.Blob blob = vision.getLargestArtifactBlobAnyColor();
        double blobX;

        if (blob == null) {
            consecutiveBlobMissCount++;
            if (consecutiveBlobMissCount > BLOB_MISS_TOLERANCE_TICKS) {
                // 연속으로 오래 놓쳤을 때만 진짜로 다시 탐색
                consecutiveBlobMissCount = 0;
                transitionTo(State.SEARCH_ARTIFACT);
                return;
            }
            // 짧게 놓친 것뿐이면 마지막으로 알려진 방향을 유지하며 계속 진행
            blobX = lastKnownBlobX;
        } else {
            consecutiveBlobMissCount = 0;
            blobX = vision.getBlobNormalizedX(blob);
            lastKnownBlobX = blobX;
        }

        // 간단한 비례 조향: 블롭이 화면 중앙에서 벗어난 만큼 회전 보정하며 전진
        // TODO: 미검증 임시 로직. 실제 로봇에서 게인/파워값 재조정 필요.
        double turnCorrection = blobX * 0.3;
        double leftPower = APPROACH_DRIVE_POWER + turnCorrection;
        double rightPower = APPROACH_DRIVE_POWER - turnCorrection;

        drive.setPowerRaw(leftPower, rightPower);
    }

    // =========================================================
    //  AIM_AT_GOAL: 슈팅 구역으로 이동 + AprilTag(골대)로 헤딩/거리 정렬
    //
    //  전략(2026-08-01 확정): 고정 슈팅으로 시작. 슈팅 정확도 자체가 아직 미검증이라
    //  이동 슈팅의 복잡도까지 얹지 않기로 함. 다만 나중에 이동 슈팅으로 확장하기 쉽도록,
    //  "슈팅 구역"을 좌표가 아니라 AprilTag 거리 범위(SHOOT_SPOT_RANGE_*)로 정의함.
    //
    //  2026-08-15 재설계: 매 틱 카메라의 순간 bearing으로 직접 조향하던 방식은 (1) 태그
    //  인식이 프레임마다 불안정해서 로봇이 지그재그/점사식으로 움직이는 문제, (2) 목표
    //  각도 근처에서 회전을 멈추는 조건이 없어 목표를 지나쳐도 계속 도는 문제가 있었음.
    //  이를 해결하기 위해 태그가 보일 때마다 "IMU 기준 목표 방향(targetHeadingLock)"을
    //  한 번 계산해서 고정하고, 그 이후에는 IMU가 이 고정된 목표를 얼마나 벗어났는지로
    //  조향한다. 카메라 인식이 흔들려도 조향 자체는 안정적인 IMU 기반이라 직진성이 좋아짐.
    //  태그를 짧게(8틱 이내) 놓쳐도 마지막 targetHeadingLock을 계속 추적하며 이동한다.
    //
    //  range 단위 주의: goalTag.ftcPose.range는 FTC SDK 기준 inch 단위로 반환됨(공식
    //  문서 확인, 2026-08-15). SHOOT_SPOT_RANGE_MIN/MAX_MM(mm 기준)과 비교하려면
    //  반드시 25.4를 곱해 mm로 변환해야 함 - 이걸 누락하면 실제 거리를 inch 그대로
    //  mm처럼 취급해서 "이미 목표보다 가깝다"고 오판, 계속 후진하는 버그가 발생했었음.
    // =========================================================
    private void handleAimAtGoal() {
        AprilTagDetection goalTag = vision.getGoalTagDetection(alliance);

        if (goalTag != null && goalTag.ftcPose != null) {
            consecutiveTagMissCount = 0;

            double bearingError = goalTag.ftcPose.bearing; // deg, 0이면 카메라 중앙
            double range = goalTag.ftcPose.range * 25.4; // inch -> mm

            targetHeadingLock = normalizeAngle(drive.getCurrentHeadingDeg() + bearingError);

            boolean inShootSpotRange = range >= SHOOT_SPOT_RANGE_MIN_MM && range <= SHOOT_SPOT_RANGE_MAX_MM;
            boolean headingAligned = Math.abs(bearingError) < GOAL_HEADING_TOLERANCE_DEG;

            if (inShootSpotRange && headingAligned) {
                drive.stop();
                transitionTo(State.SHOOT);
                return;
            }

            driveTowardLockedHeading(range);
            return;
        }

        consecutiveTagMissCount++;

        if (targetHeadingLock != null && consecutiveTagMissCount <= TAG_MISS_TOLERANCE_TICKS) {
            // 짧게 놓친 경우, 마지막으로 잡은 목표 방향을 IMU로 계속 추적
            driveTowardLockedHeading(-1);
            return;
        }

        // 태그를 한 번도 못 잡았거나 오래 놓쳤으면, EKF 추정 위치 기반으로 목표 방향 탐색
        // TODO: 로봇 시작 위치/heading(START_X_MM/Y_MM/HEADING_DEG)이 실제 배치와
        // 얼마나 정확히 일치하는지에 따라 이 추정의 신뢰도가 좌우됨.
        double goalFieldX = (alliance == Alliance.RED) ? GOAL_RED_FIELD_X_MM : GOAL_BLUE_FIELD_X_MM;
        double goalFieldY = (alliance == Alliance.RED) ? GOAL_RED_FIELD_Y_MM : GOAL_BLUE_FIELD_Y_MM;

        double dx = goalFieldX - localizer.getX();
        double dy = goalFieldY - localizer.getY();
        double desiredHeading = Math.toDegrees(Math.atan2(dy, dx));
        double headingError = normalizeAngle(desiredHeading - localizer.getHeadingDeg());

        if (Math.abs(headingError) < GOAL_HEADING_TOLERANCE_DEG) {
            drive.stop();
        } else {
            double turnDirection = (headingError >= 0) ? 1.0 : -1.0;
            drive.setPowerRaw(-turnDirection * SEARCH_TURN_POWER, turnDirection * SEARCH_TURN_POWER);
        }
    }

    /**
     * targetHeadingLock을 향해 IMU 기반으로 회전하며, rangeMM이 주어지면 거리에 따라
     * 전진/후진도 함께 수행. rangeMM이 음수면(태그를 놓친 상태) 방향만 유지하고 전진은 안 함.
     */
    private void driveTowardLockedHeading(double rangeMM) {
        double imuHeadingError = normalizeAngle(targetHeadingLock - drive.getCurrentHeadingDeg());
        double turnCorrection = clamp(imuHeadingError * 0.03, -0.3, 0.3);

        if (rangeMM < 0) {
            drive.setPowerRaw(-turnCorrection, turnCorrection);
            return;
        }

        boolean inShootSpotRange = rangeMM >= SHOOT_SPOT_RANGE_MIN_MM && rangeMM <= SHOOT_SPOT_RANGE_MAX_MM;
        double driveDirection = inShootSpotRange ? 0.0 : ((rangeMM > SHOOT_SPOT_RANGE_MAX_MM) ? 1.0 : -1.0);

        drive.setPowerRaw(
                driveDirection * APPROACH_DRIVE_POWER - turnCorrection,
                driveDirection * APPROACH_DRIVE_POWER + turnCorrection);
    }

    // =========================================================
    //  SHOOT: 슈팅 메커니즘 발사
    //  TODO: 거리→파워/RPM 룩업테이블 미구현 (Shooter.setPowerForDistance는 임시 고정값).
    //  발사 완료 판정도 아직 없어 일정 시간 대기로만 상태를 넘기는 플레이스홀더.
    // =========================================================
    private void handleShoot() {
        drive.stop();

        double range = vision.getRangeToTag(
                alliance == Alliance.RED ? Vision.TAG_ID_GOAL_RED : Vision.TAG_ID_GOAL_BLUE);
        shooter.setPowerForDistance(range);

        if (stateTimer.seconds() > SHOOT_DURATION_SEC) {
            shooter.stop();
            transitionTo(State.PARK);
        }
    }

    // =========================================================
    //  PARK: 발사 라인 밖으로 이동 (G206 리더십 점수 확보)
    //
    //  Localizer(EKF)의 추정 Y좌표 기반으로 발사 라인 통과 여부를 판단.
    //  AprilTag(Vision)의 순간 거리값에 직접 의존하지 않는 이유(2026-08-01 논의):
    //  Vision은 항상 안정적으로 잡힌다는 보장이 없음(각도/거리/조명 등에 따라 실패 가능).
    //  EKF는 AprilTag가 안 보여도 predict(encoder+IMU)만으로 계속 위치를 추정하므로,
    //  최악의 경우(AprilTag 전혀 안 잡힘)에도 최소한 encoder 기반 추정치는 확보되고,
    //  잡힐 때마다 그 추정치가 보정되어 더 정확해짐 - Vision 순간값보다 훨씬 안정적.
    //
    //  TODO: LAUNCH_LINE_Y_MM은 발사 지역을 직사각형으로 단순화한 근사치라 실제
    //  삼각형 경계와 정확히 일치하지 않을 수 있음. 실측 검증 필요.
    // =========================================================
    private void handlePark() {
        double currentY = localizer.getY();
        boolean pastLaunchLine = currentY < (LAUNCH_LINE_Y_MM - PARK_SAFETY_MARGIN_MM);

        if (pastLaunchLine || stateTimer.seconds() > PARK_TIMEOUT_SEC) {
            drive.stop();
            transitionTo(State.DONE);
            return;
        }

        // 골대 쪽(Y가 큰 방향)에서 슈팅했으므로 발사 라인을 넘으려면 -Y 방향(청중 쪽)으로 이동.
        drive.setPowerRaw(-PARK_DRIVE_POWER, -PARK_DRIVE_POWER);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static double normalizeAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }
}