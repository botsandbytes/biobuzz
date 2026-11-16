package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.Nullable;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Locale;
import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.teamcode.robot.config.generated.config;
import org.firstinspires.ftc.teamcode.utilities.BallInventory;

/**
 * Intake motor, transfer servo, and G407 four-element inventory.
 *
 * <p>Single writer: commands, {@link ShotController} and OpModes set a target with {@link #run},
 * {@link #feed}, {@link #eject} or {@link #stop}; only {@link #periodic()} writes actuator power.
 * That is where the inventory cap is applied, so no caller can drive a fifth SCORING ELEMENT in by
 * accident.
 */
public class Intake {

  /** What the rollers are doing this loop. FEED and EJECT ignore the inventory cap. */
  public enum Mode {
    STOPPED,
    COLLECT,
    FEED,
    EJECT
  }

  private final DcMotorEx rightIntake;
  private final CRServo intakeTransfer;
  private final DoubleSupplier shooterTargetPower;
  private final @Nullable DigitalChannel entrySensor;
  private final BallInventory inventory;

  private Mode mode = Mode.STOPPED;
  private double targetPower = 0.0;
  private boolean blockedByCapacity = false;
  private boolean transferStopped = false;
  private boolean shotTransferGated = false;

  public Intake(HardwareMap hardwareMap, DoubleSupplier shooterTargetPower) {
    this.shooterTargetPower = shooterTargetPower;
    rightIntake = hardwareMap.get(DcMotorEx.class, "rightIntake");
    intakeTransfer = hardwareMap.get(CRServo.class, config.intake.transfer_servo_name);

    rightIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);

    var inv = config.inventory;
    DigitalChannel sensor =
        inv.entry_sensor_enabled
            ? hardwareMap.tryGet(DigitalChannel.class, inv.entry_sensor_name)
            : null;
    if (sensor != null) {
      sensor.setMode(DigitalChannel.Mode.INPUT);
    }
    entrySensor = sensor;
    inventory = new BallInventory(inv.capacity_count, sensor != null);
  }

  /** Collect from the floor or a FLOWER: stops itself once the inventory is full. */
  public void run(double power) {
    setTarget(Mode.COLLECT, power);
  }

  /** Push SCORING ELEMENTS into the shooter. Not capped: feeding only ever empties the robot. */
  public void feed(double power) {
    setTarget(Mode.FEED, power);
  }

  /** Reverse the intake roller to spit out a fifth element (G407) or opponent NECTAR (G408). */
  public void eject(double power) {
    double reverse = -Math.abs(power);
    setTarget(Mode.EJECT, reverse);
  }

  /** Stop the intake motor and close the transfer's feed gate during a shot. */
  public void stop() {
    setTarget(Mode.STOPPED, 0.0);
  }

  /**
   * Stop only the transfer, leaving intake and shooter targets unchanged. Latched until
   * resumeTransfer().
   */
  public void stopTransfer() {
    transferStopped = true;
  }

  /** Clear the operator's transfer stop. */
  public void resumeTransfer() {
    transferStopped = false;
  }

  /** During a shot, use the feed target resolved by ShotController for both actuators. */
  void setShotTransferGated(boolean gated) {
    shotTransferGated = gated;
  }

  public String getTransferStatus() {
    return String.format(
        Locale.ROOT,
        "%s; command %.2f; port %d",
        transferStopped
            ? "Stopped by operator/path"
            : mode == Mode.COLLECT && !blockedByCapacity
                ? "Stopped while collecting"
                : (shotTransferGated ? "Following shot feed" : "Following shooter"),
        intakeTransfer.getPower(),
        intakeTransfer.getPortNumber());
  }

  private void setTarget(Mode newMode, double power) {
    mode = newMode;
    targetPower = power;
  }

  public Mode mode() {
    return mode;
  }

  /** Read the actual intake controller, independently of the bridge's hub-index mapping. */
  public String getMotorStatus() {
    return String.format(
        Locale.ROOT,
        "%s; %.0f ticks/s; command %.2f; port %d",
        mode,
        rightIntake.getVelocity(),
        rightIntake.getPower(),
        rightIntake.getPortNumber());
  }

  public boolean isRunning() {
    return mode != Mode.STOPPED;
  }

  /** True when the last {@link #periodic()} held the intake off because the robot was full. */
  public boolean isBlockedByCapacity() {
    return blockedByCapacity;
  }

  public BallInventory inventory() {
    return inventory;
  }

  /** G407: the proven predicate {@link BallInventory#mayCollect}; unknown never blocks. */
  private boolean mayCollect() {
    return BallInventory.mayCollect(inventory.isKnown(), inventory.count(), inventory.capacity());
  }

  /** The only place intake motor/servo power is written. Called once per loop by Robot.update(). */
  public void periodic() {
    if (entrySensor != null) {
      boolean raw = entrySensor.getState();
      boolean blocked = config.inventory.entry_sensor_active_low ? !raw : raw;
      inventory.updateEntrySensor(
          blocked,
          System.nanoTime() / 1_000_000L,
          config.inventory.entry_debounce_ms,
          mode == Mode.EJECT);
    } else if ((mode == Mode.COLLECT && mayCollect()) || mode == Mode.EJECT) {
      inventory.markUnknown();
    }

    blockedByCapacity = mode == Mode.COLLECT && !mayCollect();
    boolean powered = mode != Mode.STOPPED && !blockedByCapacity;
    rightIntake.setPower(powered ? targetPower : 0.0);
    intakeTransfer.setDirection(
        config.intake.transfer_reversed
            ? DcMotorSimple.Direction.REVERSE
            : DcMotorSimple.Direction.FORWARD);
    double shooterPower = shooterTargetPower.getAsDouble();
    // The transfer never runs while the intake is collecting (a full inventory holds the intake
    // off,
    // so the transfer follows the shooter again).
    boolean collecting = mode == Mode.COLLECT && !blockedByCapacity;
    boolean transferRunning =
        !transferStopped
            && !collecting
            && Double.isFinite(shooterPower)
            && shooterPower > 0.0
            && (!shotTransferGated || mode == Mode.FEED);
    intakeTransfer.setPower(transferRunning ? config.intake.transfer_power : 0.0);
  }

  public Command runIntakeCommand(double power) {
    return Command.build().setStart(() -> run(power)).setEnd(interrupted -> stop()).requiring(this);
  }

  public Command ejectCommand(double power) {
    return Command.build()
        .setStart(() -> eject(power))
        .setEnd(interrupted -> stop())
        .requiring(this);
  }

  public Command stopIntakeCommand() {
    return Command.build().setStart(this::stop).setDone(() -> true).requiring(this);
  }
}
