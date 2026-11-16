package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.records.Alliance;

/**
 * Registers each BIOBUZZ routine once per alliance. Preselects TeleOp to follow AUTO. Only Up, Down
 * and Park Only are listed on the Driver Station; CELL A and CELL B are kept but {@code @Disabled}.
 */
public final class UnifiedAutos {

  private UnifiedAutos() {}

  @Disabled
  @Autonomous(name = "Red A: tip first", group = "Autonomous", preselectTeleOp = "TeleOp")
  public static class RedCellA extends CellAAuto {
    public RedCellA() {
      super(Alliance.RED);
    }
  }

  @Disabled
  @Autonomous(name = "Blue A: tip first", group = "Autonomous", preselectTeleOp = "TeleOp")
  public static class BlueCellA extends CellAAuto {
    public BlueCellA() {
      super(Alliance.BLUE);
    }
  }

  @Disabled
  @Autonomous(name = "Red B: second CELL", group = "Autonomous", preselectTeleOp = "TeleOp")
  public static class RedCellB extends CellBAuto {
    public RedCellB() {
      super(Alliance.RED);
    }
  }

  @Disabled
  @Autonomous(name = "Blue B: second CELL", group = "Autonomous", preselectTeleOp = "TeleOp")
  public static class BlueCellB extends CellBAuto {
    public BlueCellB() {
      super(Alliance.BLUE);
    }
  }

  @Autonomous(name = "Red Park Only", group = "Autonomous", preselectTeleOp = "TeleOp")
  public static class RedParkOnly extends ParkOnlyAuto {
    public RedParkOnly() {
      super(Alliance.RED);
    }
  }

  @Autonomous(name = "Blue Park Only", group = "Autonomous", preselectTeleOp = "TeleOp")
  public static class BlueParkOnly extends ParkOnlyAuto {
    public BlueParkOnly() {
      super(Alliance.BLUE);
    }
  }

  @Autonomous(name = "Red Up", group = "Autonomous", preselectTeleOp = "TeleOp")
  public static class RedUp extends UpAuto {
    public RedUp() {
      super(Alliance.RED);
    }
  }

  @Autonomous(name = "Blue Up", group = "Autonomous", preselectTeleOp = "TeleOp")
  public static class BlueUp extends UpAuto {
    public BlueUp() {
      super(Alliance.BLUE);
    }
  }

  @Autonomous(name = "Red Down", group = "Autonomous", preselectTeleOp = "TeleOp")
  public static class RedDown extends DownAuto {
    public RedDown() {
      super(Alliance.RED);
    }
  }

  @Autonomous(name = "Blue Down", group = "Autonomous", preselectTeleOp = "TeleOp")
  public static class BlueDown extends DownAuto {
    public BlueDown() {
      super(Alliance.BLUE);
    }
  }
}
