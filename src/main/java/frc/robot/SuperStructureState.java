// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

// L1 13
// L2 42.5

// Algea L1
public class SuperStructureState {

  public static final double SOURCE_HEIGHT = 0.25;
  public static final double SOURCE_ANGLE = 50;

  public static final double L0_HEIGHT = 0.1;
  public static final double L1_HEIGHT = 23.35; // L2 Coral
  public static final double L2_HEIGHT = 51.13; // L2 Algea
  public static final double L3_HEIGHT = 0; // L3 Coral
  public static final double L4_HEIGHT = 0; // L3 Algea
  public static final double HUMAN_HEIGHT = 15.8; // Right //Transfer
  public static final double L1B_HEIGHT = 30; // L1 Outake
  public static final double L2B_HEIGHT = 52; // L2 Outake

  // intake 35
  public static final double L0_ANGLE = -45;
  public static final double L1_ANGLE = -30;
  public static final double L2_ANGLE = 10;
  public static final double L3_ANGLE = -30;
  public static final double L4_ANGLE = 10;
  public static final double HUMAN_ANGLE = -100;
  public static final double L1B_ANGLE = -100;
  public static final double L2B_ANGLE = -100;

  public static final double PROCESSOR_HEIGHT = 5;
  public static final double LOW_ALGAE_HEIGHT = 25;
  public static final double MID_ALGAE_HEIGHT = 45;
  public static final double TOP_ALGAE_HEIGHT = 81.5;

  public static final double PROCESSOR_ANGLE = 225;
  public static final double LOW_ALGAE_ANGLE = 210;
  public static final double MID_ALGAE_ANGLE = 210;
  public static final double TOP_ALGAE_ANGLE = 105;

  // angle to pass the safty zone
  public static SuperStructureState STATE_SAFTY = new SuperStructureState("Safety", 180, 75);
  public static SuperStructureState STATE_SOURCE =
      new SuperStructureState("Source", SOURCE_HEIGHT, SOURCE_ANGLE);
  public static SuperStructureState STATE_L0 = new SuperStructureState("CL0", L0_HEIGHT, L0_ANGLE);
  public static SuperStructureState STATE_L1 = new SuperStructureState("CL1", L1_HEIGHT, L1_ANGLE);
  public static SuperStructureState STATE_L2 = new SuperStructureState("CL2", L2_HEIGHT, L2_ANGLE);
  public static SuperStructureState STATE_L3 = new SuperStructureState("CL3", L3_HEIGHT, L3_ANGLE);
  public static SuperStructureState STATE_L4 = new SuperStructureState("CL4", L4_HEIGHT, L4_ANGLE);
  public static SuperStructureState STATE_HUMAN =
      new SuperStructureState("CLT", HUMAN_HEIGHT, HUMAN_ANGLE);
  public static SuperStructureState STATE_L1B =
      new SuperStructureState("CL1B", L1B_HEIGHT, L1B_ANGLE);
  public static SuperStructureState STATE_L2B =
      new SuperStructureState("CL2B", L2B_HEIGHT, L2B_ANGLE);

  public static SuperStructureState STATE_PROCESSOR =
      new SuperStructureState("PROCESSOR", PROCESSOR_HEIGHT, PROCESSOR_ANGLE);
  public static SuperStructureState STATE_ALGAE_LOW =
      new SuperStructureState("AL2", LOW_ALGAE_HEIGHT, LOW_ALGAE_ANGLE);
  public static SuperStructureState STATE_ALGAE_MID =
      new SuperStructureState("AL3", MID_ALGAE_HEIGHT, MID_ALGAE_ANGLE);
  public static SuperStructureState STATE_ALGAE_TOP =
      new SuperStructureState("AL4", TOP_ALGAE_HEIGHT, TOP_ALGAE_ANGLE);

  public double height;
  public double angle;
  public String name;

  public SuperStructureState(String name, double height, double angle) {
    this.name = name;
    this.height = height;
    this.angle = angle;
  }
}
