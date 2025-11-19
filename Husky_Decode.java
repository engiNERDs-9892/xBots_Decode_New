// Copyright (c) 2024-2025 FTC 13532
// All rights reserved.

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * This Class uses the DFRobot HuskyLens.
 *
 * The HuskyLens is a Vision Sensor with a built-in object detection model.  It can
 * detect a number of predefined objects and AprilTags in the 36h11 family, Object Colors
 *
 * HuskyLens is configured for a name of "huskylens".
 * camera image size is 320 by 240
 */

public class Husky_Decode {

  private final int READ_PERIOD = 1;

  private HuskyLens huskyLens;

  // Default constructor
  public Husky_Decode(LinearOpMode opmode) {
    hskOp = opmode;
  }

  /* Declare OpMode members.
   * gain access to methods in the calling OpMode.
   */
  private LinearOpMode hskOp = null;
  /*
   * you may set the rate Limit. Rate limits the reads solely to allow a user time to observe
   * what is happening on the Driver Station telemetry.
   * Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
   *  Typical applications would not likely rate limit.
   */
  /*
   * color - object color recognition
   * april - april tag recognition
   */
  private String HuskLensAlgorithm = "color";

  // Block variables
  private int colorId = 0; // object color ID
  private int AprilId = 0; // april tag ID
  private int height = 0; // Box height
  private int width = 0; // Box Width
  private int left = 0; // Box Left edge
  private int top = 0; // Box Top edge
  private int x = 0; // x position in image
  private int y = 0; // y position in image
  // last value
  private int LcolorId = 0; // last object color ID
  private int LAprilId = 0; // last april tag ID
  private int Lheight = 0; // last Box height
  private int Lwidth = 0; // last Box Width
  private int Lleft = 0; // last Box Left edge
  private int Ltop = 0; // last Box Top edge
  private int Lx = 0; // last x position in image
  private int Ly = 0; // last y position in image

  public void initHuskyLens() {
    huskyLens = hskOp.hardwareMap.get(HuskyLens.class, "huskylens");

    /*
     * Immediately expire so that the first time through we'll do the read.
     */
    // .expire();

    /*
     * Basic check to see if the device is alive and communicating.  This is not
     * technically necessary here as the HuskyLens class does this in its
     * doInitialization() method which is called when the device is pulled out of
     * the hardware map.  However, sometimes it's unclear why a device reports as
     * failing on initialization.  In the case of this device, it's because the
     * call to knock() failed.
     */
    if (!huskyLens.knock()) {
      hskOp.telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
    } else {
      hskOp.telemetry.addData(">>", "Husky Lens camera initiated");
    }
    setBlocks();
    hskOp.telemetry.update();
  }

  public void setAlgorithm(String algor) {
    if (algor == "color" || algor == "Color") HuskLensAlgorithm = "color";
    if (algor == "april" || algor == "April") HuskLensAlgorithm = "april";
  }

  // get the blocks from camera  and sets variables
  public void setBlocks() {
    /*
     * The device uses the concept of an algorithm to determine what types of
     * objects it will look for and/or what mode it is in.
     * Other algorithm choices for FTC might be: OBJECT_RECOGNITION, COLOR_RECOGNITION or OBJECT_CLASSIFICATION.
     */
    if (HuskLensAlgorithm == "april")
      huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    if (HuskLensAlgorithm == "color")
      huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION);
    // save block values
    Lheight = height; // Box height
    Lwidth = width; // Box Width
    Lleft = left; // Box Left edge
    Ltop = top; // Box Top edge
    Lx = x; // x position in image
    Ly = y; // y position in image

    /*
     * All algorithms, except for LINE_TRACKING, return a list of Blocks where aBlock represents the outline of a recognized object along with its ID number.
     * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
     *
     *  referenced in the header comment above for more information on IDs and how to
     * assign them to objects.
     *
     * Returns an empty array if no objects are seen.
     */
    HuskyLens.Block[] blocks = huskyLens.blocks();
    // "Block count", blocks.length);
    for (int i = 0; i < blocks.length; i++) {

      if (HuskLensAlgorithm == "color") colorId = blocks[i].id; // object color ID
      if (HuskLensAlgorithm == "april") AprilId = blocks[i].id; // April ID
      height = blocks[i].height; // Box height
      width = blocks[i].width; // Box Width
      left = blocks[i].left; // Box Left edge
      top = blocks[i].top; // Box Top edge
      x = blocks[i].x; // x position in image
      y = blocks[i].y; // y position in image
    } // end blocks  for loop
  } // end setBlocks

  // getters
  public int getColorId() {
    return colorId;
  }

  public int getAprilId() {
    return AprilId;
  }

  public int getHeightId() {
    return height;
  }

  public int getWidth() {
    return width;
  }

  public int getLeft() {
    return left;
  }

  public int getTop() {
    return top;
  }

  public int getX() {
    return x;
  }

  public int getY() {
    return y;
  }

  // movement calculations
  // Size position in image returned as area max 76800
  public int size() {
    return height * width;
  }

  // Top 0 bottom 240
  public double topPos() {
    int topRatio = top / 240;
    return topRatio;
  }

  // Left 0 right 320 return value -1 to 1 from center bottom
  public double leftPos() {
    int leftRatio = left / 320;
    return leftRatio;
  }

  // move left or right

  public double leftRight() {
    return map(leftPos());
  }

  public double fwrdBack() {
    return map(topPos());
  }

  private double map(double value) {
    /* simplified -1 * value * 2 from following:
    Output value = outputStart +
     ((value - inputStart) / (inputEnd - inputStart))
     * (outputEnd - outputStart) */
    return -1 * value * 2;
  }
} // end Husky decode class
