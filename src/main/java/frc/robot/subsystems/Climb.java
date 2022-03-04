// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Date;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

class State {
  static State current;
  static State extend1G;
  static State driveForward1;
  static State retract1G;
  static State tiltForw1T;
  static State extend2G;
  static State pivot2B;
  static State extend3A;
  static State pivot3B;
  static State retract2A;
  static State disengage1B;
  static State pivot4B;
  static State retract3A;
  static State pivot5B;
  static State extend4A;
  static State stop;
  static Climber climber;

  static long startTime;
  static long elapsedTime;

  public State() {
    climber = Climber.get_instance();
  }

  void getclassName() {
    System.out.println(this.getClass().getSimpleName());
  }

  void enter() {
  }

  void update() {
  }
}

class extend1G extends State {
  void enter() {
    startTime = new Date().getTime();
    climber.zeroEncoder();
    climber.set(.2);
    getclassName();
  }

  void update() {
    while (true) {
     // System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      //System.out.println(_i.getEncoderPos());
      if (climber.getEncoderPos() > 120 ) {
        current = driveForward1;
        return;
      }
      if (!climber.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class driveForward1 extends State {
  void enter() {
    startTime = new Date().getTime();
    getclassName();
    climber.zeroEncoder();
    climber.set(-.2);

  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (elapsedTime > 3000) {
        current = retract1G;
        return;
      }
      if (!climber.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class retract1G extends State {
  void enter() {
    startTime = new Date().getTime();
    climber.set(.2);
    climber.zeroEncoder();
    getclassName();
  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (climber.getEncoderPos() > 120 ) {
        current = tiltForw1T;
        return;
      }
      if (!climber.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class tiltForw1T extends State {
  void enter() {
    startTime = new Date().getTime();
    climber.zeroEncoder();
    climber.set(-.2);
    getclassName();
  }

  void update() {
    while (true) {
      elapsedTime = new Date().getTime() - startTime;
      //System.out.println(_i.getEncoderPos());
      if (elapsedTime > 3000) {
        current = extend2G;
        return;
      }
      if (!climber.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class extend2G extends State {
  void enter() {

    startTime = new Date().getTime();
    climber.zeroEncoder();
    getclassName();
    
    climber.set(.2);

  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (climber.getEncoderPos() > 120 ) {

        current = pivot2B;
        return;
      }
      if (!climber.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class pivot2B extends State {
  void enter() {
    startTime = new Date().getTime();
    climber.zeroEncoder();
    getclassName();
    climber.set(-.2);

  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (elapsedTime > 3000) {
        current = extend3A;
        return;
      }
      if (!climber.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class extend3A extends State {
  void enter() {
    startTime = new Date().getTime();
    climber.zeroEncoder();
    getclassName();
    climber.set(.2);

  }

  void update() {
    while (true) {
      elapsedTime = new Date().getTime() - startTime;
     // System.out.println(_i.getEncoderPos());
      if (climber.getEncoderPos() > 120) {

        current = pivot3B;
        return;
      }
      if (!climber.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class pivot3B extends State {
  void enter() {
    startTime = new Date().getTime();
    climber.zeroEncoder();
    getclassName();
    climber.set(-.2);

  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (elapsedTime > 3000) {
        current = retract2A;
        return;
      }
      if (!climber.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class retract2A extends State {
  void enter() {
    //System.out.println(_i.getEncoderPos());
    startTime = new Date().getTime();
    climber.zeroEncoder();
    getclassName();
    climber.set(.2);

  }

  void update() {
    while (true) {
      elapsedTime = new Date().getTime() - startTime;
      if (climber.getEncoderPos() > 120 ) {
        current = disengage1B;
        return;
      }
      if (!climber.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class disengage1B extends State {
  void enter() {
    startTime = new Date().getTime();
    climber.zeroEncoder();
    getclassName();
    climber.set(-.2);

  }

  void update() {
    while (true) {
      elapsedTime = new Date().getTime() - startTime;
     // System.out.println(_i.getEncoderPos());
      if (elapsedTime > 3000) {
        current = pivot4B;
        return;
      }
      if (!climber.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class pivot4B extends State {
  void enter() {
    startTime = new Date().getTime();
    climber.zeroEncoder();
    climber.set(.2);
    getclassName();
  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (climber.getEncoderPos() > 120 ) {
        current = retract3A;
        return;
      }
      if (!climber.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class retract3A extends State {
  void enter() {
    startTime = new Date().getTime();
    climber.zeroEncoder();
    getclassName();
    climber.set(-.2);

  }

  void update() {
    while (true) {
     // System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (elapsedTime > 3000) {
        current = pivot5B;
        return;
      }
      if (!climber.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class pivot5B extends State {
  void enter() {
    startTime = new Date().getTime();
    climber.zeroEncoder();
    getclassName();
    climber.set(.2);

  }

  void update() {
    while (true) {
      elapsedTime = new Date().getTime() - startTime;
      //System.out.println(_i.getEncoderPos());
      if (climber.getEncoderPos() > 120) {
        current = extend4A;
        return;
      }
      if (!climber.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class extend4A extends State {
  void enter() {
    startTime = new Date().getTime();
    climber.zeroEncoder();
    getclassName();
    climber.set(-.2);

  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (elapsedTime > 3000) {
        current = stop;
        return;
      }
      if (!climber.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}


class stop extends State {
  void enter() {
    climber.set(0);
  }

  void update() {}

  }




public class Climb {
  public static void main() {

    State.extend1G = new extend1G();
    State.driveForward1 = new driveForward1();
    State.retract1G = new retract1G();
    State.tiltForw1T = new tiltForw1T();
    State.extend2G = new extend2G();
    State.pivot2B = new pivot2B();
    State.extend3A = new extend3A();
    State.pivot3B = new pivot3B();
    State.retract2A = new retract2A();
    State.disengage1B = new disengage1B();
    State.pivot4B = new pivot4B();
    State.retract3A = new retract3A();
    State.pivot5B = new pivot5B();
    State.extend4A = new extend4A();
    State.stop = new stop();
    State.current = State.extend1G;

    while (true) {
      State.current.enter();
      State.current.update();
    }

  }

  public static Climb get_instance() {
    Climb climb = new Climb();
    return climb;
  }
}

/*
 * class forward extends State{
 * void enter() {
 * _i.set(.2);
 * 
 * }
 * void update() {
 * while(true){
 * if(_i.isClosed()){
 * current = reverse;
 * return;
 * }
 * }
 * 
 * }
 * 
 * }
 * class reverse extends State{
 * void enter() {
 * _i.set(-.2);
 * }
 * 
 * void update() {
 * while(true){
 * if(!_i.isClosed()){
 * current = stop;
 * return;
 * }
 * }
 * 
 * }
 * }
 * 
 * class stop extends State{
 * void enter() {
 * _i.set(0);
 * }
 * void update() {
 * if(!_i.isClosed()){
 * current = forward;
 * return;
 * }
 * 
 * }
 * }
 */
