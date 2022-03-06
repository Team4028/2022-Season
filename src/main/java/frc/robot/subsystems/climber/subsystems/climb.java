// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Date;
import java.util.Scanner;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

class State {
  static State current;
  static State extend1G;
  static State driveForward1;
  static State retract1G;
  static State extend2G;
  static State fireNewGExtendG;
  static State gNewTNewRetGRet;
  static State retGFireTNewretG;
 /* static State pivot3B;
  static State retract2A;
  static State disengage1B;
  static State pivot4B;
  static State retract3A;
  static State pivot5B;
  static State extend4A;*/
  static State stop;
  static climber _i;

  static long startTime;
  static long elapsedTime;

  public State() {
    _i = climber.get_instance();
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
    _i.zeroEncoder();
    _i.setGrip(.2);
    getclassName();
  }

  void update() {
    while (true) {
     // System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      //System.out.println(_i.getEncoderPos());
      if (_i.getEncoderPos() > 120 ) {
        _i.setGrip(0);
        current = driveForward1;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class driveForward1 extends State {
  void enter() {
   /* startTime = new Date().getTime();
    getclassName();
    _i.zeroEncoder();
    _i.setGrip(-.2);*/



    // Will be putting in command
  }

  void update() {
    while (true) {
      /*System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (elapsedTime > 3000) {
        current = retract1G;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }*/



      // Will be making command
    }

  }

}

class retract1G extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.setGrip(-.2);
    _i.zeroEncoder();
    getclassName();
  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (_i.getEncoderPos() < 60 ) {
        _i.setGrip(0);
        current = extend2G;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class extend2G extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.zeroEncoder();
    _i.setGrip(.2);
    getclassName();
  }

  void update() {
    while (true) {
      elapsedTime = new Date().getTime() - startTime;
      //System.out.println(_i.getEncoderPos());
      if (_i.getEncoderPos() > 120) {
        _i.setGrip(0);
        current = fireNewGExtendG;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class fireNewGExtendG extends State {
  void enter() {

    startTime = new Date().getTime();
    _i.zeroEncoder();
    getclassName();
    _i.setGripSol1(Value.kForward);
    _i.setGripSol2(Value.kForward);

  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (elapsedTime > 2000  && elapsedTime < 4000) {
        _i.setGrip(.2);
        return;
      }
      if (elapsedTime > 4000 ) {
        _i.setGrip(0);
        current = gNewTNewRetGRet;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class gNewTNewRetGRet extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.zeroEncoder();
    getclassName();
    _i.setGripSol1(Value.kReverse);
    _i.setGripSol2(Value.kReverse);
    _i.setTipSol1(Value.kReverse);
    _i.setTipSol2(Value.kReverse);

  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
        if (elapsedTime > 2000  && elapsedTime < 4000) {
          _i.setGrip(-.2);
        return;
      }
      if (elapsedTime > 4000) {
        _i.setGrip(0);
        current = retGFireTNewretG;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class retGFireTNewretG extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.zeroEncoder();
    getclassName();
    //_i.setGrip(.2);

  }

  void update() {
    while (true) {
      elapsedTime = new Date().getTime() - startTime;
     // System.out.println(_i.getEncoderPos());



     // set until tippy is off of bar
      if (_i.getEncoderPos() > 60) {
        _i.setGrip(-.2);
        return;
      }
      if (elapsedTime > 5000  && elapsedTime < 7500) {
        _i.setTipSol1(Value.kForward);
        _i.setTipSol2(Value.kForward);
      return;
    }
    if (elapsedTime > 7500  && elapsedTime < 8000) {
    if (_i.getEncoderPos() > -150) {
      _i.setGrip(.2);
    }
    else {
      _i.setGrip(0);
      current = stop;
    }
  }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

/*class pivot3B extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.zeroEncoder();
    getclassName();
    _i.setGrip(-.2);

  }
//sup
  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (elapsedTime > 3000) {
        current = retract2A;
        return;
      }
      if (!_i.isClosed()) {
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
    _i.zeroEncoder();
    getclassName();
    _i.setGrip(.2);

  }

  void update() {
    while (true) {
      elapsedTime = new Date().getTime() - startTime;
      if (_i.getEncoderPos() > 120 ) {
        current = disengage1B;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class disengage1B extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.zeroEncoder();
    getclassName();
    _i.setGrip(-.2);

  }

  void update() {
    while (true) {
      elapsedTime = new Date().getTime() - startTime;
     // System.out.println(_i.getEncoderPos());
      if (elapsedTime > 3000) {
        current = pivot4B;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class pivot4B extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.zeroEncoder();
    _i.setGrip(.2);
    getclassName();
  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (_i.getEncoderPos() > 120 ) {
        current = retract3A;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class retract3A extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.zeroEncoder();
    getclassName();
    _i.setGrip(-.2);

  }

  void update() {
    while (true) {
     // System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (elapsedTime > 3000) {
        current = pivot5B;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class pivot5B extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.zeroEncoder();
    getclassName();
    _i.setGrip(.2);

  }

  void update() {
    while (true) {
      elapsedTime = new Date().getTime() - startTime;
      //System.out.println(_i.getEncoderPos());
      if (_i.getEncoderPos() > 120) {
        current = extend4A;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class extend4A extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.zeroEncoder();
    getclassName();
    _i.setGrip(-.2);

  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (elapsedTime > 3000) {
        current = stop;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}*/


class stop extends State {
  void enter() {
    _i.setGrip(0);
  }

  void update() {}

  }




public class climb {
  public static void main() {

    State.extend1G = new extend1G();
    State.driveForward1 = new driveForward1();
    State.retract1G = new retract1G();
    State.extend2G = new extend2G();
    State.fireNewGExtendG = new fireNewGExtendG();
    State.gNewTNewRetGRet = new gNewTNewRetGRet();
    State.retGFireTNewretG = new retGFireTNewretG();
    /*State.pivot3B = new pivot3B();
    State.retract2A = new retract2A();
    State.disengage1B = new disengage1B();
    State.pivot4B = new pivot4B();
    State.retract3A = new retract3A();
    State.pivot5B = new pivot5B();
    State.extend4A = new extend4A();*/
    State.stop = new stop();
    State.current = State.extend1G;

    while (true) {
      State.current.enter();
      State.current.update();
    }

  }

  public static climb get_instance() {
    climb _instance = new climb();
    return _instance;
  }
}

/*
 * class forward extends State{
 * void enter() {
 * _i.setGrip(.2);
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
 * _i.setGrip(-.2);
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
 * _i.setGrip(0);
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