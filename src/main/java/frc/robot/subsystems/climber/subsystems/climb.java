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
  static State retract1;
  static State wait1;
  static State extend1;
  static State fire1;
  static State extend2;
  static State fire2;
  static State retract2;
  static State wait2;
  static State retract3;
  /*static State disengage1B;
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


class retract1 extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.resetEncoder();
    _i.setGrip(-.6);
    getclassName();
  }

  void update() {
    while (true) {
     // System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      //System.out.println(_i.getEncoderPos());
      if (_i.getEncoderPosition() > -50) {
        _i.setGrip(0);
        current = wait1;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}
class wait1 extends State {
  void enter() {
    //System.out.println(_i.getEncoderPos());
    startTime = new Date().getTime();
    _i.resetEncoder();
    getclassName();
    _i.setGrip(.1);

  }

  void update() {
    while (true) {
      elapsedTime = new Date().getTime() - startTime;
      if (_i.getEncoderPosition() > 10 ) {
        _i.setGrip(0);
        current = extend1;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}
class extend1 extends State {
  void enter() {
    startTime = new Date().getTime();
    getclassName();
    _i.resetEncoder();
    _i.setGrip(.6);



    
  }

  void update() {
    while (true) {
      System.out.println(_i.getEncoderPosition());
      if (_i.getEncoderPosition() > 20 ) {
        _i.setGrip(0);
        current = fire1;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }



      // Will be making command
    }

  }

}

class fire1 extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.setGripSol(true);
    _i.resetEncoder();
    current = extend2;
    getclassName();
  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
     /* if (_i.getEncoderPosition() < 60 ) {
        _i.setGrip(0);
        current = extend2;
        return;
      }*/
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class extend2 extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.resetEncoder();
    _i.setGrip(.6);
    getclassName();
  }

  void update() {
    while (true) {
      elapsedTime = new Date().getTime() - startTime;
      //System.out.println(_i.getEncoderPos());
      if (_i.getEncoderPosition() > 50) {
        _i.setGrip(0);
        current = fire2;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class fire2 extends State {
  void enter() {

    startTime = new Date().getTime();
    _i.resetEncoder();
    getclassName();
    _i.setGripSol(false);
    current = retract2;
  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
     /* if (elapsedTime > 2000  && elapsedTime < 4000) {
        _i.setGrip(.2);
        return;
      }
      if (elapsedTime > 4000 ) {
        _i.setGrip(0);
        
        return;
      }*/
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class retract2 extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.resetEncoder();
    _i.setGrip(-6);
    getclassName();
  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (_i.getEncoderPosition() > -50) {
        _i.setGrip(0);
        current = wait2;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class wait2 extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.resetEncoder();
    getclassName();
    _i.setGrip(.1);

  }

  void update() {
    while (true) {
      elapsedTime = new Date().getTime() - startTime;
     // System.out.println(_i.getEncoderPos());



     // set until tippy is off of bar
      if (_i.getEncoderPosition() > 15) {
        _i.setGrip(0);
        current = retract3;
        return;
      }
     
  
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class retract3 extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.resetEncoder();
    getclassName();
    _i.setGrip(-.2);

  }
//sup
  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
      if (_i.getEncoderPosition() > -50) {
        _i.setGrip(0);
        current = stop;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}



/*class disengage1B extends State {
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

    State.retract1 = new retract1();
    State.extend1 = new extend1();
    State.fire1 = new fire1();
    State.extend2 = new extend2();
    State.fire2 = new fire2();
    State.retract2 = new retract2();
    State.wait2 = new wait2();
    State.retract3 = new retract3();
    State.wait1 = new wait1();
    /*State.disengage1B = new disengage1B();
    State.pivot4B = new pivot4B();
    State.retract3A = new retract3A();
    State.pivot5B = new pivot5B();
    State.extend4A = new extend4A();*/
    State.stop = new stop();
    State.current = State.retract1;

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