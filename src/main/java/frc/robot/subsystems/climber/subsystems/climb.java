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
  static State extend1H;
  static State fire1H;
  static State extend2H;
  static State fire2H;
  static State retract2H;
  static State wait2H;
  static State retract3H;
  static State extend1T;
  static State fire1T;
  static State extend2T;
  static State fire2T;
  static State retract2T;
  static State wait2T;
  static State retract3T;
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
        current = extend1H;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}
class extend1H extends State {
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
        current = fire1H;
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

class fire1H extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.setGripSol(true);
    _i.resetEncoder();
    current = extend2H;
    getclassName();
  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
     /* if (_i.getEncoderPosition() < 60 ) {
        _i.setGrip(0);
        current = extend2H;
        return;
      }*/
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class extend2H extends State {
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
        current = fire2H;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class fire2H extends State {
  void enter() {

    startTime = new Date().getTime();
    _i.resetEncoder();
    getclassName();
    _i.setGripSol(false);
    current = retract2H;
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

class retract2H extends State {
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
        current = wait2H;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class wait2H extends State {
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
        current = retract3H;
        return;
      }
     
  
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class retract3H extends State {
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

class extend1T extends State {
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
        current = fire1H;
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

class fire1T extends State {
  void enter() {
    startTime = new Date().getTime();
    _i.setGripSol(true);
    _i.resetEncoder();
    current = extend2H;
    getclassName();
  }

  void update() {
    while (true) {
      //System.out.println(_i.getEncoderPos());
      elapsedTime = new Date().getTime() - startTime;
     /* if (_i.getEncoderPosition() < 60 ) {
        _i.setGrip(0);
        current = extend2H;
        return;
      }*/
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class extend2T extends State {
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
        current = fire2H;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class fire2T extends State {
  void enter() {

    startTime = new Date().getTime();
    _i.resetEncoder();
    getclassName();
    _i.setGripSol(false);
    current = retract2H;
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

class retract2T extends State {
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
        current = wait2H;
        return;
      }
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class wait2T extends State {
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
        current = retract3H;
        return;
      }
     
  
      if (!_i.isClosed()) {
        current = stop;
        return;
      }
    }

  }

}

class retract3T extends State {
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
    State.extend1H = new extend1H();
    State.fire1H = new fire1H();
    State.extend2H = new extend2H();
    State.fire2H = new fire2H();
    State.retract2H = new retract2H();
    State.wait2H = new wait2H();
    State.retract3H = new retract3H();
    State.wait1 = new wait1();
    State.extend1H = new extend1T();
    State.fire1H = new fire1T();
    State.extend2H = new extend2T();
    State.fire2H = new fire2T();
    State.retract2H = new retract2T();
    State.wait2H = new wait2T();
    State.retract3H = new retract3T();
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