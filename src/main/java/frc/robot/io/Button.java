package frc.robot.io;

public class Button {

    public enum ButtonID{

        A(1), B(2), X(3), Y(4), LEFT_BUMPER(5), RIGHT_BUMPER(6);

        ButtonID(int port){
            mPort = port;
        }

        private int mPort;

        public int getID(){
            return mPort;
        }
    }
}
