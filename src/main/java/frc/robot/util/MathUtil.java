package frc.robot.util;

public class MathUtil {
    public static double mapValue(double inLow, double inHigh, double outLow, double outHigh, double ip){
        double n = ip;
        if(n>inHigh){
          n = inHigh;
        }else if(n < inLow){
          n = inLow;
        }
    
        n = n - inLow;
        n = n / (inHigh-inLow);
        n = n * (outHigh-outLow);
        n = n + outLow;
    
        return n;
      }

    public static double clip(double val, double min, double max){
        if(val >= max){
            val = max;
        }else if(val <=min){
            val = min;
        }
        return val;
    }
}