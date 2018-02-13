package org.usfirst.frc.team7176.robot;

public class KinematicsFunction {
	
	private final static double ARM_LEN1 = 62.0;
	private final static double ARM_LEN2 = 62.0;
	public KinematicsFunction() {
		
	}
	public static double[] getPositionXY(double thelta1, double thelta2) {
		double x = 0;
		double y = 0;
         double armX1 = ((ARM_LEN1 * Math.cos(thelta1)));
         double armY1 = ((ARM_LEN1 * Math.sin(thelta1)));

         x = armX1 + (ARM_LEN2 * Math.cos(thelta1 - thelta2));
         y = armY1 + (ARM_LEN2 * Math.sin(thelta1 - thelta2));
         return new double[] { x, y };
	}
	
	public static double[] getJointAngle(double x, double y)
    {
        //calculate here

        double l1 = ARM_LEN1;
        double l2 = ARM_LEN2;
        double s2 = 0;
        double thelta1 = 0;

        double thelta2 = 0;

        double belta = 0;
        double alpha = 0;
        double c2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);

        if (Math.abs(c2) > 1)
        {
            //out of space
            thelta1 = -100;
            thelta2 = -100;

        }
        else
        {
            s2 = Math.sqrt(1 - c2 * c2);
            thelta2 = Math.atan2(s2, c2);
            belta = Math.atan2(y, x);
            alpha = Math.acos((x * x + y * y + l1 * l2 - l2 * l2) / (2 * l1 * Math.sqrt(x * x + y * y)));
            if ((alpha > 0) && (alpha < Math.PI))
            {
                thelta1 = belta + alpha;
            }

        }
        if (thelta1 >=0) thelta1 = thelta1 - Math.PI * 2;
        return new double[] { thelta1, thelta2 };
    } 
	
	public static double[] getJointAngleL(double x, double y)
    {
        //calculate here


        double l1 = ARM_LEN1;
        double l2 = ARM_LEN2;
        double s2 = 0;
        double thelta1 = 0;

        double thelta2 = 0;

        double belta = 0;
        double alpha = 0;
        double c2 = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);

        if (Math.abs(c2) > 1)
        {
            //out of space
            thelta1 = -100;
            thelta2 = -100;

        }
        else
        {
            s2 = -Math.sqrt(1 - c2 * c2);
            thelta2 = Math.atan2(s2, c2);
            belta = Math.atan2(y, x);
            double k1 = l1 + l2 * Math.cos(thelta2) ;
            double k2 = l2* Math.sin(thelta2);
            thelta1 = Math.atan2(y, x) + Math.atan2(k2, k1);

        }
        if (thelta1 >=0) thelta1 = thelta1 - Math.PI * 2;
        return new double[] { thelta1, thelta2 };
    }


}
