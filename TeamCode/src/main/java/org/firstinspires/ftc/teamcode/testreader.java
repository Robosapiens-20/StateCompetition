package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Scanner;

public class testreader {
    public static Scanner in;
    {
        try {
            in = new Scanner(new File("inputs.in"));
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }
    }
   public static int N = in.nextInt();
    public static Trajectory trajarr[] = new Trajectory[N+1];
   public static void main(String[] args)
   {
       trajarr[0] =
       for(int i =1;i<N+1;i++)
       {
           String s = in.next();
       }
   }

}
