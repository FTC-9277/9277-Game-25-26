package org.firstinspires.ftc.teamcode;

import android.webkit.JavascriptInterface;

import java.util.ArrayList;

class MyClass {

    public int a = 0;

    public void myFunction() {
        int myVal = 1;
    }

    int b = 1;

    public static double addDouble(double a, double b) {
        return a + b;
    }

    public static void main(String[] args) {

        double firstDouble = 0.3456789;
        double secondDouble = 0.986432;

        double thirdDouble = addDouble(firstDouble, secondDouble);

        System.out.println(thirdDouble);


        String name = "Hello!";


        ArrayList<String> myList = new ArrayList<>();
        myList.add("Hello!");
        myList.add("Hello!");
        myList.add("Hello!");
        myList.add("Hello!");
        myList.add("Hello!");
        System.out.println(myList.toString());
        // Result: {"Hello!", "Hello!", "Hello!", "Hello!", "Hello!"}

        myList.add(3,"I am here");
        System.out.println(myList.toString());
        // Result: {"Hello!", "Hello!", "Hello!", "I am here", "Hello!", "Hello!"}
        char newString = 'h';






        double myDouble = 22/7;
        int myInt = (int)myDouble;
        // Result: 1

        double myDouble2 = 1.999;
        long myInt2 = Math.round(myDouble2);
        // Result: 2

    }


}

