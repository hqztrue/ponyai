//gcd
package q1;
import q1.Gcd;

import java.io.*;
import java.util.*;
import java.math.*;
public class Main{
	static BigInteger a,b,c;
	public static void main(String[] args){
		Scanner cin=new Scanner(System.in);
		a=cin.nextBigInteger();b=cin.nextBigInteger();
		c=Gcd.get_gcd(a,b);
		System.out.println(c);
	}
}



