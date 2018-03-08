//gcd
import java.io.*;
import java.util.*;
import java.math.*;
public class Main{
	static BigInteger a,b,c;
	public static void main(String[] args){
		Scanner cin=new Scanner(System.in);
		a=cin.nextBigInteger();b=cin.nextBigInteger();
		c=a.gcd(b);System.out.println(c);
	}
}



