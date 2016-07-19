import weka.core.*;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.File;
import java.util.ArrayList;
import javax.sql.DataSource;
import weka.classifiers.functions.MultilayerPerceptron;
import weka.classifiers.misc.SerializedClassifier;
import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;

public class Weka
{
	public SerializedClassifier scRedApple, scYellowApple, scGlass, scCup;
	//public ArrayList<Attribute> att;

	public Weka(String model){
		scRedApple = new SerializedClassifier();
		scRedApple.setModelFile(new File("..//JavaSrc//TestStruct//new_rossa.model"));

		scYellowApple = new SerializedClassifier();
		scYellowApple.setModelFile(new File("..//JavaSrc//TestStruct//new_gialla.model"));

		scGlass = new SerializedClassifier();
		scGlass.setModelFile(new File("..//JavaSrc//TestStruct//new_bicchiere.model"));

		scCup = new SerializedClassifier();
		scCup.setModelFile(new File("..//JavaSrc//TestStruct//new_tazzina.model"));

	}

	public double runClassification(String features){
		/*		String[] parts = features.split(",");
		double[] nums = new double[parts.length];
		for (int i = 0; i < nums.length; i++) {
			nums[i] = Double.parseDouble(parts[i]);
		} */
		String toInstance = "@relation Nao_Vision_Cup\r\n"+
				"\n"+
				"@attribute meanHue numeric\r\n"+
				"@attribute mom1 numeric\r\n"+
				"@attribute mom2 numeric\r\n"+
				"@attribute mom3 numeric\r\n"+
				"@attribute mom4 numeric\r\n"+
				"@attribute mom5 numeric\r\n"+
				"@attribute mom6 numeric\r\n"+
				"@attribute mom7 numeric\r\n"+
				"@attribute stdDevHue numeric\r\n"+
				"@attribute entropy numeric\r\n"+
				"@attribute area numeric\r\n"+
				"@attribute distance numeric\r\n"+
				"@attribute class {obj, not_obj}\r\n"+
				"\r\n"+
				"@data\r\n"+
				features;
		System.out.println(toInstance);
		
		double values[] = new double[4];
		double tmp[] = new double[2];
		double currMax = Double.MIN_VALUE;
		String winner = null;
		double reject=0.6;
		try {
			InputStream is = new ByteArrayInputStream(toInstance.getBytes());
			Instances unlabeled = new Instances(new BufferedReader(new InputStreamReader(is)));
			unlabeled.setClassIndex(unlabeled.numAttributes() - 1);		
			/* Mele rosse: 0;
			 * Mele gialle: 1
			 * Bicchiere: 2
			 * Tazzina: 3
			 * */
			values[0] = scRedApple.classifyInstance(unlabeled.get(0));
			values[1] = scYellowApple.classifyInstance(unlabeled.get(0));
			values[2] = scGlass.classifyInstance(unlabeled.get(0));
			values[3] = scCup.classifyInstance(unlabeled.get(0));
			
			/*MP risponde o non risponde, quindi i valori sono 1 o 0:
			 * nel caso idale abbiamo, ad esempio 1 0 0 0
			 * nel caso peggiore possiamo avere 1 1 0 0, come decidiamo? Vediamo
			 * le probabilità e decidiamo dunque a massima prob.*/
			int count=0;;
			for(int i=0;i<values.length;i++){
				System.out.println(values[i]);
				if(values[i]==0)
					count++;
				}
			if(count>1){ //COMPETIZIONE: decidiamo a massima probabilità
				tmp = scRedApple.distributionForInstance(unlabeled.get(0));
				System.out.print("VETTORE PER MELE ROSSE: "+tmp[0]+" "+tmp[1]);
				if(tmp[0] >currMax && values[0] == 0){
					currMax = tmp[0];
					winner = "red";
				}
				tmp = scYellowApple.distributionForInstance(unlabeled.get(0));
				System.out.print("VETTORE PER MELE GIALLE: "+tmp[0]+" "+tmp[1]);
				if(tmp[0] >currMax && values[1] == 0){
					currMax =tmp[0];
					winner = "yellow";
				}
				tmp = scGlass.distributionForInstance(unlabeled.get(0));
				System.out.print("VETTORE PER  BICCHIERE: "+tmp[0]+" "+tmp[1]);
				if(tmp[0]>currMax&& values[2] == 0){
					currMax=tmp[0];
					winner = "glass";
				}
				
				tmp = scCup.distributionForInstance(unlabeled.get(0));
				System.out.print("VETTORE PER  TAZZINA: "+tmp[0]+" "+tmp[1]);
				if(tmp[0]>currMax&& values[3] == 0){
					currMax=tmp[0];
					winner = "cup";
				}
				if(currMax<reject)
					winner ="none";
			}
			
			
			
			else if(count == 1){
				if(values[0] == 0)
					winner = "red";
				if(values[1] == 0)
					winner ="yellow";
				if(values[2] == 0)
					winner ="glass";
				if(values[3] == 0)
					winner ="cup";
			}
			

			System.out.println("CurrMax: " +currMax);
		
		} catch (Exception e) {e.printStackTrace();}
		if(winner!=null)	{	
			if(winner.equals("red"))
				return 1;
			else if(winner.equals("yellow"))
				return 2;
			else if(winner.equals("glass"))
				return 3;
			else if(winner.equals("cup"))
				return 4;
			else if(winner.equals("none"))
				return 5;
		}
		return -1; //Incorrect classification

	}
}