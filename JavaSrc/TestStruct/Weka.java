import weka.core.*;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.ByteArrayInputStream;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.Arrays;

import weka.classifiers.functions.LibSVM;
import weka.classifiers.functions.MultilayerPerceptron;



public class Weka
{
	public LibSVM RedApple, YellowApple, Glass, Cup,NOTRedApple,	NOTYellowApple,NOTGlass,NOTCup;
	public MultilayerPerceptron MP;
	//public ArrayList<Attribute> att;

	public Weka(String model) throws Exception{


		MP = (MultilayerPerceptron) weka.core.SerializationHelper.read("..//JavaSrc//TestStruct//MP_100.model");
		/*RedApple =  (LibSVM) weka.core.SerializationHelper.read("res//Red.model");



		YellowApple =  (LibSVM) weka.core.SerializationHelper.read("res//Yellow.model");


		Glass =  (LibSVM) weka.core.SerializationHelper.read("res//Glass.model");


		Cup =  (LibSVM) weka.core.SerializationHelper.read("res//Cup.model");


		NOTRedApple =  (LibSVM) weka.core.SerializationHelper.read("res//NOTRed.model");


		NOTYellowApple =  (LibSVM) weka.core.SerializationHelper.read("res//NOTYellow.model");


		NOTGlass =  (LibSVM) weka.core.SerializationHelper.read("res//NOTGlass.model");


		NOTCup =  (LibSVM) weka.core.SerializationHelper.read("res//NOTCup.model");*/

	}

	double doMPClassify(String features){
		String toReturn = null;
		double[] prob = new double[4];
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
				"@attribute class {bicchiere, tazzina,mela_rossa,mela_gialla}\r\n"+
				"\r\n"+
				"@data\r\n"+
				features;
		try {
			InputStream is = new ByteArrayInputStream(toInstance.getBytes());
			Instances unlabeled = new Instances(new BufferedReader(new InputStreamReader(is)));
			unlabeled.setClassIndex(unlabeled.numAttributes() - 1);	
			double result = MP.classifyInstance(unlabeled.get(0));
			if(result == 0)
				return 3;
			else if(result == 1)
				return 4;
			else if(result == 2)
				return 1;
			else if(result == 3)
				return 2;

			prob = MP.distributionForInstance(unlabeled.get(0));

			Arrays.sort(prob);
			if(prob[prob.length-1]<0.9901)
				return 5;
		}catch (Exception e) {e.printStackTrace();}
		return -1; //misclasification
	}

public double runClassification(String features){
	
	String toReturn = null;
	double[] prob = new double[4];
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
			"@attribute class {bicchiere, tazzina,mela_rossa,mela_gialla}\r\n"+
			"\r\n"+
			"@data\r\n"+
			features;
	try {
		InputStream is = new ByteArrayInputStream(toInstance.getBytes());
		Instances unlabeled = new Instances(new BufferedReader(new InputStreamReader(is)));
		unlabeled.setClassIndex(unlabeled.numAttributes() - 1);	
		double result = MP.classifyInstance(unlabeled.get(0));
		if(result == 0)
			return 3; // bicchiere
		else if(result == 1)
			return 4;  //tazzina
		else if(result == 2)
			return 1;  //mela_rossa
		else if(result == 3)
			return 2; //mela gialla

		prob = MP.distributionForInstance(unlabeled.get(0));

		Arrays.sort(prob);
		if(prob[prob.length-1]<0.9901)
			return 5;
	}catch (Exception e) {e.printStackTrace();}
	return -1; //misclasification
		

		/*String MPpredict = doMPClassify(features);
		if(!MPpredict.equals("X")){
			if(MPpredict.equals("mela_rossa"))
				return 1;
			else if(MPpredict.equals("mela_gialla"))
				return 2;
			else if(MPpredict.equals("bicchiere"))
				return 3;
			else if(MPpredict.equals("tazzina"))
				return 4;
		 }else{

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

			double values[] = new double[8];
			double tmp[] = new double[2];
			double currMax = Double.MIN_VALUE;
			String winner = null;
			double reject=0.4;
			try {
				InputStream is = new ByteArrayInputStream(toInstance.getBytes());
				Instances unlabeled = new Instances(new BufferedReader(new InputStreamReader(is)));
				unlabeled.setClassIndex(unlabeled.numAttributes() - 1);		
				/* Mele rosse: 0;
				 * Mele gialle: 1
				 * Bicchiere: 2
				 * Tazzina: 3
				 * 
				values[0] = RedApple.classifyInstance(unlabeled.get(0));
				values[1] = YellowApple.classifyInstance(unlabeled.get(0));
				values[2] = Glass.classifyInstance(unlabeled.get(0));
				values[3] = Cup.classifyInstance(unlabeled.get(0));
				values[4] = NOTRedApple.classifyInstance(unlabeled.get(0));
				values[5] = NOTYellowApple.classifyInstance(unlabeled.get(0));
				values[6] = NOTGlass.classifyInstance(unlabeled.get(0));
				values[7] = NOTCup.classifyInstance(unlabeled.get(0));
				/*MP risponde o non risponde, quindi i valori sono 1 o 0:
				 * nel caso idale abbiamo, ad esempio 1 0 0 0
				 * nel caso peggiore possiamo avere 1 1 0 0, come decidiamo? Vediamo
				 * le probabilità e decidiamo dunque a massima prob.


				int count=0;;
				for(int i=0;i<values.length;i++){
					System.out.println(values[i]);
					if(values[i]==0)
						count++;
				}
				if(count>1){ //COMPETIZIONE: decidiamo a massima probabilità
					tmp = RedApple.distributionForInstance(unlabeled.get(0));
					if(tmp[0] >currMax){
						currMax = tmp[0];
						winner = "red";
					}
					tmp = YellowApple.distributionForInstance(unlabeled.get(0));
					if(tmp[0] >currMax){
						currMax =tmp[0];
						winner = "yellow";
					}
					tmp = Glass.distributionForInstance(unlabeled.get(0));
					if(tmp[0]>currMax){
						currMax=tmp[0];
						winner = "glass";
					}

					tmp = Cup.distributionForInstance(unlabeled.get(0));
					if(tmp[0]>currMax){
						currMax=tmp[0];
						winner = "cup";
					}

					tmp = NOTRedApple.distributionForInstance(unlabeled.get(0));
					if(tmp[0] >currMax){
						currMax = tmp[0];
						winner = "none";
					}
					tmp = NOTYellowApple.distributionForInstance(unlabeled.get(0));
					if(tmp[0] >currMax){
						currMax =tmp[0];
						winner = "none";
					}
					tmp = NOTGlass.distributionForInstance(unlabeled.get(0));
					if(tmp[0]>currMax){
						currMax=tmp[0];
						winner = "none";
					}

					tmp = NOTCup.distributionForInstance(unlabeled.get(0));
					if(tmp[0]>currMax){
						currMax=tmp[0];
						winner = "none";
					}

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

					if(values[4] == 0)
						winner = "none";
					if(values[5] == 0)
						winner ="none";
					if(values[6] == 0)
						winner ="none";
					if(values[7] == 0)
						winner ="none";
				}
				if(currMax<reject)
					winner ="none";

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
	//	}*/
	//	return -1; //Incorrect classification

	}
}