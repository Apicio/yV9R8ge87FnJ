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
import weka.classifiers.functions.SMO;



public class Weka
{
	public SMO ApplesVsCups, Apples, Cups;

	public Weka(String model) throws Exception{


		ApplesVsCups = (SMO) weka.core.SerializationHelper.read("..//JavaSrc//TestStruct//svm_apples_cups.model");
		Apples = (SMO) weka.core.SerializationHelper.read("..//JavaSrc//TestStruct//svm_apples_red_yellow.model");
		Cups = (SMO) weka.core.SerializationHelper.read("..//JavaSrc//TestStruct//svm_cups_coffee_plastic.model");
	}


	public Instances returnIstances(String s) throws IOException{
		InputStream is = new ByteArrayInputStream(s.getBytes());
		Instances unlabeled = new Instances(new BufferedReader(new InputStreamReader(is)));
		unlabeled.setClassIndex(unlabeled.numAttributes() - 1);	
		return unlabeled;
	}

	public double runClassification(String features1, String features2){
		double res1, res2, res3;
		double toReturn = 0;
		String winner ="";
		double[] prob = new double[4];
		String toInstanceApplesVsCups = "@relation Nao_Vision_Cup\r\n"+
				"\n"+
				"@attribute B numeric\r\n"+
				"@attribute G numeric\r\n"+
				"@attribute R numeric\r\n"+
				"@attribute class {cup,apple}\r\n"+
				"\r\n"+
				"@data\r\n"+
				features1;

		String toInstanceApples = "@relation Nao_Vision_Cup\r\n"+
				"\n"+
				"@attribute B numeric\r\n"+
				"@attribute G numeric\r\n"+
				"@attribute R numeric\r\n"+
				"@attribute class {mela_rossa,mela_gialla}\r\n"+
				"\r\n"+
				"@data\r\n"+
				features1;

		String toInstanceCups = "@relation Nao_Vision_Cup\r\n"+
				"\n"+
				"@attribute 0 numeric\r\n"+
				"@attribute 1 numeric\r\n"+
				"@attribute 16 numeric\r\n"+
				"@attribute 224 numeric\r\n"+
				"@attribute 225 numeric\r\n"+
				"@attribute 239 numeric\r\n"+
				"@attribute 240 numeric\r\n"+
				"@attribute 241 numeric\r\n"+
				"@attribute 248 numeric\r\n"+
				"@attribute 254 numeric\r\n"+
				"@attribute class {bicchiere,tazzina}\r\n"+
				"\r\n"+
				"@data\r\n"+
				features2;
		try {
			Instances unlabeled = returnIstances(toInstanceApplesVsCups);
			res1 = ApplesVsCups.classifyInstance(unlabeled.get(0));
			if(res1 == 0){ //Tazzine
				Instances unlabCups = returnIstances(toInstanceCups);
				res2 = Cups.classifyInstance(unlabCups.get(0));
				if(res2 == 0)
					winner = "bicchiere";
				else
					winner = "tazzina";
			}
			else if(res1==1){ //Apples
				Instances unlabApple = returnIstances(toInstanceApples);
				res3 = Apples.classifyInstance(unlabApple.get(0));
				if(res3 == 0)
					winner = "mela_rossa";
				else
					winner = "mela_gialla";
				
			}

		}catch (Exception e) {e.printStackTrace();}
		switch(winner){
		case "mela_rossa":
			toReturn = 0;
			break;
		case "mela_gialla":
			toReturn = 1;
			break;
		case "tazzina":
			toReturn = 2;
			break;
		case "bicchiere":
			toReturn= 3;
			break;
			
		}
		return toReturn; //misclasification
	}
}