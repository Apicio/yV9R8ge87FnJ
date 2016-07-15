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
	public SerializedClassifier sc;
	//public ArrayList<Attribute> att;
	
	public Weka(String model){
		sc = new SerializedClassifier();
/*		att = new ArrayList<Attribute>();
		att.add(new Attribute("meanHue"));
		att.add(new Attribute("mom1"));
		att.add(new Attribute("mom2"));
		att.add(new Attribute("mom3"));
		att.add(new Attribute("mom4"));
		att.add(new Attribute("mom5"));
		att.add(new Attribute("mom6"));
		att.add(new Attribute("mom7"));
		att.add(new Attribute("stdDevHue"));
		att.add(new Attribute("entropy"));
		att.add(new Attribute("area"));
		att.add(new Attribute("distance"));*/
		sc.setModelFile(new File(model));
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
							"@attribute class {mela_rossa,mela_gialla,bicchiere,tazzina}\r\n"+
							"\r\n"+
							"@data\r\n"+
							features;
		System.out.println(toInstance);
		double val = Double.MIN_VALUE;
		try {
			InputStream is = new ByteArrayInputStream(toInstance.getBytes());
			Instances unlabeled = new Instances(new BufferedReader(new InputStreamReader(is)));
			unlabeled.setClassIndex(unlabeled.numAttributes() - 1);			
			val=sc.classifyInstance(unlabeled.get(0));
		} catch (Exception e) {e.printStackTrace();}
		
		return val;
		
	}
}