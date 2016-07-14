import weka.core.*;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.File;
import java.util.ArrayList;
import javax.sql.DataSource;
import weka.classifiers.functions.MultilayerPerceptron;
import weka.classifiers.misc.SerializedClassifier;

public class Weka
{
	public SerializedClassifier sc;
	public ArrayList<Attribute> att;
	
	public Weka(String model){
		sc = new SerializedClassifier();
		att = new ArrayList<Attribute>();
		sc.setModelFile(new File("MP_99.7.model"));
	}
	public double runClassification(String features){
		att.clear();	
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
		att.add(new Attribute("distance"));
		
		Instances i = new Instances("TestRelation",att,1);
		i.setClassIndex(0);
		DenseInstance instance = new DenseInstance(2); 
		/*instance.setValue(att, features);
		double val=0;
		
		try {
			val=sc.classifyInstance(i.get(0));
			} catch (Exception e) {e.printStackTrace();}*/
		System.out.println(features);
		return 3.14;
		
	}
}