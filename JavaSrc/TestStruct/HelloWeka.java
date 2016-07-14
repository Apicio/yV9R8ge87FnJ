import weka.core.*;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.File;
import java.util.ArrayList;
import javax.sql.DataSource;
import weka.classifiers.functions.MultilayerPerceptron;
import weka.classifiers.misc.SerializedClassifier;

public class HelloWeka
{
	public static void main(String args[]) 
	{
		System.out.println("Hello Weka" + args);
		try{
		  BufferedReader reader = new BufferedReader(new FileReader("data.arff"));
		  Instances data = new Instances(reader);
		  reader.close();
		  data.setClassIndex(data.numAttributes() - 1);
		  }catch(IOException e){
			  
		}
	}
}