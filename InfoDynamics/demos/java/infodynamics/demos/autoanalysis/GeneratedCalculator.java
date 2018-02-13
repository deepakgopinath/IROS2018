package infodynamics.demos.autoanalysis;

import infodynamics.utils.ArrayFileReader;
import infodynamics.utils.MatrixUtils;

import infodynamics.measures.continuous.*;
import infodynamics.measures.continuous.kraskov.*;

public class GeneratedCalculator {

  public static void main(String[] args) throws Exception {

    // 0. Load/prepare the data:
    String dataFile = "/home/deepak/Desktop/Research/PaperSubmissions/IROS2018/InfoDynamics/demos/data/2coupledRandomCols-1.txt";
    ArrayFileReader afr = new ArrayFileReader(dataFile);
    double[][] data = afr.getDouble2DMatrix();
    double[] source = MatrixUtils.selectColumn(data, 0);
    double[] destination = MatrixUtils.selectColumn(data, 1);

    // 1. Construct the calculator:
    TransferEntropyCalculatorKraskov calc;
    calc = new TransferEntropyCalculatorKraskov();
    // 2. Set any properties to non-default values:
    calc.setProperty(TransferEntropyCalculator.K_PROP_NAME,
        "2");
    // 3. Initialise the calculator for (re-)use:
    calc.initialise();
    // 4. Supply the sample data:
    calc.setObservations(source, destination);
    // 5. Compute the estimate:
    double result = calc.computeAverageLocalOfObservations();

    System.out.printf("TE_Kraskov (KSG)(col_0 -> col_1) = %.4f nats\n",
        result);
  }
}

