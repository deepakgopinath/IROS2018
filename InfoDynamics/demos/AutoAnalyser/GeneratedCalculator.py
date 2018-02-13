from jpype import *
import numpy
# Our python data file readers are a bit of a hack, python users will do better on this:
sys.path.append("/home/deepak/Desktop/Research/PaperSubmissions/IROS2018/InfoDynamics/demos/python")
import readFloatsFile

# Add JIDT jar library to the path
jarLocation = "/home/deepak/Desktop/Research/PaperSubmissions/IROS2018/InfoDynamics/infodynamics.jar"
# Start the JVM (add the "-Xmx" option with say 1024M if you get crashes due to not enough memory space)
startJVM(getDefaultJVMPath(), "-ea", "-Djava.class.path=" + jarLocation)

# 0. Load/prepare the data:
dataRaw = readFloatsFile.readFloatsFile("/home/deepak/Desktop/Research/PaperSubmissions/IROS2018/InfoDynamics/demos/data/2coupledRandomCols-1.txt")
# As numpy array:
data = numpy.array(dataRaw)
source = data[:,0]
destination = data[:,1]

# 1. Construct the calculator:
calcClass = JPackage("infodynamics.measures.continuous.kraskov").TransferEntropyCalculatorKraskov
calc = calcClass()
# 2. Set any properties to non-default values:
calc.setProperty("k_HISTORY", "2")
# 3. Initialise the calculator for (re-)use:
calc.initialise()
# 4. Supply the sample data:
calc.setObservations(source, destination)
# 5. Compute the estimate:
result = calc.computeAverageLocalOfObservations()

print("TE_Kraskov (KSG)(col_0 -> col_1) = %.4f nats" %
    (result))
