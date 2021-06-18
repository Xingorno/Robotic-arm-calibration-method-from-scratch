#include <fstream>
#include <iostream>

using namespace std;


template <size_t rows, size_t cols>
void ReadLookupTable(string fileName, double (&encoder_array)[rows][cols])
{
    ifstream dataFile(fileName);
    while (!dataFile.eof())
    {
        for (int i = 0; i < rows; i++)
        {

            for (int j = 0; j < cols; j++)
            {
                dataFile >> encoder_array[i][j];
            }
        }
       
    }

}

template <size_t rows, size_t cols>
double LinearInterpolation(double(&encoder_array)[rows][cols], double encoderReading_degree)
{
    double correctedAngle_degree;
    int indexLeft, indexRight;
    if (encoderReading_degree < encoder_array[0][0])
    {
        return encoderReading_degree;
    }
    if (encoderReading_degree > encoder_array[0][cols-1])
    {
        return encoderReading_degree;
    }

    for (int i = 0; i < cols; i++)
    {
        if (encoderReading_degree < encoder_array[0][i+1])
        {
            indexLeft = i;
            indexRight = i + 1;
            double delta = encoder_array[0][indexRight] - encoder_array[0][indexLeft];
            
            correctedAngle_degree = (encoder_array[0][indexRight] - encoderReading_degree) * encoder_array[1][indexLeft] / delta + (encoderReading_degree - encoder_array[0][indexLeft]) * encoder_array[1][indexRight] / delta;
            return correctedAngle_degree;
            
        }
    }
}

void main()
{
	
    string fileNameEncoder1 = "E:/PROGRAM/Project_PhD/Calibration/MyCode/LUT_algorithm/LUT_encoder/encoder1_LUT_mean.txt";
    string fileNameEncoder2 = "E:/PROGRAM/Project_PhD/Calibration/MyCode/LUT_algorithm/LUT_encoder/encoder2_LUT.txt";
	// Define encoder lookup table
	double encoder1_LUT[2][361];
	double encoder2_LUT[2][89];
	//double encoder3_LUT[2][101];

    ReadLookupTable(fileNameEncoder1, encoder1_LUT);
    ReadLookupTable(fileNameEncoder2, encoder2_LUT);





    double encoder1CorrectedAngle_degree, encoder2CorrectedAngle_degree;

    encoder1CorrectedAngle_degree = LinearInterpolation(encoder1_LUT, -30);
    encoder2CorrectedAngle_degree = LinearInterpolation(encoder2_LUT, 20);
    cout << "corrected angle in degree:" << encoder1CorrectedAngle_degree << endl;
    
}

