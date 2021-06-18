
#include <fstream>
#include <math.h> 
#include <string>
#include <iostream>
#include <sstream>


void main()
{
    std::ofstream output("E:/PROGRAM/Project_PhD/Calibration/MyCode/Data/20210613/20210613_fulljoints_NDI_data.txt");



    std::ifstream dataFile("E:/PROGRAM/Project_PhD/Calibration/MyCode/Data/20210613/20210613_Calibration_data_collection_encoder5_2230_full_filterted.txt");


    std::string line, v;
    while (!dataFile.eof())
    {
        getline(dataFile, line);
        if (line[0] == 'D' && line[1] == 'a') // read "" points
        {
            int i = 0;
            
            for (i = 0; i < 4; i++)
            {
                getline(dataFile, line);
                //getline(dataFile, line);
                //getline(datafile, line);
                std::istringstream iss(line);
                //iss >> v >> value_x >> value_y >> value_z;
                std::cout << line << std::endl;

                output << line << '\n';
            }

           // int a = line.size();

            //output << line[8] << ' ';
            //for (i = 10; i < a; i++)
            //{
            //    output << line[i];
            //}
            //output << '\n';


        }

    }

    output.close();
}
