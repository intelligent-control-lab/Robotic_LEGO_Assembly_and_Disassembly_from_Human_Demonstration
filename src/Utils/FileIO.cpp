/*
***********************************************************************************************************************************************************************
This file defines object loaders.
Copyright notice for IP Docket # 2023-234 and IP Docket # 2023-235.
Copyright (C) 2023

Authors:
Ruixuan Liu: ruixuanl@andrew.cmu.edu
Changliu Liu : cliu6@andrew.cmu.edu

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 3
of the License, or (at your option) any later version.
 
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
***********************************************************************************************************************************************************************
*/
#include <Utils/FileIO.hpp>

namespace lego_assembly
{
namespace io
{

Eigen::MatrixXd LoadMatFromFile(const std::string fname)
{
    try
    {
        Eigen::MatrixXd mat(0, 0);
        std::ifstream file(fname);
        if (!file.is_open())
        {
            std::ostringstream ss;
            ss << ERR_HEADER << "Cannot open file " << fname;
            throw std::runtime_error(ss.str());
        }
        std::string line;
        std::vector<double> row;
        while (std::getline(file, line))
        {
            row.clear();
            std::string word = ""; 
            for (auto x : line){
                if (x == '\t' || x == ' ' || x == ',') 
                {    
                    row.push_back(std::stod(word));
                    word = "";
                } 
                else
                { 
                    word = word + x; 
                } 
            }
            row.push_back(std::stod(word));
            mat = math::EigenVcat(mat, math::ToEigen(row).transpose());
        }

        file.close();

        std::cout << "Loaded mat of shape [" << mat.rows()
                    << ", " << mat.cols() << "] from [" << fname << "]\n";

        return mat;
    }
    catch(const std::exception& e)
    {
        throw;
    }
}

}
}