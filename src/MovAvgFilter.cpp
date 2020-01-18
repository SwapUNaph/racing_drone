/**
 * @file MovAvgFilter.cpp
 * @author Swapneel Naphade (naphadeswapneel@gmail.com)
 * @brief Moving Average Filter class definition
 * @version 0.1
 * @date 01-18-2020
 * 
 *  Copyright (c) 2020 Swapneel Naphade
 * 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 * 
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 * 
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

#include "racing_drone/MovAvgFilter.hpp"

MovAvgFilter::MovAvgFilter(int win_, int n_) : window(win_), n(n_)
{
    input.resize(n);
    output.resize(n);
    sumVec.resize(n);

    for(int i=0; i<n; i++)
    {
        input(i) = 0.0;
        output(i) = 0.0;
        sumVec(i) = 0.0;
    }
}

MovAvgFilter::~MovAvgFilter(){}

ublas::vector<double> MovAvgFilter::update(ublas::vector<double>& inVec)
{
    input = inVec;  
    if( valQ.size() == window )
    {
        sumVec -= valQ.front();
        valQ.pop();
    }   

    sumVec += input;
    valQ.push(input);

    if(valQ.size() != 0)
        output = sumVec / valQ.size();

    std::cout << "Input: " << input << ", Output: " << output << std::endl;
    
    return output;
}