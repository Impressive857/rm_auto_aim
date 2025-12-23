//
// Created by lbw on 25-7-23.
//

#ifndef CONTROLLER_TOOLS_H
#define CONTROLLER_TOOLS_H
#include <array>
#include <cmath>
#include <iostream>
#include <ostream>


namespace ckyf
{
    namespace Tools
    {
        template <int Window>
        class SigmaCalculator
        {
        public:
            using iter = typename std::array<double, Window>::iterator;

            SigmaCalculator(): data_len(0), mean(0), sum_square(0), variance(0)
            {
                tail_iter = datas.begin();
                for (auto& it : datas)
                {
                    it = 0;
                }
            }

            ~SigmaCalculator() = default;

            template <class T>
            void push(T& num)
            {
                double head_num = *tail_iter;
                *tail_iter = static_cast<double>(num);
                tail_iter = next(tail_iter);
                mean = (mean * data_len - head_num + num) / std::min(data_len + 1, Window);
                sum_square = sum_square - head_num * head_num + num * num;
                data_len = std::min(data_len + 1, Window);

                if (data_len > 1)
                    variance = (sum_square - data_len * mean * mean) / (data_len - 1);
                else
                    variance = 0.0;
            }

            double getSigma() { return std::sqrt(variance); }

            void reset()
            {
                mean = 0;
                sum_square = 0;
                for (auto& i : datas) i = 0;
                variance = 0;
            }

            iter next(iter it) { return it == datas.end() - 1 ? datas.begin() : it + 1; }

        private:
            int data_len;
            std::array<double, Window> datas;
            iter tail_iter;
            double mean;
            double sum_square;
            double variance;
        };
    }
}


#endif //CONTROLLER_TOOLS_H
