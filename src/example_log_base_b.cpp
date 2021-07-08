/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
        Calculate the log of the number with base b i.e. log_b(x) = y
        The log is shortcut for the repeated division. For example
        2^3 = 8 => log_2(8) = 3
        which means the number 8 is divided by the number 2(base), 3 times to get a value less then the base.
        10^2 = 100 => log_10(100) = 2
        which means the number 100 is divided by the number 10(base), 2 times to get a value less then the base.
        In general the log_n(x)  means how many times the number is divided by the base to get a value less then the base.
        Example:
            log_3(150) = log_10(150)/log_10(3) = log_2(150)/log_2(3) = 4.56
            Explanation
            since 3^4=81 and 3^5=243, this means the number must be between 4 and 5.
            150 is divided 4 times by 3 to reach 1.85 (which is less then 3(base)), to get the value after decimal.
            let's rais the 10th power of the reminder (1.85185 ^ 10 = 474.3), because of the decimal number system.
            keep the division process with the result until we get value less then 3(base), it needs 5 division and the 
            reminder is 1.95. So far our result after one decimal place is 4.5.
            Next, further raise the 10th power of the reminder (1.95189^10 = 802.71). which nees further 6 divisions by 3
            to get 1.1011. So our result until the two decimail place is 4.56.
        Example:
            If there are five alphabets which can occure with different probabilities e.g.
            p(A) = p(B) = p(C) = p(D) = p(E) = 0.2, then the entropy can be calculated as follows
            Entropy = - Sum (0.2*log2(0.2)+0.2*log2(0.2)+0.2*log2(0.2)+0.2*log2(0.2)+0.2*log2(0.2))
            Entropy = 2.32 
            (Maximum because each symbol is equally likely) = log2(n)
            where n are number os samples, since each sample is equal likely, therefore, they sum up to 1.
            In other words, the worst case entropy is the log2(Size of the set).
            It can also be written as log10(Size of the set)/log10(2)


*/

#include <ros/ros.h>     // ROS stuff
#include <bits/stdc++.h> // C++ stuff

using namespace std;

int main(int argc, char *argv[])
{
    double b = 3, x = 150;
    cout << log2(150) / log2(3) << endl;

    // Implementation
    // Get the number before decimal
    double result = 0;
    double init = x;
    while (init > b)
    {
        init /= b;
        result++;
    }
    // Get the number after decimal
    double pos = 1;
    for (int i = 0; i < 10; i++) // More iterations, more precision
    {
        init *= init; // square the number (power of 2)
        pos /= 2.0;
        if (init > b)
        {
            result += pos;
            init /= b;
        }
    }
    // Display result
    cout << result << endl;

    return 0;
}