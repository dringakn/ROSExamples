/**
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Desciption: Entropy Estimation
        Entropy is a measure of variability for a categorical data, because variance
        can't handle the discreate data. For example, if a certain range of numbers
        are encoded as a label, e.g (0-10] = 1, (10-30] = 2, (30-40] = 3 then the variance
        will fail to encapusulate the homogenity of the data. 
        The entropy of a vector can be calculated as follows:
            E(v) = - Sum( p(i) * log_n( p(i) ) )
            where 
            p(i) = probability of the i-label = ith-Outcomes/TotalOutcomes
            -log_n(p(i)) = log_n(1/p(i)) = Information gain
            Note:
                The base n of the log represent the number of states for the label.
                If it has two states such as true/false then it can be treated as bits or log_2.
                If it has three states such as free/occupied/unknown then it can be treated as trits or log_3.
                log_n(x) can be calculated as: log_2(x)/log_2(n)
        Example: Calculate the entropy of {1, 2, 2, 2, 2, 2, 3, 3} with each label can have three states.
  
 **/

#include <bits/stdc++.h>
using namespace std;

int main(int argc, char *argv[])
{
    // vector<int> v = {1, 1, 1, 2, 2, 2, 3, 3, 3}; // Maximum entropy
    vector<int> v = {1, 1, 1, 1, 1, 1, 1, 1, 1}; // Minimum Entropy
    // vector<int> v = {1, 2, 2, 2, 2, 2, 3, 3, 3};
    unordered_map<int, double> p;

    // Calculate outcome for each label
    for (auto &&e : v)
        p[e]++;

    double p_label, entropy = 0;
    for (auto &&e : p)
    {
        cout << e.first << "\t" << e.second << endl;
        // Normalize the probability
        p_label = e.second / v.size();
        entropy += (p_label * log2(p_label));
    }
    entropy *= (-1 / log2(3)); // Constant factor
    cout << "Entropy: " << entropy << endl;

    return 0;
}