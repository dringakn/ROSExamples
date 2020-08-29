/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
*/
#include "random_numbers/random_numbers.h"
#include "ros/ros.h"

const unsigned int NP = 10;
double arr[NP] = {1, 5, 10, 15, 20, 3, 7, 8, 25, 50};

unsigned int max(double array[NP], double &maxValue) {
  unsigned int maxIdx = 0;
  maxValue = array[0];  // start with max = first element
  for (unsigned int i = 1; i < NP; i++)
    if (array[i] > maxValue) {
      maxValue = array[i];
      maxIdx = i;
    }
  return maxIdx;  // return index of highest value in array
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "example_resample_algorithm_node");
  ros::NodeHandle nh;
  random_numbers::RandomNumberGenerator rng;

  // normalize and print array
  double sumArr = 0;
  for (unsigned int i = 0; i < NP; i++) sumArr += arr[i];
  for (unsigned int i = 0; i < NP; i++) {
    arr[i] /= sumArr;
    printf("%5.3f,", arr[i]);
  }
  printf("\n");

  // resampling algorithm
  std::vector<double> S(NP);
  double beta = 0.0, maxArr;
  unsigned int maxArrIdx = max(arr, maxArr);
  unsigned int idx =
      rng.uniformInteger(0, NP - 1);  // Generate random sample index
  for (unsigned int k = 0; k < NP; k++) {
    beta = beta + rng.uniformReal(0, 2 * maxArr);
    while (beta > arr[idx]) {
      beta = beta - arr[idx];
      idx = (idx + 1) % (NP - 1);
    }
    S[k] = arr[idx];
  }

  // print results
  for (int i = 0; i < NP; i++) printf("%5.3f,", S[i]);
  printf("\nID:%d Max:%5.3f\n", maxArrIdx + 1, maxArr);

  return 0;
}