/*
    Author: Dr. Ing. Ahmad Kamal Nasir
    Email: dringakn@gmail.com
    Description:
      Convert the GPS Geodatic coordinates into the Cartesian coordinates.
      The z-axis/altitude will be left as it is, therefore, don't require to be converted.
    Notes:
      Cartesian: easting = x, northing = y, altitude = z
      Geodatic: Latitude = x, Longitude = y, altitude = z
*/
#include <bits/stdc++.h>
#include <gps_conversions.h>
#include <ros/ros.h>

using namespace std;

int main(int argc, char* argv[])
{
  //   ros::init(argc, argv, "example_gps_conversions");
  //   ros::NodeHandle nh;

  /*
  index	name	    longitude	        latitude	        elevation
    1	Point 1	    -8.44203080508882	40.2302032564375	78.7976802631579
    2	Point 2	    -8.44190803264883	40.2300210920837	78.6117641860465
    3	Point 3	    -8.44177559121158	40.2298210959457	78.4193481012657
    4	Point 4	    -8.44163898298858	40.2298744761743	78.4668382857144
    5	Point 5	    -8.44177069146842	40.2300741533997	78.4685222527473
    6	Point 6	    -8.44188682828131	40.2302532401066	78.7466698209719
    7	Point 7	    -8.44191376790555	40.2302246349381	78.6167026058632
    8	Point 8	    -8.44198131768767	40.2301995581995	78.5883797900263
    9	Point 9	    -8.44184657045027	40.2299929791707	78.4165585365853
    10	Point 10	-8.44177057227171	40.2298888765154	78.3307582633054
    11	Point 11	-8.44170371082388	40.2299172114597	78.2917740298507
    12	Point 12	-8.44175806070625	40.2299987645875	78.364098125
    13	Point 13	-8.44181518252599	40.2300873318117	78.4225301948052
    14	Point 14	-8.4418224513127	40.2300272101466	78.4232381107492

   */
  double northing, easting;
  std::string zone;
  double coords[][3] = {
    // lat, long, altitude
    { -8.44203080508882, 40.2302032564375, 78.7976802631579 },
    { -8.44190803264883, 40.2300210920837, 78.6117641860465 },
    { -8.44177559121158, 40.2298210959457, 78.4193481012657 },
    { -8.44163898298858, 40.2298744761743, 78.4668382857144 },
    { -8.44177069146842, 40.2300741533997, 78.4685222527473 },
    { -8.44188682828131, 40.2302532401066, 78.7466698209719 },
    { -8.44191376790555, 40.2302246349381, 78.6167026058632 },
    { -8.44198131768767, 40.2301995581995, 78.5883797900263 },
    { -8.44184657045027, 40.2299929791707, 78.4165585365853 },
    { -8.44177057227171, 40.2298888765154, 78.3307582633054 },
    { -8.44170371082388, 40.2299172114597, 78.2917740298507 },
    { -8.44175806070625, 40.2299987645875, 78.364098125 },
    { -8.44181518252599, 40.2300873318117, 78.4225301948052 },
    { -8.4418224513127, 40.2300272101466, 78.4232381107492 },
  };

  int idx = 1;
  cout.precision(14);
  cout.setf(ios::fixed);
  for (auto&& pt : coords)
  {
    gps_common::LLtoUTM(pt[0], pt[1], northing, easting, zone);
    // easting = x, northing = y, altitude = z
    cout << "Point - " << idx++ << endl;
    cout << "lat:" << pt[0] << ", long:" << pt[1] << ", alti:" << pt[2] << endl;
    cout << "x:" << easting << ", y:" << northing << ", z:" << pt[2] << endl << endl;
  }

  return 0;
}