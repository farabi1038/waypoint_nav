#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <iostream>
#include <utility>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Joy.h>
#include <ros/duration.h>
#include <ros/time.h>
#include <math.h>


#include <tf/transform_listener.h>





double latiPoint_raw = 0, longiPoint_raw = 0, latiPoint_filtered = 0, longiPoint_filtered = 0;
double utmX_raw = 0, utmY_raw = 0, utmX_filtered = 0, utmY_filtered = 0;
double collection_time;
std::string utmZone, path_local_filtered, path_local_raw, path_abs_filtered, path_abs_raw;
bool collect_request = false;
bool continue_collection = true;
std::string end_button_sym, collect_button_sym;
int end_button_num = 0, collect_button_num = 0, numWaypoints= 0, numWaypoint_clusters= 0, numPoints;





const double RADIANS_PER_DEGREE = M_PI/180.0;
const double DEGREES_PER_RADIAN = 180.0/M_PI;

// Grid granularity for rounding UTM coordinates to generate MapXY.
const double grid_size = 100000.0;    // 100 km grid

// WGS84 Parameters
#define WGS84_A   6378137.0   // major axis
#define WGS84_B   6356752.31424518  // minor axis
#define WGS84_F   0.0033528107    // ellipsoid flattening
#define WGS84_E   0.0818191908    // first eccentricity
#define WGS84_EP  0.0820944379    // second eccentricity

// UTM Parameters
#define UTM_K0    0.9996               // scale factor
#define UTM_FE    500000.0             // false easting
#define UTM_FN_N  0.0                  // false northing, northern hemisphere
#define UTM_FN_S  10000000.0           // false northing, southern hemisphere
#define UTM_E2    (WGS84_E*WGS84_E)    // e^2
#define UTM_E4    (UTM_E2*UTM_E2)      // e^4
#define UTM_E6    (UTM_E4*UTM_E2)      // e^6
#define UTM_EP2   (UTM_E2/(1-UTM_E2))  // e'^2



static inline char UTMLetterDesignator(double Lat)
{
  char LetterDesignator;

  if     ((84 >= Lat) && (Lat >= 72))  LetterDesignator = 'X';
  else if ((72 > Lat) && (Lat >= 64))  LetterDesignator = 'W';
  else if ((64 > Lat) && (Lat >= 56))  LetterDesignator = 'V';
  else if ((56 > Lat) && (Lat >= 48))  LetterDesignator = 'U';
  else if ((48 > Lat) && (Lat >= 40))  LetterDesignator = 'T';
  else if ((40 > Lat) && (Lat >= 32))  LetterDesignator = 'S';
  else if ((32 > Lat) && (Lat >= 24))  LetterDesignator = 'R';
  else if ((24 > Lat) && (Lat >= 16))  LetterDesignator = 'Q';
  else if ((16 > Lat) && (Lat >= 8))   LetterDesignator = 'P';
  else if (( 8 > Lat) && (Lat >= 0))   LetterDesignator = 'N';
  else if (( 0 > Lat) && (Lat >= -8))  LetterDesignator = 'M';
  else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
  else if ((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
  else if ((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
  else if ((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
  else if ((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
  else if ((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
  else if ((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
  else if ((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
  else if ((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
        // 'Z' is an error flag, the Latitude is outside the UTM limits
  else LetterDesignator = 'Z';
  return LetterDesignator;
}


static inline void LLtoUTM(const double Lat, const double Long,
                           double &UTMNorthing, double &UTMEasting,
                           std::string &UTMZone)
{
  double a = WGS84_A;
  double eccSquared = UTM_E2;
  double k0 = UTM_K0;

  double LongOrigin;
  double eccPrimeSquared;
  double N, T, C, A, M;

  // Make sure the longitude is between -180.00 .. 179.9
  double LongTemp = (Long+180)-static_cast<int>((Long+180)/360)*360-180;

  double LatRad = Lat*RADIANS_PER_DEGREE;
  double LongRad = LongTemp*RADIANS_PER_DEGREE;
  double LongOriginRad;
  int    ZoneNumber;

  ZoneNumber = static_cast<int>((LongTemp + 180)/6) + 1;

  if ( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
    ZoneNumber = 32;

        // Special zones for Svalbard
  if ( Lat >= 72.0 && Lat < 84.0 )
  {
    if (      LongTemp >= 0.0  && LongTemp <  9.0 ) ZoneNumber = 31;
    else if ( LongTemp >= 9.0  && LongTemp < 21.0 ) ZoneNumber = 33;
    else if ( LongTemp >= 21.0 && LongTemp < 33.0 ) ZoneNumber = 35;
    else if ( LongTemp >= 33.0 && LongTemp < 42.0 ) ZoneNumber = 37;
  }
        // +3 puts origin in middle of zone
  LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;
  LongOriginRad = LongOrigin * RADIANS_PER_DEGREE;

  // Compute the UTM Zone from the latitude and longitude
  char zone_buf[] = {0, 0, 0, 0};
  snprintf(zone_buf, sizeof(zone_buf), "%d%c", ZoneNumber, UTMLetterDesignator(Lat));
  UTMZone = std::string(zone_buf);

  eccPrimeSquared = (eccSquared)/(1-eccSquared);

  N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
  T = tan(LatRad)*tan(LatRad);
  C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
  A = cos(LatRad)*(LongRad-LongOriginRad);

  M = a*((1 - eccSquared/4 - 3*eccSquared*eccSquared/64
                - 5*eccSquared*eccSquared*eccSquared/256) * LatRad
               - (3*eccSquared/8 + 3*eccSquared*eccSquared/32
                  + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
               + (15*eccSquared*eccSquared/256
                  + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
               - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));

  UTMEasting = static_cast<double>
          (k0*N*(A+(1-T+C)*A*A*A/6
                 + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
           + 500000.0);

  UTMNorthing = static_cast<double>
          (k0*(M+N*tan(LatRad)
               *(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
                 + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));

  if (Lat < 0)
          {
            // 10000000 meter offset for southern hemisphere
            UTMNorthing += 10000000.0;
          }
}




void raw_gps_CB(const sensor_msgs::NavSatFix gps_msg)
{
		latiPoint_raw = gps_msg.latitude;
		longiPoint_raw = gps_msg.longitude;
}

void filtered_gps_CB(const sensor_msgs::NavSatFix gps_msg)
{
		latiPoint_filtered = gps_msg.latitude;
		longiPoint_filtered = gps_msg.longitude;
}

void joy_CB(const sensor_msgs::Joy joy_msg)
{
	if(joy_msg.buttons[collect_button_num]==1)
	{
		collect_request = true;

	}	
	if(joy_msg.buttons[end_button_num]==1)
	{
		ROS_INFO("end request registered");
		continue_collection = false;
	}
}




int main(int argc, char** argv)
{
     // Initialize node and time
		ros::init(argc, argv, "plot_gps_waypoints"); //initiate node called plot_gps_waypoints
		ros::NodeHandle n;
	
    // Get params
		ros::param::get("/outdoor_waypoint_nav/collect_button_num", collect_button_num);
		ros::param::get("/outdoor_waypoint_nav/end_button_num", end_button_num);
    	ros::param::get("/outdoor_waypoint_nav/filtered_coordinates_file", path_local_filtered);
    	ros::param::get("/outdoor_waypoint_nav/raw_coordinates_file", path_local_raw);
		ros::param::get("/outdoor_waypoint_nav/collect_button_sym", collect_button_sym);
		ros::param::get("/outdoor_waypoint_nav/end_button_sym", end_button_sym);
		ros::param::get("/outdoor_waypoint_nav/num_points", numPoints);
		ros::param::get("/outdoor_waypoint_nav/collection_time", collection_time);

    // Initialize time and set rates
		ros::Time::init();
        ros::Rate rate1(1);
		
     //Subscribe to topics
        ros::Subscriber sub_gps_raw = n.subscribe("/navsat/fix", 100, raw_gps_CB);
        ros::Subscriber sub_gps_filtered = n.subscribe("/outdoor_waypoint_nav/gps/filtered", 100, filtered_gps_CB);
		ros::Subscriber sub_joy = n.subscribe("/joy_teleop/joy", 100, joy_CB);


   //Read file path and create/open file
		std::string path_abs_raw =  ros::package::getPath("outdoor_waypoint_nav") + path_local_raw;	
        std::string path_abs_filtered =  ros::package::getPath("outdoor_waypoint_nav") + path_local_filtered;
		std::ofstream coordFile_raw (path_abs_raw.c_str());
        std::ofstream coordFile_filtered (path_abs_filtered.c_str());

		ROS_INFO("Saving raw coordinates to: %s", path_abs_raw.c_str());
        ROS_INFO("Saving filtered coordinates to: %s", path_abs_filtered.c_str());
		
	// Give instructions:
		ROS_INFO("Press %s button to collect and store waypoint.", collect_button_sym.c_str());
		ROS_INFO("Press %s button to end waypoint collection.", end_button_sym.c_str());
		std::cout << std::endl;

		if(coordFile_raw.is_open() && coordFile_filtered.is_open())
		{
	        while(continue_collection)
			{
				ros::spinOnce();
				if(collect_request == true)
				{

					//write waypoint
					std::cout << std::endl;
	                ROS_INFO("Collecting new set of waypoints...");
					numWaypoint_clusters++;

	                // collect one filtered point            
	                LLtoUTM(latiPoint_filtered, longiPoint_filtered, utmY_filtered, utmX_filtered, utmZone);
	                coordFile_filtered << std::fixed << std::setprecision(8) << utmX_filtered << " " << utmY_filtered << std::endl;
					
	                // collect 30 raw points at 5 hz
	                for(int i=0; i<numPoints; i++)
	                {
	                    ros::spinOnce();
	                    LLtoUTM(latiPoint_raw, longiPoint_raw, utmY_raw, utmX_raw, utmZone);
	                    coordFile_raw << std::fixed << std::setprecision(8) << utmX_raw << " " << utmY_raw << std::endl;
						numWaypoints++;
						ROS_INFO("Collected GPS Point Number %d.", numWaypoints);
	                    ros::Duration(collection_time).sleep(); 
	                }
					
					ROS_INFO("You have collected another waypoint cluster!");
					ROS_INFO("Press %s button to collect and store another waypoint.", collect_button_sym.c_str());
					ROS_INFO("Press %s button to end waypoint collection.", end_button_sym.c_str());
					collect_request = false; //reset
				}
				else{}
			rate1.sleep();
			}
		}
		else
		{
			ROS_ERROR("Unable to open file.");
			ROS_INFO("Exiting..");
		}
		
		ROS_INFO("Closed waypoint files, you have collected %d waypoint clusters and %d waypoints", numWaypoint_clusters, numWaypoints);
		coordFile_raw.close();
        coordFile_filtered.close();
		ROS_INFO("Ending node...");
		ros::shutdown();

        return 0;
}
