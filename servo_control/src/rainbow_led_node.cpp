#include <ros/ros.h>
#include <string>
#include <led_msgs/SetLEDs.h>
#include <led_msgs/LEDStateArray.h>
#include <cmath>


typedef struct {
    double r;       // ∈ [0, 1]
    double g;       // ∈ [0, 1]
    double b;       // ∈ [0, 1]
} rgb;

typedef struct {
    double h;       // ∈ [0, 360]
    double s;       // ∈ [0, 1]
    double v;       // ∈ [0, 1]
} hsv;


rgb hsv2rgb(hsv HSV);

//main function for the opencv person detector node, which creates the opencv person detector object and spinnnnns
int main(int argc, char **argv)
{
    //led_msgs::SetLEDs::Request&
    ros::init(argc, argv, "rainbow_led_node");
    ros::NodeHandle nh;

    int num_leds;
    nh.param("num_leds", num_leds, 2);
    ROS_INFO("Number of LEDs is %d", num_leds);

    ros::service::waitForService("/led/set_leds");
    ROS_INFO("LED driver found");
    
    // Create a ROS service client
    ros::ServiceClient client = nh.serviceClient<led_msgs::SetLEDs>("/led/set_leds");

    // Create a service message
    led_msgs::SetLEDs srv;

    

    // Set the initial iteration value
    int iteration = 0;
    float duration = 10.0f;
    int rate_hz = 10;

    // Loop at 20Hz until the node is shut down
    ros::Rate rate(rate_hz);

    while(ros::ok()) {
        // Clear the LED array for each iteration
        srv.request.leds.clear();

        for (int i = 0; i < num_leds; ++i)
        {
            led_msgs::LEDState led_msg;
            led_msg.index = i;

            // Calculate HSV values for rainbow effect
    	    float normalized_iteration = static_cast<float>(iteration) / (rate_hz * duration);

            float hue = fmod(normalized_iteration + i / static_cast<float>(num_leds), 1.0);

	        hsv HSV = (hsv){.h = hue*360, .s = 1.0, .v = 1.0};
            rgb RGB = hsv2rgb(HSV);
		
            led_msg.r = static_cast<int>(RGB.r * 255.0);
            led_msg.g = static_cast<int>(RGB.g * 255.0);
            led_msg.b = static_cast<int>(RGB.b * 255.0);
            srv.request.leds.push_back(led_msg);
        }

        // Call the service
        if (!client.call(srv))
        {
            ROS_ERROR("Failed to call service set_leds");
        }
        
        // Increment the iteration for the next cycle
        iteration++;

        ros::spinOnce();

        rate.sleep();
    }

    return 0;

}


rgb hsv2rgb(hsv HSV)
{
    rgb RGB;
    double H = HSV.h, S = HSV.s, V = HSV.v,
            P, Q, T,
            fract;

    (H == 360.)?(H = 0.):(H /= 60.);
    fract = H - floor(H);

    P = V*(1. - S);
    Q = V*(1. - S*fract);
    T = V*(1. - S*(1. - fract));

    if      (0. <= H && H < 1.)
        RGB = (rgb){.r = V, .g = T, .b = P};
    else if (1. <= H && H < 2.)
        RGB = (rgb){.r = Q, .g = V, .b = P};
    else if (2. <= H && H < 3.)
        RGB = (rgb){.r = P, .g = V, .b = T};
    else if (3. <= H && H < 4.)
        RGB = (rgb){.r = P, .g = Q, .b = V};
    else if (4. <= H && H < 5.)
        RGB = (rgb){.r = T, .g = P, .b = V};
    else if (5. <= H && H < 6.)
        RGB = (rgb){.r = V, .g = P, .b = Q};
    else
        RGB = (rgb){.r = 0., .g = 0., .b = 0.};

    return RGB;
}