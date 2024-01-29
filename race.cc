#include <ignition/msgs/twist.pb.h>
#include <ignition/msgs/laserscan.pb.h>
#include <ignition/transport/Node.hh>

std::string topic_pub = "/vehicle/cmd_vel";
ignition::transport::Node node;
auto publisher = node.Advertise<ignition::msgs::Twist>(topic_pub);

double kp = 1.0;
double max_range = 3;


void callback(const ignition::msgs::LaserScan &message) {
    int index = 0;
    double x1, x2 = 0;
    double y1, y2 = 0;

    int max_index = 0;
    double max_distance = 0;

    double angles[270];
    double ranges[270];

    for (int angle_deg = 45; angle_deg < 315; angle_deg++) {
        double range = message.ranges(angle_deg);
        if (range < max_range) {
            index++;

            double angle_rad = static_cast<double>(angle_deg) / 180.0 * M_PI;
            angles[index] = angle_rad;
            ranges[index] = range;

            x1 = x2;
            y1 = y2;
            x2 = cos(angle_rad) * range;
            y2 = sin(angle_rad) * range;

            if (index > 1) {
                double distance = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
                if (distance > max_distance) {
                    max_distance = distance;
                    max_index = index;
                }
            }
        }
    }
    x1 = cos(angles[max_index]) * ranges[max_index];
    y1 = sin(angles[max_index]) * ranges[max_index];
    x2 = cos(angles[max_index - 1]) * ranges[max_index - 1];
    y2 = sin(angles[max_index - 1]) * ranges[max_index - 1];

    double xc = (x1 + x2) / 2;
    double yc = (y1 + y2) / 2;

    double theta = atan2(yc, xc);
    double error = theta - M_PI;
    error = atan2(sin(error), cos(error));

    ignition::msgs::Twist data;
    data.mutable_linear()->set_x(1);
    data.mutable_angular()->set_z(kp * error);
    publisher.Publish(data);
}


int main(int argc, char **argv) {
    std::string topic = "/vehicle/lidar";
    if (!node.Subscribe(topic, callback)) {
        std::cerr << "Error subscribing to topic [" << topic << "]" << std::endl;
        return -1;
    }
    std::cout << "Success in subscribing to topic [" << topic << "]" << std::endl;

    ignition::transport::waitForShutdown();

    return 0;
}