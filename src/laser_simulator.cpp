#include "rclcpp/rclcpp.hpp"
#include "tf2/utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "mobile_robot_simulator/laser_simulator.h"

LaserScannerSimulator::LaserScannerSimulator(rclcpp::Node::SharedPtr node)
  : logger_(node->get_logger()), clock_(node->get_clock()), buffer_(clock_), tl(buffer_)
{
    node_ = node;
    // get parameters
    get_params();
    laser_pub = node_->create_publisher<sensor_msgs::msg::LaserScan>("scan",10); // scan publisher
    map_service_ = node_->create_client<nav_msgs::srv::GetMap>("map_server/map");
    // get map
    get_map();
    RCLCPP_INFO(logger_, "Initialized laser scanner simulator");
}

LaserScannerSimulator::~LaserScannerSimulator()
{
    if (is_running) stop();
}

void LaserScannerSimulator::get_params()
{
    //laser parameters - defaults are appriximately that of a Sick S300
    l_frame = node_->declare_parameter("laser_frame_id", "base_link");
    l_fov = node_->declare_parameter("laser_fov", 1.5*M_PI);
    l_beams = node_->declare_parameter("laser_beam_count", 541);
    l_max_range = node_->declare_parameter("laser_max_range", 30.0);
    l_min_range = node_->declare_parameter("laser_min_range", 0.05);
    l_frequency = node_->declare_parameter("laser_frequency", 10.0);
    // noise model parameters (see Probabilistic Robotics eq. 6.12)
    use_noise_model = node_->declare_parameter("apply_noise", true);
    sigma_hit = node_->declare_parameter("sigma_hit", 0.005);
    lambda_short = node_->declare_parameter("lambda_short", 2.0);
    z_mix[0] = node_->declare_parameter("z_hit", 0.995);
    z_mix[1] = node_->declare_parameter("z_short", 0.0);
    z_mix[2] = node_->declare_parameter("z_max", 0.005);
    z_mix[3] = node_->declare_parameter("z_rand", 0.0);
    // update the noise model internally
    set_noise_params(use_noise_model, sigma_hit, lambda_short, z_mix[0], z_mix[1], z_mix[2], z_mix[3]);
}

void LaserScannerSimulator::start()
{
    using std::chrono_literals::operator""s;
    loop_timer = node_->create_timer(1.0s/l_frequency, std::bind(&LaserScannerSimulator::update_loop, this));
    is_running = true;
    RCLCPP_INFO(logger_, "Started laser scanner simulator update loop");
}

void LaserScannerSimulator::stop()
{
    loop_timer->cancel();
    is_running = false;
    RCLCPP_INFO(logger_, "Stopped laser scanner simulator");
}

void LaserScannerSimulator::update_loop()
{
    // If we don't have a map, try to get one
    if (!have_map) get_map();
    // first, get the pose of the laser in the map frame
    double l_x, l_y, l_theta;
    get_laser_pose(&l_x,&l_y,&l_theta);
    //ROS_INFO_STREAM_THROTTLE(2,"x: " << l_x << " y: " << l_y << " theta: " <<  l_theta);
    update_scan(l_x,l_y,l_theta);
    output_scan.header.stamp = clock_->now();
    laser_pub->publish(output_scan);
}

void LaserScannerSimulator::get_map()
{
    RCLCPP_INFO(logger_, "Requesting the map");
    auto req = std::make_shared<nav_msgs::srv::GetMap::Request>();
    nav_msgs::srv::GetMap::Response resp;
    auto cb = [this](rclcpp::Client<nav_msgs::srv::GetMap>::SharedFuture future){
      assert(future.valid());
      auto result = future.get();
      map = result->map;
      RCLCPP_INFO_STREAM(logger_, "Got a " << map.info.width << "x" << map.info.height << " map with resolution " << map.info.resolution);
      have_map = true;
    };
    auto future = map_service_->async_send_request(req, cb);

    // TODO(Ramon): How can I know when the service failed?
    // https://github.com/ros2/ros2/issues/921
    // RCLCPP_WARN_THROTTLE(logger_, *clock_, 10000, "No map received - service '/static_map' not available (will publish only max_range)");
    // have_map = false;
}

void LaserScannerSimulator::set_laser_params(std::string frame_id, double fov, unsigned int beam_count, double max_range, double min_range, double update_frequency)
{
    l_frame = frame_id;
    l_fov = fov;
    l_beams = beam_count;
    l_max_range = max_range;
    l_min_range = min_range;
    l_frequency = update_frequency;
    RCLCPP_INFO(logger_, "Updated parameters of simulated laser");
}

void LaserScannerSimulator::get_laser_pose(double * x, double * y, double * theta)
{
    rclcpp::Time now = clock_->now();
    tf2::Stamped<tf2::Transform> transf;
    try
    {
        auto tf = buffer_.lookupTransform("map", l_frame, now, rclcpp::Duration::from_seconds(1));
        tf2::convert(tf, transf);
    }
    catch (const tf2::TransformException& ex)
    {
        RCLCPP_ERROR(logger_, "%s",ex.what());
        *x = 0.0; *y = 0.0, *theta * 0.0;
        return;
    }
    *x = transf.getOrigin().getX();
    *y = transf.getOrigin().getY();
    *theta = tf2::getYaw(transf.getRotation());
}

void LaserScannerSimulator::update_scan(double x, double y, double theta)
{
    //timing
    //rclcpp::Time start = rclcpp::Time::now();
    // laser params
    output_scan.angle_min = -l_fov/2.0;
    output_scan.angle_max = l_fov/2.0;
    output_scan.angle_increment = l_fov/l_beams;
    output_scan.range_min = l_min_range;
    output_scan.range_max = l_max_range+0.001;
    output_scan.time_increment = (1.0/l_frequency) / l_beams;
    output_scan.scan_time = (1.0/l_frequency);

    std::vector<float> ranges;
    double this_range;
    double this_ang;
    // header
    output_scan.header.frame_id = l_frame;
    output_scan.header.stamp = clock_->now();

    for (unsigned int i=0; i<=l_beams; i++)
    {
        if (!have_map)
        {
            ranges.push_back((float)l_max_range);
            continue;
        }
        this_ang = theta + output_scan.angle_min + i*output_scan.angle_increment;
        this_range = find_map_range(x,y,this_ang);
        //ROS_INFO_STREAM_THROTTLE(1,"x: " << x << " y: " << y << " theta: " << this_ang);
        ranges.push_back(this_range);
    }
    output_scan.ranges = ranges;
    //ROS_INFO_STREAM_THROTTLE(2, "Simulated scan in " << (rclcpp::Time::now()-start).toSec() << "s");
}

double LaserScannerSimulator::find_map_range(double x, double y, double theta)
{
    // using "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
    // ======== initialization phase ========
    //ROS_INFO_STREAM_THROTTLE(1,"x,y,theta " << x << ", " << y << ", " << theta);
    double origin[2]; // u
    origin[0] = x;
    origin[1] = y;
    //ROS_INFO_STREAM_THROTTLE(1,"origin " << origin[0] << ", " << origin[1]);
    double dir[2]; // v
    dir[0] = cos(theta);
    dir[1] = sin(theta);
    //ROS_INFO_STREAM_THROTTLE(1,"dir " << dir[0] << ", " << dir[1]);
    int start_x, start_y;
    get_world2map_coordinates(x,y,&start_x,&start_y);
    //ROS_INFO_STREAM_THROTTLE(1,"start " << start_x << ", " << start_y);
    if (start_x<0 || start_y<0 || start_x >= map.info.width || start_y >= map.info.height)
    {
        //outside the map, find entry point
        double delta_x = abs(map.info.origin.position.x - x);
        double delta_y = abs(map.info.origin.position.y - y);
        double intersect_x = x + (dir[0] * delta_x);
        double intersect_y = y + (dir[1] * delta_y);
        get_world2map_coordinates(intersect_x,intersect_y,&start_x,&start_y);
    }
    int current[2]; // X, Y
    current[0] = start_x;
    current[1] = start_y;
    //ROS_INFO_STREAM_THROTTLE(1,"current " << current[0] << ", " << current[1]);

    int step[2]; // stepX, stepY
    double tMax[2]; // tMaxX, tMaxY
    double tDelta[2]; // tDeltaX, tDeltaY

    double voxel_border[2];
    get_map2world_coordinates(current[0], current[1], &voxel_border[0], &voxel_border[1]);
    voxel_border[0] -= 0.5 * map.info.resolution; //this is the lower left voxel corner
    voxel_border[1] -= 0.5 * map.info.resolution; //this is the lower left voxel corner
    //ROS_INFO_STREAM_THROTTLE(1,"voxel_border " << voxel_border[0] << ", " << voxel_border[1]);

    for (unsigned int i=0;i<2;i++) // for each dimension (x,y)
    {
        // determine step direction
        if (dir[i] > 0.0) step[i] = 1;
        else if (dir[i] < 0.0) step[i] = -1;
        else step[i] = 0;

        // determine tMax, tDelta
        if (step[i] != 0)
        {
            // corner point of voxel (in direction of ray)
            if (step[i] == 1)
            {
                voxel_border[i] += (float) (step[i] * map.info.resolution);
            }
            // tMax - voxel boundary crossing
            tMax[i] = (voxel_border[i] - origin[i]) / dir[i];
            // tDelta - distance along ray so that veritcal/horizontal component equals one voxel
            tDelta[i] = map.info.resolution / fabs(dir[i]);
        }
        else
        {
            tMax[i] = std::numeric_limits<double>::max();
            tDelta[i] = std::numeric_limits<double>::max();
        }

    }
    //ROS_INFO_STREAM_THROTTLE(1,"step " << step[0] << ", " << step[1]);
    //ROS_INFO_STREAM_THROTTLE(1,"tMax " << tMax[0] << ", " << tMax[1]);
    //ROS_INFO_STREAM_THROTTLE(1,"tDelta " << tDelta[0] << ", " << tDelta[1]);

    //ROS_DEBUG_STREAM("Starting at index " << start_x << "," << start_y);

    // ======== incremental traversal ========
    while (true)
    {
        // advance
        unsigned int dim; // X or Y direction
        if (tMax[0] < tMax[1]) dim = 0;
        else dim = 1;
        // advance one voxel
        current[dim] += step[dim];
        tMax[dim] += tDelta[dim];
        //ROS_DEBUG_STREAM("Examining index " << current[0] << "," << current[1]);
        // are we outside the map?
        if (current[0] < 0 || current[0] >= map.info.width || current[1] < 0 || current[1] >= map.info.height)
        {
            return l_max_range;
        }
        // determine current range
        double current_range = sqrt(pow((current[0] - start_x),2) + pow((current[1] - start_y),2)) * map.info.resolution;
        // are we at max range?
        if (current_range > l_max_range) return l_max_range;
        else {
            int occ = get_map_occupancy(current[0],current[1]);
            if (occ >= 60) // current cell is occupied
            {
                // are we below the minimum range of the laser scanner?
                if (current_range < l_min_range) continue;
                // if not, return the current range, with noise (optionally) applied
                if (use_noise_model) return apply_range_noise(current_range);
                else return current_range;
            }
        }
    } // end while
}

double LaserScannerSimulator::apply_range_noise(double range_reading)
{
    // using Measurement model of laser range scanner, following method from chapter 6.3.1 of Probabilistic Robotics
    double noise_modifier = selector(rand_gen);
    // hit: gaussian noise on reading
    if (noise_modifier < z_mix[0]) return range_reading + p_hit(rand_gen);
    // short: short readings, exponential function
    else if (noise_modifier <  z_mix[0] +  z_mix[1]) return p_short(rand_gen);
    // rand: random reading, uniform distribution
    else if (noise_modifier < z_mix[0] +  z_mix[1] +  z_mix[3]) return p_rand(rand_gen);
    // max: no detection, max reading
    else return l_max_range;
}

void LaserScannerSimulator::set_noise_params(bool use_model, double sigma_hit_reading, double lambda_short_reading, double z_hit, double z_short, double z_max, double z_rand)
{
    use_noise_model = use_model;
    sigma_hit = sigma_hit_reading;
    lambda_short = lambda_short_reading;
    z_mix[0] = z_hit;
    z_mix[1] = z_short;
    z_mix[2] = z_max;
    z_mix[3] = z_rand;
    // check that weights are normalized
    double z_sum = z_mix[0]+z_mix[1]+z_mix[2]+z_mix[3];
    if (z_sum != 1)
    {
        RCLCPP_WARN_STREAM(logger_, "Noise model weighting sums not normalized (sum is " << z_sum << "), normalizing them");
        z_mix[0] = z_mix[0]/z_sum;
        z_mix[1] = z_mix[1]/z_sum;
        z_mix[2] = z_mix[2]/z_sum;
        z_mix[3] = z_mix[3]/z_sum;
        RCLCPP_WARN_STREAM(logger_, "After normalization - z_hit " << z_mix[0] << ", z_short " << z_mix[1] << ", z_max " << z_mix[2] << ", z_rand " << z_mix[3]);

    }
    // reset distributions
    p_hit = std::normal_distribution<double>(0.0,sigma_hit);
    p_short = std::exponential_distribution<double>(lambda_short);
    p_rand = std::uniform_real_distribution<double>(0.0,l_max_range);
    selector = std::uniform_real_distribution<double>(0.0,1.0);
}

void LaserScannerSimulator::get_world2map_coordinates(double world_x, double world_y, int * map_x, int * map_y)
{
    *map_x = (int) (std::floor((world_x - map.info.origin.position.x) / map.info.resolution));
    *map_y = (int) (std::floor((world_y - map.info.origin.position.y) / map.info.resolution));
    //ROS_INFO_STREAM_THROTTLE(1, "world2map - x: " << world_x << " map_x: " << *map_x);
}

void LaserScannerSimulator::get_map2world_coordinates(int map_x, int map_y, double * world_x, double * world_y)
{
    *world_x = (map_x * map.info.resolution) + map.info.origin.position.x + 0.5*map.info.resolution;
    *world_y = (map_y * map.info.resolution) + map.info.origin.position.y + 0.5*map.info.resolution;
    //ROS_INFO_STREAM_THROTTLE(1, "map2world - map_x: " << map_x << " x: " << *world_x);
}

int LaserScannerSimulator::get_map_occupancy(int x, int y)
{
    //ROS_DEBUG_STREAM("x: " << x << " y: " << y << " index: " <<  y*map.info.width + x);
    return map.data[y*map.info.width + x];
}
