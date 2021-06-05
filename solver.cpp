# include <cppad/ipopt/solve.hpp>
# include <ros/ros.h>
# include <geometry_msgs/Twist.h>
# include <math.h>
# include <turtlesim/Pose.h>
turtlesim::Pose pose_runner;
turtlesim::Pose pose_obstacle;
turtlesim::Pose pose_final_goal;
turtlesim::Pose pose_temp_goal;

namespace {
     using CppAD::AD;

     class FG_eval {
     public:
          typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;
          void operator()(ADvector& fg, const ADvector& x)
          {     assert( fg.size() == 2 );
               assert( x.size()  == 2 );

               // Fortran style indexing
               AD<double> x1 = x[0];
               AD<double> x2 = x[1];
               // f(x)
               fg[0] = (x1-pose_final_goal.x)*(x1-pose_final_goal.x) + (x2-pose_final_goal.y)*(x2-pose_final_goal.y);
               // g_1 (x)
               fg[1] = (x1-pose_obstacle.x)*(x1-pose_obstacle.x) + (x2-pose_obstacle.y)*(x2-pose_obstacle.y);
               // g_2 (x)

               return;
          }
     };
}

bool get_started(void)
{
     bool ok = true;
     size_t i;
     typedef CPPAD_TESTVECTOR( double ) Dvector;

     // number of independent variables (domain dimension for f and g)
     size_t nx = 2;
     // number of constraints (range dimension for g)
     size_t ng = 1;
     // initial value of the independent variables
     Dvector xi(nx);
     xi[0] = pose_runner.x;
     xi[1] = pose_runner.y;
     // lower and upper limits for x
     Dvector xl(nx), xu(nx);
     for(i = 0; i < nx; i++)
     {
          xl[i] = pose_runner.x-0.2;
          xu[i] = pose_runner.x+0.2;
     }
     // lower and upper limits for g
     Dvector gl(ng), gu(ng);
     gl[0] = 2.0;     gu[0] = 1.0e19;


     // object that computes objective and constraints
     FG_eval fg_eval;

     // options
     std::string options;
     // turn off any printing
     options += "Integer print_level  0\n";
     options += "String  sb          yes\n";
     // maximum number of iterations
     options += "Integer max_iter     10\n";
     // approximate accuracy in first order necessary conditions;
     // see Mathematical Programming, Volume 106, Number 1,
     // Pages 25-57, Equation (6)
     options += "Numeric tol          1e-6\n";
     // derivative testing
     options += "String  derivative_test            second-order\n";
     // maximum amount of random pertubation; e.g.,
     // when evaluation finite diff
     options += "Numeric point_perturbation_radius  0.\n";

     // place to return solution
     CppAD::ipopt::solve_result<Dvector> solution;
     // solve the problem
     CppAD::ipopt::solve<Dvector, FG_eval>(
          options, xi, xl, xu, gl, gu, fg_eval, solution
     );
     pose_temp_goal.x = solution.x[0];
     pose_temp_goal.y = solution.x[1];
     //
     // Check some of the solution values
     //
     ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
     //std::cout<<solution.x[0];
     return ok;
}
float distance(float x1, float x2, float y1, float y2)
{
     //std::cout<<x1<<" "<<x2<<" "<<y1<<" "<<y2<<" ";
     return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
float steering_angle()
{
     return atan2(pose_temp_goal.y - pose_runner.y, pose_temp_goal.x - pose_runner.x);
}
float angular_vel(int constant=1)
{
     std::cout<<steering_angle()<<" "<<pose_runner.theta<<" ";
     return constant * (steering_angle() - pose_runner.theta);
}
void pose_runner_Callback(const turtlesim::Pose::ConstPtr& msg)
{
     pose_runner.x = msg->x;
     pose_runner.y = msg->y;
}
void pose_obs_Callback(const turtlesim::Pose::ConstPtr& msg)
{
     pose_obstacle.x = msg->x;
     pose_obstacle.y = msg->y;
}

int main(int argc, char **argv)
{
     ros::init(argc, argv, "turtlepath_node");
     ros::NodeHandle n;
     ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1000);
     int freq = 5;
     ros::Rate loop_rate(5);
     geometry_msgs::Twist vel;
     ros::Subscriber sub = n.subscribe("turtle2/pose", 1000, pose_runner_Callback);
     ros::Subscriber sub2 = n.subscribe("turtle1/pose", 1000, pose_obs_Callback);
     pose_final_goal.x = pose_final_goal.y = 10.0;
     while (ros::ok())
     {
          get_started();
          vel.linear.x = distance(pose_runner.x, pose_temp_goal.x, pose_runner.y, pose_temp_goal.y) * freq;
          vel.linear.y = vel.linear.z = vel.angular.x = vel.angular.y = 0;
          vel.angular.z = angular_vel();
          //std::cout<<" "<<pose_runner.x<<" "<<pose_temp_goal.x<<" ";
          chatter_pub.publish(vel);
          if(distance(pose_runner.x,pose_final_goal.x,pose_runner.y,pose_final_goal.y)<0.5) break;
          ros::spinOnce();
          loop_rate.sleep();
     }
     vel.linear.x = vel.angular.z = 0;
     chatter_pub.publish(vel);
     return 0;
}
