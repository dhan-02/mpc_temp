# include <cppad/ipopt/solve.hpp>
# include <cppad/cppad.hpp>
# include <ros/ros.h>
# include <geometry_msgs/Twist.h>
# include <math.h>
# include <turtlesim/Pose.h>
turtlesim::Pose pose_runner;
turtlesim::Pose pose_obstacle;
turtlesim::Pose pose_final_goal;

int look_ahead = 5;
float dt = 0.1;
float delta = 0.1;
float vel_max = 3.0;
float lin_vel;
float ang_vel;

float distance(float x1,float x2,float y1,float y2);

namespace {
     using CppAD::AD;
     using CppAD::Value;
     using CppAD::Var2Par;
     class FG_eval {
     public:
          typedef CPPAD_TESTVECTOR( AD<double> ) ADvector;
          void operator()(ADvector& fg, const ADvector& x)
          {    assert( fg.size() == 4*(look_ahead-1)+1 );
               assert( x.size()  == look_ahead*5 );
               for(int i = 1; i < look_ahead; i++)
               {

                    // Fortran style indexing
                    AD<double> x1 = x[i-1];                 // x_current
                    AD<double> x2 = x[look_ahead+i-1];      // y_current
                    AD<double> x3 = x[2*look_ahead+i-1];    // theta_current
                    AD<double> x4 = x[3*look_ahead+i-1];    // v_current
                    AD<double> x5 = x[4*look_ahead+i-1];    // w_current
                    AD<double> x6 = x[i];                   //x_next
                    AD<double> x7 = x[look_ahead+i];        //y_next
                    AD<double> x8 = x[2*look_ahead+i];      //theta_next
                    AD<double> x9 = x[3*look_ahead+i];      //v_next
                    AD<double> x10 = x[4*look_ahead+i];     //w_next
                    // f(x)
                    fg[0] += distance(Value(Var2Par(x6)),pose_final_goal.x,Value(Var2Par(x7)),pose_final_goal.y);

                    // g_1 (x)
                    fg[i] = x6 - (x1 + (x4 * CppAD::cos(x3) * dt));
                    fg[look_ahead+i-1] = x7 - (x2 + (x4 * CppAD::sin(x3) * dt));
                    fg[2*look_ahead+i-2] = x8 - (x3 + x5 * dt);
                    fg[3*look_ahead+i-3] = distance(Value(Var2Par(x6)),pose_obstacle.x,Value(Var2Par(x7)),pose_obstacle.y);

                    // g_2 (x)

                    return;
               }
          }
     };
}

bool get_started(void)
{
     bool ok = true;
     size_t i;
     typedef CPPAD_TESTVECTOR( double ) Dvector;

     // number of independent variables (domain dimension for f and g)
     size_t nx = look_ahead*5;
     // number of constraints (range dimension for g)
     size_t ng = 4*(look_ahead-1);
     // initial value of the independent variables
     Dvector xi(nx),xl(nx),xu(nx);
     xi[0] = pose_runner.x;
     xl[0] = pose_runner.x;
     xu[0] = pose_runner.x;
     for (int i = 1; i < look_ahead; i++)
     {
          xi[i] = pose_runner.x;
          xl[i] = -12;   // Boundaries of turtlesim wall
          xu[i] = 12;
     }

     xi[look_ahead] = pose_runner.y;
     xl[look_ahead] = pose_runner.y;
     xu[look_ahead] = pose_runner.y;
     for (int i = look_ahead + 1; i < 2*look_ahead; i++)
     {
          xi[i] = pose_runner.y;
          xl[i] = -12;
          xu[i] = 12;
     }

     xi[2*look_ahead] = pose_runner.theta;
     xl[2*look_ahead] = pose_runner.theta;
     xu[2*look_ahead] = pose_runner.theta;
     for (int i = 2*look_ahead + 1; i < 3*look_ahead; i++)
     {
          xi[i] = pose_runner.theta;
          xl[i] = -3.14;
          xu[i] = +3.14;
     }

     xi[3*look_ahead] = pose_runner.linear_velocity;
     xl[3*look_ahead] = pose_runner.linear_velocity;
     xu[3*look_ahead] = pose_runner.linear_velocity;
     for (int i = 3*look_ahead + 1; i < 4*look_ahead; i++)
     {
          xi[i] = pose_runner.linear_velocity;
          xl[i] = 0.0;
          xu[i] = vel_max;
     }

     xi[4*look_ahead] = pose_runner.angular_velocity;
     xl[4*look_ahead] = pose_runner.angular_velocity;
     xu[4*look_ahead] = pose_runner.angular_velocity;
     for (int i = 4*look_ahead + 1; i < 5*look_ahead; i++)
     {
          xi[i] = pose_runner.angular_velocity;
          xl[i] = -1;
          xu[i] = 1;
     }
     // lower and upper limits for g
     Dvector gl(ng), gu(ng);
     //gl[0] = 2.0;     gu[0] = 1.0e19;
     for(i = 0; i < ng; i++)
     {
          if(i<4)
          {
                gl[i] = -delta;
                gu[i] = delta;
          }
          else if(i<8)
          {
                gl[i] = -delta;
                gu[i] = delta;
          }
          else if(i<12)
          {
                gl[i] = -delta;
                gu[i] = delta;

          }
          else if(i<16)
          {
                gl[i] = 3;     // Min distance from obstacle
                gu[i] = 1.0e19;

          }
     }

     // object that computes objective and constraints
     FG_eval fg_eval;

     // options
     std::string options;

     // Uncomment this if you'd like more print information
     options += "Integer print_level  0\n";

     // Disables printing IPOPT creator banner
     options += "String  sb          yes\n";

     // NOTE: Setting sparse to true allows the solver to take advantage
     // of sparse routines, this makes the computation MUCH FASTER. If you
     // can uncomment 1 of these and see if it makes a difference or not but
     // if you uncomment both the computation time should go up in orders of
     // magnitude.
     options += "Sparse  true        forward\n";
     options += "Sparse  true        reverse\n";

     // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
     // Change this as you see fit.
     options += "Numeric max_cpu_time          0.5\n";
     //options += "Integer max_iter              100\n";

     // place to return solution
     CppAD::ipopt::solve_result<Dvector> solution;
     // solve the problem
     CppAD::ipopt::solve<Dvector, FG_eval>(
          options, xi, xl, xu, gl, gu, fg_eval, solution
     );

     // Check some of the solution values
     ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;
     lin_vel = solution.x[3*look_ahead+1];
     ang_vel = solution.x[4*look_ahead+1];
     return ok;
}
float distance(float x1, float x2, float y1, float y2)
{
     //std::cout<<x1<<" "<<x2<<" "<<y1<<" "<<y2<<" ";
     return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
void pose_runner_Callback(const turtlesim::Pose::ConstPtr& msg)
{
     pose_runner.x = msg->x;
     pose_runner.y = msg->y;
     pose_runner.theta = msg->theta;
     pose_runner.linear_velocity = msg->linear_velocity;
     pose_runner.angular_velocity = msg->angular_velocity;
}
void pose_obs_Callback(const turtlesim::Pose::ConstPtr& msg)
{
     pose_obstacle.x = msg->x;
     pose_obstacle.y = msg->y;
     pose_obstacle.theta = msg->theta;
     pose_obstacle.linear_velocity = msg->linear_velocity;
     pose_obstacle.angular_velocity = msg->angular_velocity;
}
void input()
{
     std::cout<<"**************** WElCOME ********************\n\n";
     std::cout<<"Enter Goal x: "; std::cin>>pose_final_goal.x; std::cout<<"\n\n";
     std::cout<<"Enter Goal y: "; std::cin>>pose_final_goal.y; std::cout<<"\n\n";
     std::cout<<"Processing.........\n\n";
}
int main(int argc, char **argv)
{
     ros::init(argc, argv, "turtlepath_node");
     char key = 'Y';
     ros::NodeHandle n;
     ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 1000);
     ros::Rate loop_rate(100);
     geometry_msgs::Twist vel;
     ros::Subscriber sub = n.subscribe("turtle2/pose", 1000, pose_runner_Callback);
     ros::Subscriber sub2 = n.subscribe("turtle1/pose", 1000, pose_obs_Callback);
     while(key!='N'||key!='n')
     {
          input();
          while (ros::ok())
          {
               get_started();
               vel.linear.x = lin_vel;
               vel.angular.z = ang_vel;
               vel.linear.y = vel.linear.z = vel.angular.x = vel.angular.y = 0;
               chatter_pub.publish(vel);
               if(distance(pose_runner.x,pose_final_goal.x,pose_runner.y,pose_final_goal.y)<0.5)
               {
                     vel.linear.x = vel.angular.z = 0;
                     chatter_pub.publish(vel);
                     break;
               }
               ros::spinOnce();
               loop_rate.sleep();
          }
          std::cout<<"Goal Reached\n\n****************************************\n\n";
          std::cout<<"Do you want to go again (Y/N) : "; std::cin>>key; std::cout<<"\n\n";
     }

     return 0;
}
