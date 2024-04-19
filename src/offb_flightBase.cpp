#include <offb_flightBase.h>

namespace AutoFlight
{
    flightBase::flightBase(const ros::NodeHandle &nh) : nh_(nh)
    {
        // 获取起飞高度
        if (not this->nh_.getParam("autonomous_flight/takeoff_height", this->takeoffHgt_))
        {
            this->takeoffHgt_ = 1.0;
            cout << "[AutoFlight]: No takeoff height param found. Use default: 1.0 m." << endl;
        }
        else
        {
            cout << "[AutoFlight]: Takeoff Height: " << this->takeoffHgt_ << "m." << endl;
        }

        // Subscriber
        /*
         *mavros/state  与飞行控制器的连接状态和飞行控制器模式。
         *mavros/local_position/pose  无人机在 ENU 坐标系中的本地位置和方向。
         */
        this->stateSub_ = this->nh_.subscribe<mavros_msgs::State>("/mavros/state", 1000, &flightBase::stateCB, this);
        this->currPosSub_ = this->nh_.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &flightBase::currPoseCB, this);

        // Service client
        /*
         *mavros/cmd/arming解锁或上锁无人机的电机（更改电机状态）。
         *mavros/set_mode设置控制器的飞行模式。通常用于设置 OFFBOARD（离岸）模式，以接受来自机载计算机的指令。
         */
        this->armClient_ = this->nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
        this->setModeClient_ = this->nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

        // Publisher
        /*
         * /mavros/setpoint_position/local设置无人机的目标位置和偏航(在ENU坐标系中)。
         */
        this->posePub_ = this->nh_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1000);   // 发送目标位置
        this->statePub_ = this->nh_.advertise<tracking_controller::Target>("/autonomous_flight/target_state", 1000); //

        this->currPoseReceived_ = false;
        this->mavrosStateReceived_ = false;
        ros::Rate r(10);

        // 检查movros主题是否准备成功
        while (ros::ok() and not(this->currPoseReceived_ and this->mavrosStateReceived_))
        {
            ros::spinOnce();
            r.sleep();
        }
        cout << "[AutoFlight]: Odom and mavros topics are ready." << endl;

        // 创建目标位置发送线程
        this->targetPubWorker_ = std::thread(&flightBase::publishTarget, this);
        this->targetPubWorker_.detach();
    }

    // 目标位置发送线程
    void flightBase::publishTarget()
    {
        ros::Rate r(200);

        /*
         *warmup（开启offboard模式前必须已经开始流式传输设定值。
         *否则，模式切换将被拒绝。 这里的100 可以被设置为任意数）
         */
        for (int i = 100; ros::ok() && i > 0; --i)
        {
            this->poseTgt_.header.stamp = ros::Time::now();
            this->posePub_.publish(this->poseTgt_);
        }

        mavros_msgs::SetMode offboardMode;
        offboardMode.request.custom_mode = "OFFBOARD";
        mavros_msgs::CommandBool armCmd;
        armCmd.request.value = true;
        ros::Time lastRequest = ros::Time::now();
        while (ros::ok())
        {
            if (this->mavrosState_.mode != "OFFBOARD" && (ros::Time::now() - lastRequest > ros::Duration(5.0)))
            {
                if (this->setModeClient_.call(offboardMode) && offboardMode.response.mode_sent)
                {
                    cout << "[AutoFlight]: Offboard mode enabled." << endl;
                }
                lastRequest = ros::Time::now();
            }
            else
            {
                if (!this->mavrosState_.armed && (ros::Time::now() - lastRequest > ros::Duration(5.0)))
                {
                    if (this->armClient_.call(armCmd) && armCmd.response.success)
                    {
                        cout << "[AutoFlight]: Vehicle armed." << endl;
                    }
                    lastRequest = ros::Time::now();
                }
            }

            if (this->poseControl_)
            {
                // this->poseTgt_.header.stamp = ros::Time::now();
                this->posePub_.publish(this->poseTgt_);
            }
            else
            {
                this->statePub_.publish(this->stateTgt_);
            }
            // ros::spinOnce();
            r.sleep();
        }
    }

    void flightBase::stateCB(const mavros_msgs::State::ConstPtr &state)
    {
        this->mavrosState_ = *state;
        if (not this->mavrosStateReceived_)
        {
            this->mavrosStateReceived_ = true; // 状态接收标志位设为True
        }
    }

    void flightBase::currPoseCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
    {
        this->currPos_ = *msg;
        if (not this->currPoseReceived_)
        {
            this->currPoseReceived_ = true; // 位置接收标志位设为True
        }
    }

    void flightBase::takeoff()
    {
        // from cfg yaml read the flight height
        geometry_msgs::PoseStamped ps;
        ps.header.frame_id = "map";
        ps.header.stamp = ros::Time::now();
        ps.pose.position.x = this->currPose_.pose.position.x;
        ps.pose.position.y = this->currPose_.pose.position.y;
        ps.pose.position.z = this->takeoffHgt_;
        ps.pose.orientation = this->currPose_.pose.orientation;
        this->updateTarget(ps);

        cout << "[AutoFlight]: Start taking off..." << endl;
        ros::Rate r(30);

        // 等待起飞到目标高度
        while (ros::ok() and std::abs(this->currPose_.pose.position.z - this->takeoffHgt_) >= 0.1)
        {
            ros::spinOnce();
            r.sleep();
        }

        // 到达起飞目标高度后，延时3秒
        ros::Time startTime = ros::Time::now();
        while (ros::ok())
        {
            ros::Time currTime = ros::Time::now();
            if ((currTime - startTime).toSec() >= 3)
            {
                break;
            }
            ros::spinOnce();
            r.sleep();
        }
        cout << "[AutoFlight]: Takeoff succeed!" << endl;
    }

    void flightBase::circle()
    {
        // circle tracking parameters
        if (not this->nh_.getParam("autonomous_flight/circle_radius", this->radius_))
        {
            this->radius_ = 2.0;
            cout << "[AutoFlight]: No radius param found. Use default: 2.0 m." << endl;
        }
        else
        {
            cout << "[AutoFlight]: Circle Radius: " << this->radius_ << "m." << endl;
        }

        if (not this->nh_.getParam("autonomous_flight/time_to_max_radius", this->timeStep_))
        {
            this->timeStep_ = 30;
            cout << "[AutoFlight]: No time to max radius param found. Use default: 30s." << endl;
        }
        else
        {
            cout << "[AutoFlight]: Time to Maximum Circle Radius: " << this->timeStep_ << "s." << endl;
        }

        if (not this->nh_.getParam("autonomous_flight/yaw_control", this->yawControl_))
        {
            this->yawControl_ = false;
            cout << "[AutoFlight]: No yaw control param found. Use default: false." << endl;
        }
        else
        {
            cout << "[AutoFlight]: Yaw Control: " << this->yawControl_ << endl;
        }

        if (not this->nh_.getParam("autonomous_flight/velocity", this->velocity_))
        {
            this->velocity_ = 0.5;
            cout << "[AutoFlight]: No angular velocity param found. Use default: 0.5 m/s." << endl;
        }
        else
        {
            cout << "[AutoFlight]: Velocity: " << this->velocity_ << "m/s." << endl;
        }

        double x = 0;
        double y = 0;
        double z = 0;
        double vx = 0;
        double vy = 0;
        double vz = 0;
        double ax = 0;
        double ay = 0;
        double az = 0;
        double yaw = 0;
        double theta = 0;
        double radius = 0;
        double velocity = 0;
        int rate = 100;
        ros::Rate r(rate);
        double theta_start;
        double theta_end;
        double step = this->timeStep_ * rate;
        int circle = 1;
        int terminate = 0;
        ros::Time startTime = ros::Time::now();
        ros::Time endTime1, endTime2, endTime3;
        while (ros::ok() && terminate == 0)
        {
            x = radius * cos(theta);
            y = radius * sin(theta);
            vx = -velocity * sin(theta);
            vy = velocity * cos(theta);
            ax = -velocity * velocity / radius * cos(theta);
            ay = -velocity * velocity / radius * sin(theta);
            if (this->yawControl_ == true)
            {
                yaw = theta + PI_const / 2;
            }
            else if (this->yawControl_ == false)
            {
                yaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);
            }
            z = this->takeoffHgt_;
            vz = 0;
            az = 0;

            tracking_controller::Target target;
            target.position.x = x;
            target.position.y = y;
            target.position.z = z;
            target.velocity.x = vx;
            target.velocity.y = vy;
            target.velocity.z = vz;
            target.acceleration.x = ax;
            target.acceleration.y = ay;
            target.acceleration.z = az;
            target.yaw = yaw;

            this->updateTargetWithState(target);

            if (circle == 1)
            {
                radius += this->radius_ / step;
                velocity += this->velocity_ / step;

                if (std::abs(radius - radius_) <= 0.01)
                {
                    theta_start = theta;
                    endTime1 = ros::Time::now();
                    circle += 1;
                }
            }
            else if (circle == 2)
            {
                radius = this->radius_;
                velocity = this->velocity_;
                theta_end = 3 * 2 * PI_const;
                if (std::abs((theta - theta_start) - theta_end) <= 0.1)
                {
                    endTime2 = ros::Time::now();
                    circle += 1;
                }
            }
            else if (circle == 3)
            {
                radius -= this->radius_ / step;
                velocity -= this->velocity_ / step;
                if (std::abs(radius - 0.0) <= 0.01)
                {
                    endTime3 = ros::Time::now();
                    circle += 1;
                }
            }
            else
            {
                terminate = 1;
                break;
            }
            ros::Time currentTime = ros::Time::now();
            double t = (currentTime - startTime).toSec();
            theta = velocity / radius * t;
            cout << "[AutoFlight]: Drawing Circle...Radius: " << radius << "\t\r" << std::flush;
            ;
            ros::spinOnce();
            r.sleep();
        }

        if (this->yawControl_ == true)
        {
            while (ros::ok() and std::abs(this->odom_.pose.pose.orientation.z - 0.0) >= 0.01)
            {
                theta += (PI_const * 2) / 180;
                yaw = theta + PI_const / 2;

                tracking_controller::Target target;
                target.position.x = x;
                target.position.y = y;
                target.position.z = z;
                target.velocity.x = vx;
                target.velocity.y = vy;
                target.velocity.z = vz;
                target.acceleration.x = ax;
                target.acceleration.y = ay;
                target.acceleration.z = az;
                target.yaw = yaw;
                ROS_INFO("Decreasing Radius...");

                updateTargetWithState(target);
                ros::spinOnce();
                r.sleep();
            }
        }
    }

    void flightBase::run()
    {

        // flight test with circle
        double r; // radius
        double v; // circle velocity

        // track circle radius parameters
        if (not this->nh_.getParam("autonomous_flight/radius", r))
        {
            r = 2.0;
            cout << "[AutoFlight]: No circle radius param found. Use default: 2.0 m." << endl;
        }
        else
        {
            cout << "[AutoFlight]: Circle radius: " << r << "m." << endl;
        }

        // track circle velocity parameters
        if (not this->nh_.getParam("autonomous_flight/circle_velocity", v))
        {
            v = 1.0;
            cout << "[AutoFlight]: No circle velocity param found. Use default: 1.0 m/s." << endl;
        }
        else
        {
            cout << "[AutoFlight]: Circle velocity: " << v << "m/s." << endl;
        }

        double z = this->odom_.pose.pose.position.z;
        geometry_msgs::PoseStamped startPs;
        startPs.pose.position.x = r;
        startPs.pose.position.y = 0.0;
        startPs.pose.position.z = z;
        this->updateTarget(startPs);

        cout << "[AutoFlight]: Go to target point..." << endl;
        ros::Rate rate(30);
        while (ros::ok() and std::abs(this->odom_.pose.pose.position.x - startPs.pose.position.x) >= 0.1)
        {
            ros::spinOnce();
            rate.sleep();
        }
        cout << "[AutoFlight]: Reach target point." << endl;

        ros::Time startTime = ros::Time::now();
        while (ros::ok())
        {
            ros::Time currTime = ros::Time::now();
            double t = (currTime - startTime).toSec();
            double rad = v * t / r;
            double x = r * cos(rad);
            double y = r * sin(rad);
            double vx = -v * sin(rad);
            double vy = v * cos(rad);
            double vz = 0.0;
            double aNorm = v * v / r;
            Eigen::Vector3d accVec(x, y, 0);
            accVec = -aNorm * accVec / accVec.norm();
            double ax = accVec(0);
            double ay = accVec(1);
            double az = 0.0;

            // state target message
            tracking_controller::Target target;
            target.position.x = x;
            target.position.y = y;
            target.position.z = z;
            target.velocity.x = vx;
            target.velocity.y = vy;
            target.velocity.z = vz;
            target.acceleration.x = ax;
            target.acceleration.y = ay;
            target.acceleration.z = az;
            this->updateTargetWithState(target);
            ros::spinOnce();
            rate.sleep();
        }
    }

    void flightBase::stop()
    {
        geometry_msgs::PoseStamped ps;
        ps.pose = this->odom_.pose.pose;
        this->updateTarget(ps);
    }

    /*
     *功能：控制飞行器朝特定方向旋转
     *参数：yaw（偏航角）
     *     desiredAngularVel（期望的角速度）
     */
    void flightBase::moveToOrientation(double yaw, double desiredAngularVel)
    {
        double yawTgt = yaw;
        geometry_msgs::Quaternion orientation = AutoFlight::quaternion_from_rpy(0, 0, yaw);//将期望旋转的角度转为四元数
        double yawCurr = AutoFlight::rpy_from_quaternion(this->currPose_.pose.orientation);//获取当前偏航角并转为rpy
        geometry_msgs::PoseStamped ps;
        ps.pose = this->currPose_.pose;
        ps.pose.orientation = orientation; //获得期望位姿

        double yawDiff = yawTgt - yawCurr; // difference between yaw
        double direction = 0;
        double yawDiffAbs = std::abs(yawDiff);
        if ((yawDiffAbs <= PI_const) and (yawDiff > 0))
        {
            direction = 1.0; // counter clockwise 逆时针
        }
        else if ((yawDiffAbs <= PI_const) and (yawDiff < 0))
        {
            direction = -1.0; // clockwise  顺时针
        }
        else if ((yawDiffAbs > PI_const) and (yawDiff > 0))
        {
            direction = -1.0; // rotate in clockwise direction 顺时针方向旋转
            yawDiffAbs = 2 * PI_const - yawDiffAbs;
        }
        else if ((yawDiffAbs > PI_const) and (yawDiff < 0))
        {
            direction = 1.0; // counter clockwise 逆时针
            yawDiffAbs = 2 * PI_const - yawDiffAbs;
        }

        double endTime = yawDiffAbs / desiredAngularVel; //需要转过的角度/角速度 = 时间
        tracking_controller::Target target;
        geometry_msgs::PoseStamped psT;
        psT.pose = ps.pose;
        ros::Time startTime = ros::Time::now();
        ros::Time currTime = ros::Time::now();
        ros::Rate r(200);
        while (ros::ok() and not this->isReach(ps))
        {
            currTime = ros::Time::now();
            double t = (currTime - startTime).toSec();

            if (t >= endTime)
            {
                psT = ps;
            }
            else
            {
                double currYawTgt = yawCurr + (double)direction * t / endTime * yawDiffAbs;
                geometry_msgs::Quaternion quatT = AutoFlight::quaternion_from_rpy(0, 0, currYawTgt);
                psT.pose.orientation = quatT;
            }
            // this->updateTarget(psT);
            target.position.x = psT.pose.position.x;
            target.position.y = psT.pose.position.y;
            target.position.z = psT.pose.position.z;
            target.yaw = AutoFlight::rpy_from_quaternion(psT.pose.orientation);
            this->updateTargetWithState(target);
            // cout << "here" << endl;
            ros::spinOnce();
            r.sleep();
        }
    }

    void flightBase::updateTarget(const geometry_msgs::PoseStamped &ps)
    {
        this->poseTgt_ = ps;
        this->poseTgt_.header.frame_id = "map";
        this->poseControl_ = true;
    }

    void flightBase::updateTargetWithState(const tracking_controller::Target &target)
    {
        this->stateTgt_ = target;
        this->poseControl_ = false;
    }

    bool flightBase::isReach(const geometry_msgs::PoseStamped &poseTgt, bool useYaw)
    {
        double targetX, targetY, targetZ, targetYaw, currX, currY, currZ, currYaw;
        targetX = poseTgt.pose.position.x;
        targetY = poseTgt.pose.position.y;
        targetZ = poseTgt.pose.position.z;
        targetYaw = AutoFlight::rpy_from_quaternion(poseTgt.pose.orientation);
        currX = this->odom_.pose.pose.position.x;
        currY = this->odom_.pose.pose.position.y;
        currZ = this->odom_.pose.pose.position.z;
        currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);

        bool reachX, reachY, reachZ, reachYaw;
        reachX = std::abs(targetX - currX) < 0.1;
        reachY = std::abs(targetY - currY) < 0.1;
        reachZ = std::abs(targetZ - currZ) < 0.15;
        if (useYaw)
        {
            reachYaw = std::abs(targetYaw - currYaw) < 0.1;
        }
        else
        {
            reachYaw = true;
        }

        if (reachX and reachY and reachZ and reachYaw)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool flightBase::isReach(const geometry_msgs::PoseStamped &poseTgt, double dist, bool useYaw)
    {
        double targetX, targetY, targetZ, targetYaw, currX, currY, currZ, currYaw;
        targetX = poseTgt.pose.position.x;
        targetY = poseTgt.pose.position.y;
        targetZ = poseTgt.pose.position.z;
        targetYaw = AutoFlight::rpy_from_quaternion(poseTgt.pose.orientation);
        currX = this->odom_.pose.pose.position.x;
        currY = this->odom_.pose.pose.position.y;
        currZ = this->odom_.pose.pose.position.z;
        currYaw = AutoFlight::rpy_from_quaternion(this->odom_.pose.pose.orientation);

        bool reachX, reachY, reachZ, reachYaw;
        reachX = std::abs(targetX - currX) < dist;
        reachY = std::abs(targetY - currY) < dist;
        reachZ = std::abs(targetZ - currZ) < dist;
        if (useYaw)
        {
            reachYaw = std::abs(targetYaw - currYaw) < 0.1;
        }
        else
        {
            reachYaw = true;
        }

        // cout << "x: " << std::abs(targetX - currX) << " y: " << std::abs(targetY - currY) << " z: " << std::abs(targetZ - currZ) << endl;
        if (reachX and reachY and reachZ and reachYaw)
        {
            return true;
        }
        else
        {
            return false;
        }
    }
}