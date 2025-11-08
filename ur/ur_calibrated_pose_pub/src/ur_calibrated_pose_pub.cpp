#include <ur_calibrated_pose_pub/ur_calibrated_pose_pub.h>
#include <mutex>
#include <array>
#include <algorithm>

namespace {

// Thread-sicherer Puffer für den neuesten Gelenkwinkelzustand
std::array<double, 6> g_q_latest{{0,0,0,0,0,0}};
std::mutex g_q_mtx;
std::array<int,6> g_idx{{-1,-1,-1,-1,-1,-1}};
bool g_idx_ready = false;
ros::Time g_last_stamp;

// Denavit–Hartenberg Einzelschritt
inline Eigen::Matrix4d dh_T(double theta, double d, double a, double alpha)
{
  const double ct = std::cos(theta);
  const double st = std::sin(theta);
  const double ca = std::cos(alpha);
  const double sa = std::sin(alpha);

  Eigen::Matrix4d T;
  // UR: TransZ(d) * RotZ(theta) * TransX(a) * RotX(alpha)
  T <<  ct,   -st*ca,   st*sa,  a*ct,
        st,    ct*ca,  -ct*sa,  a*st,
         0,        sa,      ca,     d,
         0,         0,       0,     1;
  return T;
}

} // anonymous ns

namespace ur_calibrated_pose_pub
{
  URCalibratedPosePub::URCalibratedPosePub(ros::NodeHandle nh,
                                           ros::NodeHandle private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  {}

  void URCalibratedPosePub::readParams()
  {
    private_nh_.param<std::string>("ur_joint_state_topic_name", ur_joint_state_topic_name_, std::string("/joint_states"));
    private_nh_.param<std::string>("joint_prefix", joint_prefix_, std::string(""));
    // "calibrated" oder "ideal"
    private_nh_.param<std::string>("dh_parameter_switch", dh_parameter_switch_, std::string("calibrated"));
  }

  void URCalibratedPosePub::init()
  {
    readParams();
    //getCalibratedDHParameter(); // bleibt unverändert; liefert calibrated_dh_transformations_list_

    joint_state_subscriber_ = nh_.subscribe(ur_joint_state_topic_name_, 1,
      &URCalibratedPosePub::jointStateCallback, this,
      ros::TransportHints().tcpNoDelay());

    ur_calibrated_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("ur_calibrated_pose", 1);
  }

  // Pose aus (ideal|calibrated) Basis-DH + aktuellem q berechnen
	bool URCalibratedPosePub::computePose(Eigen::Isometry3d& out_T_base_tcp)
	{
		std::array<double,6> q;
		{ std::lock_guard<std::mutex> lk(g_q_mtx); q = g_q_latest; }

		// Liste wählen: erst calibrated, sonst ideal
		const auto& pref = (dh_parameter_switch_ == "ideal")
							? ideal_dh_transformations_list_
							: calibrated_dh_transformations_list_;
		const auto& alt  = (dh_parameter_switch_ == "ideal")
							? calibrated_dh_transformations_list_
							: ideal_dh_transformations_list_;

		const std::vector<dh_utils::DHTransformation>* base_ptr = nullptr;
		if (pref.size() >= 6) base_ptr = &pref;
		else if (alt.size() >= 6) base_ptr = &alt;
		else return false;  // noch keine DH-Daten verfügbar

		const auto& base = *base_ptr;

		Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
		for (int i = 0; i < 6; ++i) {
			const double theta0 = base[i].getTheta();
			const double d      = base[i].getd();
			const double a      = base[i].geta();
			const double alpha  = base[i].getAlpha();
			T = T * dh_T(theta0 + q[i], d, a, alpha);
		}
		out_T_base_tcp = Eigen::Isometry3d(T);
		return true;
	}

  void URCalibratedPosePub::execute()
  {
    const double publish_rate_hz = private_nh_.param("publish_rate", 250.0);
    ros::Rate r(publish_rate_hz);

    while (ros::ok())
    {
      ros::spinOnce();

      Eigen::Isometry3d T;
      if (computePose(T))
      {
        const Eigen::Vector3d p = T.translation();
        const Eigen::Quaterniond q(T.rotation());

        geometry_msgs::PoseStamped msg;
        msg.header.frame_id = "base_link"; // bei Bedarf anpassen
        msg.header.stamp = g_last_stamp.isZero() ? ros::Time::now() : g_last_stamp;

        msg.pose.position.x = p.x();
        msg.pose.position.y = p.y();
        msg.pose.position.z = p.z();
        msg.pose.orientation.x = q.x();
        msg.pose.orientation.y = q.y();
        msg.pose.orientation.z = q.z();
        msg.pose.orientation.w = q.w();

        ur_calibrated_pose_publisher_.publish(msg);
      }

      r.sleep();
    }
  }

  void URCalibratedPosePub::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    // Joint-Index-Mapping einmalig aufbauen
    if (!g_idx_ready)
    {
      const std::array<std::string,6> want{
        joint_prefix_ + "shoulder_pan_joint",
        joint_prefix_ + "shoulder_lift_joint",
        joint_prefix_ + "elbow_joint",
        joint_prefix_ + "wrist_1_joint",
        joint_prefix_ + "wrist_2_joint",
        joint_prefix_ + "wrist_3_joint"
      };

      bool all_found = true;
      for (int i = 0; i < 6; ++i)
      {
        auto it = std::find(msg->name.begin(), msg->name.end(), want[i]);
        if (it == msg->name.end())
        {
          all_found = false;
          break;
        }
        g_idx[i] = static_cast<int>(std::distance(msg->name.begin(), it));
      }
      if (!all_found) {
        return; // warten bis alle Joints vorhanden sind
      }
      g_idx_ready = true;
    }

    std::array<double,6> q;
    for (int i = 0; i < 6; ++i) {
      const int k = g_idx[i];
      if (k < 0 || static_cast<size_t>(k) >= msg->position.size()) {
        return; // ungültige Nachricht
      }
      q[i] = msg->position[k];
    }

    {
      std::lock_guard<std::mutex> lk(g_q_mtx);
      g_q_latest = q;
      g_last_stamp = msg->header.stamp;
    }
  }

} // namespace ur_calibrated_pose_pub
