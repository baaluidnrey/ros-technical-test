#ifndef PANEL_TP_FANUC_H
#define PANEL_TP_FANUC_H


#include <ros/ros.h>
#include <rviz/panel.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/String.h>
#include <tf/tf.h>
#include <tf2_msgs/TFMessage.h>
#include <QtWidgets>
#include <math.h>


// Modes de controle
#define MODE_ARTI     0
#define MODE_CART     1
#define MODE_VITESSE  2

// Representation de la rotation
#define QUATERNION    0
#define MATRICE_ROT   1
#define RPY           2


namespace rviz_plugins_robexp
{


class Panel_TP_Fanuc: public rviz::Panel
{

  Q_OBJECT

  public:

    Panel_TP_Fanuc( QWidget* parent = 0 );
    ~Panel_TP_Fanuc();

    QHBoxLayout* create_horizontal_layout(QWidget* box, QString label);

    void config_arti(QDoubleSpinBox* box);
    void config_pos(QDoubleSpinBox* box);
    void config_quat(QDoubleSpinBox* box, double valeur);
    void config_mat_rot(QDoubleSpinBox* box, double valeur);
    void config_rpy(QDoubleSpinBox* box, double valeur);
    void config_vit_lin(QDoubleSpinBox* box);
    void config_vit_ang(QDoubleSpinBox* box);
    geometry_msgs::Pose compute_pose() const;
    sensor_msgs::JointState compute_joints() const;
    void jointsCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void framesCallback(const tf2_msgs::TFMessage::ConstPtr& msg);
    void setQuaternion(tf::Quaternion quat);
    void setMatrix(tf::Matrix3x3 matrix);
    void setRPY(double roll, double pitch, double yaw);



  public Q_SLOTS:
    void changer_representation_rotation(QString selection);
    void go_home();
    void clear_path() const;
    void stop_mvt();
    void change_mvt_state(bool etat);
    void move();
    void send_joints() const;
    void move_arti();
    void send_pose();
    void move_cart();
    void send_cmd_vel() const;
    void move_vel();
    void change_config();
    void send_config();
    void refresh_arti();
    void refresh_cart();
    void refresh_orientation();
    void display_cmd_arti();
    void display_cmd_op();
    void display_cmd_vel();



  private:

    // articulaire
    QDoubleSpinBox *_q1;
    QDoubleSpinBox *_q2;
    QDoubleSpinBox *_q3;
    QDoubleSpinBox *_q4;
    QDoubleSpinBox *_q5;
    QDoubleSpinBox *_q6;

    // position
    QDoubleSpinBox *_px;
    QDoubleSpinBox *_py;
    QDoubleSpinBox *_pz;

    // quaternion
    QDoubleSpinBox *_qx;
    QDoubleSpinBox *_qy;
    QDoubleSpinBox *_qz;
    QDoubleSpinBox *_qw;

    // matrice de rotation
    QDoubleSpinBox *_rot11;
    QDoubleSpinBox *_rot12;
    QDoubleSpinBox *_rot13;
    QDoubleSpinBox *_rot21;
    QDoubleSpinBox *_rot22;
    QDoubleSpinBox *_rot23;
    QDoubleSpinBox *_rot31;
    QDoubleSpinBox *_rot32;
    QDoubleSpinBox *_rot33;

    // roll - pitch - yaw
    QDoubleSpinBox *_roll;
    QDoubleSpinBox *_pitch;
    QDoubleSpinBox *_yaw;

    // vitesse lineaire
    QDoubleSpinBox *_vx;
    QDoubleSpinBox *_vy;
    QDoubleSpinBox *_vz;

    // vitesse angulaire
    QDoubleSpinBox *_wx;
    QDoubleSpinBox *_wy;
    QDoubleSpinBox *_wz;

    // configuration articulaire
    QComboBox *_config_torso;
    QComboBox *_config_shoulder;
    QComboBox *_config_wrist;

    // selection de la representation de l'orientation
    QWidget *_widget_quat;
    QWidget *_widget_rot_mat;
    QWidget *_widget_rpy;

    // Panneaux de commandes
    QGroupBox *_box_arti;
    QGroupBox *_box_op;
    QGroupBox *_box_vel;
    QWidget *_widget_arti;
    QWidget *_widget_op;
    QWidget *_widget_vel;

    // ROS
    ros::NodeHandle _nh;
    ros::Publisher _joints_publisher;
    ros::Publisher _manage_path_publisher;
    ros::Publisher _cmd_vel_publisher;
    ros::Publisher _pose_publisher;
    ros::Publisher _config_publisher;
    ros::Subscriber _joints_subscriber;
    ros::Subscriber _frames_subscriber;

    // mode de controle
    int _mode;
    int _representation_rotation;
    bool _mvt;
    std::string _config;
    bool _chgt_config;
    bool _blocage;
    tf::Vector3 _position;
    tf::Quaternion _orientation;
};

}


#endif // PANEL_TP_FANUC_H
