#include "panel_tp_fanuc.h"



namespace rviz_plugins_robexp
{

Panel_TP_Fanuc::Panel_TP_Fanuc( QWidget* parent ) :
  rviz::Panel( parent )
  {


    /*
      INITIALISATION DES ATTRIBUTS
    */

    _q1 = new QDoubleSpinBox;
    _q2 = new QDoubleSpinBox;
    _q3 = new QDoubleSpinBox;
    _q4 = new QDoubleSpinBox;
    _q5 = new QDoubleSpinBox;
    _q6 = new QDoubleSpinBox;

    _px = new QDoubleSpinBox;
    _py = new QDoubleSpinBox;
    _pz = new QDoubleSpinBox;

    _qx = new QDoubleSpinBox;
    _qy = new QDoubleSpinBox;
    _qz = new QDoubleSpinBox;
    _qw = new QDoubleSpinBox;

    _rot11 = new QDoubleSpinBox;
    _rot12 = new QDoubleSpinBox;
    _rot13 = new QDoubleSpinBox;
    _rot21 = new QDoubleSpinBox;
    _rot22 = new QDoubleSpinBox;
    _rot23 = new QDoubleSpinBox;
    _rot31 = new QDoubleSpinBox;
    _rot32 = new QDoubleSpinBox;
    _rot33 = new QDoubleSpinBox;

    _roll = new QDoubleSpinBox;
    _pitch = new QDoubleSpinBox;
    _yaw = new QDoubleSpinBox;

    _vx = new QDoubleSpinBox;
    _vy = new QDoubleSpinBox;
    _vz = new QDoubleSpinBox;

    _wx = new QDoubleSpinBox;
    _wy = new QDoubleSpinBox;
    _wz = new QDoubleSpinBox;

    _config_torso = new QComboBox;
    _config_shoulder = new QComboBox;
    _config_wrist = new QComboBox;

    _widget_quat = new QWidget;
    _widget_rot_mat = new QWidget;
    _widget_rpy = new QWidget;

    _box_arti = new QGroupBox("Consigne articulaire");
    _box_op = new QGroupBox("Consigne opérationnelle");
    _box_vel = new QGroupBox("Consigne de vitesse");
    _widget_arti = new QWidget;
    _widget_op = new QWidget;
    _widget_vel = new QWidget;

    _joints_publisher = _nh.advertise<sensor_msgs::JointState>("/joints_state", 1);
    _cmd_vel_publisher = _nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    _pose_publisher = _nh.advertise<geometry_msgs::Pose>("/pose", 1);
    _manage_path_publisher = _nh.advertise<std_msgs::String>("/manage_path", 1);
    _config_publisher = _nh.advertise<std_msgs::String>("/config", 1);

    _joints_subscriber = _nh.subscribe("/joints_state", 1, &Panel_TP_Fanuc::jointsCallback, this); // https://answers.ros.org/question/108551/using-subscribercallback-function-inside-of-a-class-c/
    _frames_subscriber = _nh.subscribe("/tf", 1, &Panel_TP_Fanuc::framesCallback, this);

    _mode = MODE_ARTI;
    _mvt = false;
    _representation_rotation = MATRICE_ROT;
    _config = "NUT";
    _chgt_config = false;
    _blocage = false;

    tf::Vector3 _position = tf::Vector3(0.0, 0.0, 0.0); // position courante
    tf::Quaternion _orientation = tf::Quaternion(0.0, 0.0, 0.0, 1.0); // orientation courante



    /*
      CONSIGNE ANGULAIRE
    */

      // ARTICULATION 1
      QHBoxLayout *arti1 = create_horizontal_layout(_q1, "q1 :");
      config_arti(_q1);

      // ARTICULATION 2
      QHBoxLayout *arti2 = create_horizontal_layout(_q2, "q2 :");
      config_arti(_q2);

      // ARTICULATION 3
      QHBoxLayout *arti3 = create_horizontal_layout(_q3, "q3 :");
      config_arti(_q3);

      // ARTICULATION 4
      QHBoxLayout *arti4 = create_horizontal_layout(_q4, "q4 :");
      config_arti(_q4);

      // ARTICULATION 5
      QHBoxLayout *arti5 = create_horizontal_layout(_q5, "q5 :");
      config_arti(_q5);

      // ARTICULATION 6
      QHBoxLayout *arti6 = create_horizontal_layout(_q6, "q6 :");
      config_arti(_q6);

      // LAYOUT COMMANDE ARTICULAIRE

      // _widget_arti
      QGridLayout *layout_widget_arti = new QGridLayout;
      layout_widget_arti->addLayout(arti1,0,0);
      layout_widget_arti->addLayout(arti2,0,1);
      layout_widget_arti->addLayout(arti3,0,2);
      layout_widget_arti->addLayout(arti4,1,0);
      layout_widget_arti->addLayout(arti5,1,1);
      layout_widget_arti->addLayout(arti6,1,2);
      _widget_arti->setLayout(layout_widget_arti);

      // _box_arti
      QVBoxLayout *layout_arti = new QVBoxLayout;
      layout_arti->addWidget(_widget_arti);
      _box_arti->setLayout(layout_arti);
      _box_arti->setCheckable(true);
      //_box_arti->setStyleSheet("QGroupBox { font-weight: bold; } ");



    /*
      CONSIGNE OPERATIONNELLE
    */

      // CONFIGURATION ARTICULAIRE TORSE
      QHBoxLayout *torso = create_horizontal_layout(_config_torso, "Torse :");
      _config_torso->addItem("Toward (T)");
      _config_torso->addItem("Backward (B)");

      // CONFIGURATION ARTICULAIRE EPAULE
      QHBoxLayout *shoulder = create_horizontal_layout(_config_shoulder, "Epaule :");
      _config_shoulder->addItem("Up (U)");
      _config_shoulder->addItem("Down (D)");

      // CONFIGURATION ARTICULAIRE POIGNET
      QHBoxLayout *wrist = create_horizontal_layout(_config_wrist, "Poignet :");
      _config_wrist->addItem("No-Flip (N)");
      _config_wrist->addItem("Flip (F)");

      // LAYOUT CONFIGURATION ARTICULAIRE
      QVBoxLayout *layout_config_q = new QVBoxLayout;
      layout_config_q->addLayout(torso);
      layout_config_q->addLayout(shoulder);
      layout_config_q->addLayout(wrist);

      QGroupBox *box_config_q = new QGroupBox("Configuration articulaire");
      box_config_q->setLayout(layout_config_q);



      // POSITION X
      QHBoxLayout *pos_x = create_horizontal_layout(_px, "x :");
      config_pos(_px);

      // POSITION Y
      QHBoxLayout *pos_y = create_horizontal_layout(_py, "y :");
      config_pos(_py);

      // POSITION Z
      QHBoxLayout *pos_z = create_horizontal_layout(_pz, "z :");
      config_pos(_pz);

      // LAYOUT POSITION
      QVBoxLayout *layout_pos = new QVBoxLayout;
      layout_pos->addLayout(pos_x);
      layout_pos->addLayout(pos_y);
      layout_pos->addLayout(pos_z);

      QGroupBox *box_pos = new QGroupBox("Position");
      box_pos->setLayout(layout_pos);



      // CONFIGURATION DE LA ROTATION
      QComboBox *config_rot_value = new QComboBox;
      config_rot_value->addItem("Matrice");
      config_rot_value->addItem("Roll Pitch Yaw");
      config_rot_value->addItem("Quaternion");

      QHBoxLayout *config_rot_layout = create_horizontal_layout(config_rot_value, "Convention :");




      // QUATERNION

        // QUATERNION X
        QHBoxLayout *quat_x = create_horizontal_layout(_qx, "x :");
        config_quat(_qx, 1.0);

        // QUATERNION Y
        QHBoxLayout *quat_y = create_horizontal_layout(_qy, "y :");
        config_quat(_qy, 0.0);

        // QUATERNION Z
        QHBoxLayout *quat_z = create_horizontal_layout(_qz, "z :");
        config_quat(_qz, 0.0);

        // QUATERNION W
        QHBoxLayout *quat_w = create_horizontal_layout(_qw, "w :");
        config_quat(_qw, 0.0);

        // LAYOUT
        QVBoxLayout *layout_quat = new QVBoxLayout;
        layout_quat->addLayout(quat_x);
        layout_quat->addLayout(quat_y);
        layout_quat->addLayout(quat_z);
        layout_quat->addLayout(quat_w);

        // WIDGET
        _widget_quat->setLayout(layout_quat);
        _widget_quat->setVisible(false);        // matrice au lancement


      // MATRICE DE ROTATION

        // R..
        config_mat_rot(_rot11, 1.0);
        config_mat_rot(_rot12, 0.0);
        config_mat_rot(_rot13, 0.0);
        config_mat_rot(_rot21, 0.0);
        config_mat_rot(_rot22, -1.0);
        config_mat_rot(_rot23, 0.0);
        config_mat_rot(_rot31, 0.0);
        config_mat_rot(_rot32, 0.0);
        config_mat_rot(_rot33, -1.0);

        // LAYOUT
        QGridLayout *layout_rot_mat = new QGridLayout;
        layout_rot_mat->addWidget(_rot11,0,0);
        layout_rot_mat->addWidget(_rot12,0,1);
        layout_rot_mat->addWidget(_rot13,0,2);
        layout_rot_mat->addWidget(_rot21,1,0);
        layout_rot_mat->addWidget(_rot22,1,1);
        layout_rot_mat->addWidget(_rot23,1,2);
        layout_rot_mat->addWidget(_rot31,2,0);
        layout_rot_mat->addWidget(_rot32,2,1);
        layout_rot_mat->addWidget(_rot33,2,2);

        // WIDGET
        _widget_rot_mat->setLayout(layout_rot_mat);
        _widget_rot_mat->setVisible(true);          // matrice au lancement


      // ROLL - PITCH - YAW

        // ROT X
        QHBoxLayout *rpy_r = create_horizontal_layout(_roll, "\u03B3 :"); // \u03B3 = gamma
        config_rpy(_roll,180.0);

        // ROT Y
        QHBoxLayout *rpy_p = create_horizontal_layout(_pitch, "\u03B2 :"); // \u03B2 = beta
        config_rpy(_pitch,0.0);

        // ROT Z
        QHBoxLayout *rpy_y = create_horizontal_layout(_yaw, "\u03B1 :");  // \u03B1 = alpha
        config_rpy(_yaw,0.0);

        // LAYOUT
        QVBoxLayout *layout_rpy = new QVBoxLayout;
        layout_rpy->addLayout(rpy_r);
        layout_rpy->addLayout(rpy_p);
        layout_rpy->addLayout(rpy_y);

        // WIDGET
        _widget_rpy->setLayout(layout_rpy);
        _widget_rpy->setVisible(false);          // matrice au lancement



      // LAYOUT ORIENTATION
      QVBoxLayout *box_or_layout = new QVBoxLayout;
      box_or_layout->addLayout(config_rot_layout);
      box_or_layout->addWidget(_widget_quat);
      box_or_layout->addWidget(_widget_rot_mat);
      box_or_layout->addWidget(_widget_rpy);

      QGroupBox *box_or = new QGroupBox("Orientation");
      box_or->setLayout(box_or_layout);



      // LAYOUT COMMANDE OPERATIONNELLE

      // position et orientation
      QHBoxLayout *layout_op_cmd = new QHBoxLayout;
      layout_op_cmd->addWidget(box_pos);
      layout_op_cmd->addWidget(box_or);

      /*
      QVBoxLayout *layout_op = new QVBoxLayout;
      layout_op->addWidget(box_config_q);
      layout_op->addLayout(layout_op_cmd);

      _box_op->setLayout(layout_op);
      */

      // _widget_op
      QVBoxLayout *layout_widget_op = new QVBoxLayout;
      layout_widget_op->addWidget(box_config_q);
      layout_widget_op->addLayout(layout_op_cmd);
      _widget_op->setLayout(layout_widget_op);

      // _box_op
      QVBoxLayout *layout_op = new QVBoxLayout;
      layout_op->addWidget(_widget_op);
      _box_op->setLayout(layout_op);
      _box_op->setCheckable(true);
      //_box_op->setStyleSheet("QGroupBox { font-weight: bold; } ");




    /*
      CONSIGNE DE VITESSE
    */

      // VITESSE LINEAIRE /X
      QHBoxLayout *vel_lin_x = create_horizontal_layout(_vx, "x :");
      config_vit_lin(_vx);

      // VITESSE LINEAIRE /Y
      QHBoxLayout *vel_lin_y = create_horizontal_layout(_vy, "y :");
      config_vit_lin(_vy);

      // VITESSE LINEAIRE /Z
      QHBoxLayout *vel_lin_z = create_horizontal_layout(_vz, "z :");
      config_vit_lin(_vz);

      // VITESSE ANGULAIRE /X
      QHBoxLayout *vel_ang_x = create_horizontal_layout(_wx, "x :");
      config_vit_ang(_wx);

      // VITESSE ANGULAIRE /Y
      QHBoxLayout *vel_ang_y = create_horizontal_layout(_wy, "y :");
      config_vit_ang(_wy);

      // VITESSE ANGULAIRE /Z
      QHBoxLayout *vel_ang_z = create_horizontal_layout(_wz, "z :");
      config_vit_ang(_wz);


      // LAYOUT VITESSE LINEAIRE
      QVBoxLayout *layout_vel_lin = new QVBoxLayout;
      layout_vel_lin->addLayout(vel_lin_x);
      layout_vel_lin->addLayout(vel_lin_y);
      layout_vel_lin->addLayout(vel_lin_z);

      QGroupBox *box_vel_lin = new QGroupBox("Vitesse linéaire");
      box_vel_lin->setLayout(layout_vel_lin);


      // LAYOUT VITESSE ANGULAIRE
      QVBoxLayout *layout_vel_ang = new QVBoxLayout;
      layout_vel_ang->addLayout(vel_ang_x);
      layout_vel_ang->addLayout(vel_ang_y);
      layout_vel_ang->addLayout(vel_ang_z);

      QGroupBox *box_vel_ang = new QGroupBox("Vitesse angulaire");
      box_vel_ang->setLayout(layout_vel_ang);


      // LAYOUT COMMANDE DE VITESSE
      // _widget_vel
      QHBoxLayout *layout_widget_vel = new QHBoxLayout;
      layout_widget_vel->addWidget(box_vel_lin);
      layout_widget_vel->addWidget(box_vel_ang);
      _widget_vel->setLayout(layout_widget_vel);

      // _box_vel
      QVBoxLayout *layout_vel = new QVBoxLayout;
      layout_vel->addWidget(_widget_vel);
      _box_vel->setLayout(layout_vel);
      _box_vel->setCheckable(true);
      //_box_vel->setStyleSheet("QGroupBox { font-weight: bold; } ");



    /*
      BOUTONS DE COMMANDE
    */

      // BOUTONS
      QPushButton *button_homing = new QPushButton("Position initiale");
      QPushButton *button_mvt_continu = new QPushButton("Mouvement continu");
      QPushButton *button_mvt_unique = new QPushButton("Aller");
      QPushButton *button_stop = new QPushButton("Stop");
      QPushButton *button_clear_path = new QPushButton("Effacer trajectoire");

      button_mvt_continu->setCheckable(true);

      // LAYOUT BOUTONS
      QGridLayout *layout_buttons = new QGridLayout;
      layout_buttons->addWidget(button_mvt_continu,0,0);
      layout_buttons->addWidget(button_mvt_unique,0,1);
      layout_buttons->addWidget(button_stop,0,2);
      layout_buttons->addWidget(button_clear_path,1,0);
      layout_buttons->addWidget(button_homing,1,2);

      QGroupBox *box_buttons = new QGroupBox("Panneau de commande");
      box_buttons->setLayout(layout_buttons);
      //box_buttons->setStyleSheet("QGroupBox { font-weight: bold; } ");


    /*
      LAYOUT FENETRE
    */
      QVBoxLayout *layout = new QVBoxLayout;
      layout->addWidget(box_buttons);
      layout->addWidget(_box_arti);
      layout->addWidget(_box_op);
      layout->addWidget(_box_vel);
      this->setLayout(layout);



    /*
        CONNEXIONS
        Une partie des connexions est realisee dans les methodes de config des differentes QDoubleSpinBox
    */

      // REPRESENTATION DE L'ORIENTATION
      connect( config_rot_value, SIGNAL(currentTextChanged(QString)), this, SLOT(changer_representation_rotation(QString)) );

      // BOUTONS DE COMMANDE
      connect( button_homing, SIGNAL(clicked()), this, SLOT(go_home()) );
      connect( button_clear_path, SIGNAL(clicked()), this, SLOT(clear_path()) );
      connect( button_stop, SIGNAL(clicked()), this, SLOT(stop_mvt()) );
      connect( button_mvt_continu, SIGNAL(toggled(bool)), this, SLOT(change_mvt_state(bool)) );
      connect( button_mvt_unique, SIGNAL(clicked()), this, SLOT(move()) );

      // CONFIGURATION ARTICULAIRE
      connect( _config_torso, SIGNAL(currentTextChanged(QString)), this, SLOT(change_config()) );
      connect( _config_shoulder, SIGNAL(currentTextChanged(QString)), this, SLOT(change_config()) );
      connect( _config_wrist, SIGNAL(currentTextChanged(QString)), this, SLOT(change_config()) );

      // AFFICHAGE DES DIFFERENTS MODES DE CONTROLE
      connect( _box_arti, SIGNAL(toggled(bool)), this, SLOT(display_cmd_arti()));
      connect( _box_op, SIGNAL(toggled(bool)), this, SLOT(display_cmd_op()));
      connect( _box_vel, SIGNAL(toggled(bool)), this, SLOT(display_cmd_vel()));

  }



Panel_TP_Fanuc::~Panel_TP_Fanuc()
{
  delete _q1;
  delete _q2;
  delete _q3;
  delete _q4;
  delete _q5;
  delete _q6;

  delete _px;
  delete _py;
  delete _pz;

  delete _qx;
  delete _qy;
  delete _qz;
  delete _qw;

  delete _rot11;
  delete _rot12;
  delete _rot13;
  delete _rot21;
  delete _rot22;
  delete _rot23;
  delete _rot31;
  delete _rot32;
  delete _rot33;

  delete _roll;
  delete _pitch;
  delete _yaw;

  delete _vx;
  delete _vy;
  delete _vz;

  delete _wx;
  delete _wy;
  delete _wz;

  delete _config_torso;
  delete _config_shoulder;
  delete _config_wrist;

  delete _widget_quat;
  delete _widget_rot_mat;
  delete _widget_rpy;

  delete _box_arti;
  delete _box_op;
  delete _box_vel;
  delete _widget_arti;
  delete _widget_op;
  delete _widget_vel;
}



/*
  CONFIGURATION DES QDoubleSpinBox
*/

QHBoxLayout* Panel_TP_Fanuc::create_horizontal_layout(QWidget* box, QString label)
{
  QHBoxLayout *layout = new QHBoxLayout;
  QLabel *qlabel = new QLabel(label);
  //qlabel->setAlignment(Qt::AlignRight);
  layout->addWidget(qlabel);
  layout->addWidget(box);
  return layout;
}


void Panel_TP_Fanuc::config_arti(QDoubleSpinBox* box)
{
  // mise en forme
  box->setSuffix("°");
  box->setSingleStep(5);
  box->setRange(-180.0, 180.0);

  // connexions
  connect( box, SIGNAL(valueChanged(double)), this, SLOT(move_arti()) );
}


void Panel_TP_Fanuc::config_pos(QDoubleSpinBox* box)
{
  // mise en forme
  box->setSuffix("m");
  box->setSingleStep(0.05);
  box->setRange(-1.0, 1.0);

  // connexions
  connect( box, SIGNAL(valueChanged(double)), this, SLOT(move_cart()) );
}


void Panel_TP_Fanuc::config_quat(QDoubleSpinBox* box, double valeur)
{
  // mise en forme
  box->setValue(valeur);
  box->setSingleStep(0.05);
  box->setRange(-1.0, 1.0);

  // connexions
  connect( box, SIGNAL(valueChanged(double)), this, SLOT(refresh_orientation()) );
  connect( box, SIGNAL(valueChanged(double)), this, SLOT(move_cart()) );
}


void Panel_TP_Fanuc::config_mat_rot(QDoubleSpinBox* box, double valeur)
{
  // mise en forme
  box->setSingleStep(0.05);
  box->setRange(-1.0, 1.0);
  box->setValue(valeur);
  //box->setFrame(false);
  box->setButtonSymbols(QAbstractSpinBox::NoButtons);

  // connexions
  connect( box, SIGNAL(valueChanged(double)), this, SLOT(refresh_orientation()) );
  connect( box, SIGNAL(valueChanged(double)), this, SLOT(move_cart()) );
}


void Panel_TP_Fanuc::config_rpy(QDoubleSpinBox* box, double valeur)
{
  // mise en forme
  box->setSuffix("°");
  box->setSingleStep(5);
  box->setRange(-360.0, 360.0);
  box->setValue(valeur);

  // connexions
  connect( box, SIGNAL(valueChanged(double)), this, SLOT(refresh_orientation()) );
  connect( box, SIGNAL(valueChanged(double)), this, SLOT(move_cart()) );
}


void Panel_TP_Fanuc::config_vit_lin(QDoubleSpinBox* box)
{
  // mise en forme
  box->setSuffix("m/sec");
  box->setSingleStep(0.01);
  box->setRange(-1.0, 1.0);

  // connexions
  connect( box, SIGNAL(valueChanged(double)), this, SLOT(move_vel()) );
}


void Panel_TP_Fanuc::config_vit_ang(QDoubleSpinBox* box)
{
  // mise en forme
  box->setSuffix("°/sec");
  box->setSingleStep(1);
  box->setRange(-180.0, 180.0);

  // connexions
  connect( box, SIGNAL(valueChanged(double)), this, SLOT(move_vel()) );
}

// Creer un objet de type pose avec les consignes operationnelles
geometry_msgs::Pose Panel_TP_Fanuc::compute_pose() const
{
  geometry_msgs::Pose pose;
  pose.position.x = _px->value();
  pose.position.y = _py->value();
  pose.position.z = _pz->value();
  pose.orientation.x = _qx->value();
  pose.orientation.y = _qy->value();
  pose.orientation.z = _qz->value();
  pose.orientation.w = _qw->value();
  return pose;
}


// Creer un objet de type joints avec les consignes articulaires
sensor_msgs::JointState Panel_TP_Fanuc::compute_joints() const
{
  sensor_msgs::JointState joints;
  std::vector<double> position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  position[0] = _q1->value()*(M_PI/180.0);
  position[1] = _q2->value()*(M_PI/180.0);
  position[2] = _q3->value()*(M_PI/180.0);
  position[3] = _q4->value()*(M_PI/180.0);
  position[4] = _q5->value()*(M_PI/180.0);
  position[5] = _q6->value()*(M_PI/180.0);
  joints.position = position;
  return joints;
}

// Appele des qu'un message /joint_states est recu
void Panel_TP_Fanuc::jointsCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  if (!_blocage)
  {
    _blocage = true;
    _q1->setValue(msg->position[0]*(180.0/M_PI));
    _q2->setValue(msg->position[1]*(180.0/M_PI));
    _q3->setValue(msg->position[2]*(180.0/M_PI));
    _q4->setValue(msg->position[3]*(180.0/M_PI));
    _q5->setValue(msg->position[4]*(180.0/M_PI));
    _q6->setValue(msg->position[5]*(180.0/M_PI));
    _blocage = false;
  }
}


void Panel_TP_Fanuc::framesCallback(const tf2_msgs::TFMessage::ConstPtr& msg)
{
  tf::Vector3 position = tf::Vector3(0.0, 0.0, 0.0);
  tf::Quaternion orientation = tf::Quaternion(0.0, 0.0, 0.0, 1.0);
  geometry_msgs::Transform tf_i;
  tf::Transform frame_i = tf::Transform(orientation, position); // initialisation a l'identite
  tf::Transform frame_0n = tf::Transform(orientation, position); // initialisation a l'identite
  tf::Matrix3x3 matrix;
  double roll, pitch, yaw;

  if (!_blocage)
  {
    _blocage = true;

    // calcul de la pose de l'effecteur dans le repere de la base
    for (int i=0; i<6; i++)
    {
      tf_i = msg->transforms[i].transform;
      tf::transformMsgToTF(tf_i,frame_i); // http://docs.ros.org/en/melodic/api/tf/html/c++/namespacetf.html#a42961f01ddc521bf453f5991175f97c2
      frame_0n = frame_0n*frame_i;
    }
    position = frame_0n.getOrigin();
    orientation = frame_0n.getRotation();

    if (_position != position || _orientation != orientation) // si le robot a bougé
    {
      // mise a jour de l'affichage
      _px->setValue(position[0]);
      _py->setValue(position[1]);
      _pz->setValue(position[2]);
      _qx->setValue(orientation[0]);
      _qy->setValue(orientation[1]);
      _qz->setValue(orientation[2]);
      _qw->setValue(orientation[3]);

      // mise a jour de l'affichage des deux autres representations de l'orientation
      matrix = tf::Matrix3x3(orientation);
      matrix.getRPY(roll,pitch,yaw);
      setMatrix(matrix);
      setRPY(roll,pitch,yaw);

      // on enregistre la pose courante
      _position = position;
      _orientation = orientation;
    }

    _blocage = false;
  }
}


// Remplit les champs du quaternion
void Panel_TP_Fanuc::setQuaternion(tf::Quaternion quat)
{
  _qx->setValue(quat[0]);
  _qy->setValue(quat[1]);
  _qz->setValue(quat[2]);
  _qw->setValue(quat[3]);
}

// Remplit les champs du quaternion
void Panel_TP_Fanuc::setMatrix(tf::Matrix3x3 matrix)
{
  _rot11->setValue(matrix.getRow(0)[0]);
  _rot12->setValue(matrix.getRow(0)[1]);
  _rot13->setValue(matrix.getRow(0)[2]);
  _rot21->setValue(matrix.getRow(1)[0]);
  _rot22->setValue(matrix.getRow(1)[1]);
  _rot23->setValue(matrix.getRow(1)[2]);
  _rot31->setValue(matrix.getRow(2)[0]);
  _rot32->setValue(matrix.getRow(2)[1]);
  _rot33->setValue(matrix.getRow(2)[2]);
}

void Panel_TP_Fanuc::setRPY(double roll, double pitch, double yaw)
{
  _roll->setValue(roll*(180.0/M_PI));
  _pitch->setValue(pitch*(180.0/M_PI));
  _yaw->setValue(yaw*(180.0/M_PI));
}


/*
  SLOTS
*/

// Modification de la représentation de l'orientation avec la ComboBox
// Le signal du ComBox est connecté à cette fonction slot.
void Panel_TP_Fanuc::changer_representation_rotation(QString selection)
{
  if (selection == "Quaternion")
  {
    _widget_quat->setVisible(true);
    _widget_rot_mat->setVisible(false);
    _widget_rpy->setVisible(false);
    _representation_rotation = QUATERNION;
  }

  if (selection == "Matrice")
  {
    _widget_quat->setVisible(false);
    _widget_rot_mat->setVisible(true);
    _widget_rpy->setVisible(false);
    _representation_rotation = MATRICE_ROT;
  }

  if (selection == "Roll Pitch Yaw")
  {
    _widget_quat->setVisible(false);
    _widget_rot_mat->setVisible(false);
    _widget_rpy->setVisible(true);
    _representation_rotation = RPY;
  }

}


// Le robot va en position initiale lors de l'appui sur "Position initiale"
void Panel_TP_Fanuc::go_home()
{
  sensor_msgs::JointState msg;
  std::vector<double> position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  msg.position = position;
  _joints_publisher.publish(msg);

  // On remet les articulations à 0 :
  _blocage = true;
  _q1->setValue(0.0);
  _q2->setValue(0.0);
  _q3->setValue(0.0);
  _q4->setValue(0.0);
  _q5->setValue(0.0);
  _q6->setValue(0.0);
  _blocage = false;
}


// La trajectoire est effacée lors de l'appui sur "Effacer trajectoire"
void Panel_TP_Fanuc::clear_path() const
{
  std_msgs::String msg;
  msg.data = "clear";
  _manage_path_publisher.publish(msg);
}


// Une consigne de vitesse nulle est envoyée lors de l'appui sur "Stop"
void Panel_TP_Fanuc::stop_mvt()
{
  geometry_msgs::Twist msg;
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  _cmd_vel_publisher.publish(msg);

  // On remet la vitesse à 0 :
  _blocage = true;
  _vx->setValue(0.0);
  _vy->setValue(0.0);
  _vz->setValue(0.0);
  _wx->setValue(0.0);
  _wy->setValue(0.0);
  _wz->setValue(0.0);
  _blocage = false;
}


// L'état de _mvt est modifié lors de l'appui sur le bouton "Mouvement continu"
void Panel_TP_Fanuc::change_mvt_state(bool etat)
{
  _mvt = etat;
}


// Le robot bouge lors de l'appui sur "Bouger" avec le dernier mode de déplacement activé
void Panel_TP_Fanuc::move()
{
  switch (_mode)
  {
    case MODE_ARTI:
      send_joints();
      break;

    case MODE_CART:
      send_pose();
      break;

    case MODE_VITESSE:
      send_cmd_vel();
      break;
  }
}


// Envoi des consignes articulaires (JointState)
void Panel_TP_Fanuc::send_joints() const
{
  sensor_msgs::JointState msg = compute_joints();
  _joints_publisher.publish(msg);
}


// Fonction appelée lors de la modification d'une articulation
void Panel_TP_Fanuc::move_arti()
{
  _mode = MODE_ARTI;
  if (_mvt && !_blocage) // bouge uniquement si le bouton "Mouvement continu" est appuyé
  {
    _blocage = true;
    send_joints();
    //refresh_cart();
    _blocage = false;
  }
}


// Envoi d'une consigne cartésienne (PoseStamped)
void Panel_TP_Fanuc::send_pose()
{
  // envoi de la configuration si elle a changé
  if (_chgt_config)
  {
    send_config();
  }

  geometry_msgs::Pose msg = compute_pose();
  _pose_publisher.publish(msg);
}


// Fonction appelée lors de la modification de la position ou l'orientation
void Panel_TP_Fanuc::move_cart()
{
  _mode = MODE_CART;
  if (_mvt && !_blocage) // bouge uniquement si le bouton "Mouvement continu" est appuyé
  {
    _blocage = true;
    send_pose();
    //refresh_arti();
    //refresh_cart();
    _blocage = false;
  }
}


// Envoi d'une consigne de vitesse (Twist)
void Panel_TP_Fanuc::send_cmd_vel() const
{
  geometry_msgs::Twist msg;
  msg.linear.x = _vx->value();
  msg.linear.y = _vy->value();
  msg.linear.z = _vz->value();
  msg.angular.x = _wx->value()*(M_PI/180);
  msg.angular.y = _wy->value()*(M_PI/180);
  msg.angular.z = _wz->value()*(M_PI/180);
  _cmd_vel_publisher.publish(msg);
}


// Fonction appelée lors de la modification de la vitesse
void Panel_TP_Fanuc::move_vel()
{
  _mode = MODE_VITESSE;
  if (_mvt && !_blocage) // bouge uniquement si le bouton "Mouvement continu" est appuyé
  {
    _blocage = true;
    send_cmd_vel();
    //refresh_arti();
    //refresh_cart();
    _blocage = false;
  }
}


// Modification de la configuration articulaire
void Panel_TP_Fanuc::change_config()
{
  std::string torso;
  std::string shoulder;
  std::string wrist;

  // torso
  if (_config_torso->currentText() == "Toward (T)")         torso = "T";
  else if (_config_torso->currentText() == "Backward (B)")  torso = "B";

  // shoulder
  if (_config_shoulder->currentText() == "Up (U)")          shoulder = "U";
  else if (_config_shoulder->currentText() == "Down (D)")   shoulder = "D";

  // wrist
  if (_config_wrist->currentText() == "No-Flip (N)")        wrist = "N";
  else if (_config_wrist->currentText() == "Flip (F)")      wrist = "F";

  // config
  _config = wrist + shoulder + torso;

  // on indique que la configuration a changé
  _chgt_config = true;

  // on publie la pose si le robot est autorisé à bouger
  //if (_mode == MODE_CART and _mvt)
  if (_mvt)
  {
    send_pose();
    //send_config();
  }

}


// Envoi de la configuration articulaire
void Panel_TP_Fanuc::send_config()
{
  std_msgs::String msg;
  msg.data = _config;
  _config_publisher.publish(msg);
  _chgt_config = false;
}


// Modification des valeurs des articulations sur l'affichage
void Panel_TP_Fanuc::refresh_arti()
{

}


// Modification des valeurs de la pose sur l'affichage
void Panel_TP_Fanuc::refresh_cart()
{

}


// Mise a jour des valeurs d'orientation pour toutes les representations
void Panel_TP_Fanuc::refresh_orientation()
{
    tf::Quaternion quat;
    tf::Matrix3x3 matrix;
    double roll, pitch, yaw;

  switch (_representation_rotation)
  {
    case QUATERNION:
      quat = tf::Quaternion(_qx->value(),_qy->value(),_qz->value(),_qw->value());
      matrix = tf::Matrix3x3(quat);
      matrix.getRPY(roll,pitch,yaw);
      setMatrix(matrix);
      setRPY(roll,pitch,yaw);
      break;

    case MATRICE_ROT:
      matrix = tf::Matrix3x3(_rot11->value(),_rot12->value(),_rot13->value(),_rot21->value(),_rot22->value(),_rot23->value(),_rot31->value(),_rot32->value(),_rot33->value());
      matrix.getRotation(quat);
      matrix.getRPY(roll,pitch,yaw);
      setQuaternion(quat);
      setRPY(roll,pitch,yaw);
      break;

    case RPY:
      quat.setRPY(_roll->value()*(M_PI/180.0),_pitch->value()*(M_PI/180.0),_yaw->value()*(M_PI/180.0));
      matrix.setRPY(_roll->value()*(M_PI/180.0),_pitch->value()*(M_PI/180.0),_yaw->value()*(M_PI/180.0));
      setQuaternion(quat);
      setMatrix(matrix);
      break;
  }
}

// gestion de l'affichage du panneau de commande de consigne articulaire
void Panel_TP_Fanuc::display_cmd_arti()
{
  _widget_arti->setVisible(_box_arti->isChecked());
}

// gestion de l'affichage du panneau de commande de consigne operationnelle
void Panel_TP_Fanuc::display_cmd_op()
{
  _widget_op->setVisible(_box_op->isChecked());
}

// gestion de l'affichage du panneau de commande de vitesse
void Panel_TP_Fanuc::display_cmd_vel()
{
  _widget_vel->setVisible(_box_vel->isChecked());
}



} // namespace



// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(rviz_plugins_robexp::Panel_TP_Fanuc,rviz::Panel )
