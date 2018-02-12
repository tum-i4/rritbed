#!/usr/bin/env python
"""
Rebuilding turtle_frame.cpp in Python
"""

# Currently not supported:
# - graphical output
#   - 2D space
#   - turtle images
# - services
#   - clear
#   - reset
#   - spawn
#   - kill

DEFAULT_BG_R = 0x45
DEFAULT_BG_G = 0x56
DEFAULT_BG_B = 0xff


class TurtleFrame(object):
	""" The turtle frame class """

	# path_image_(500, 500, QImage::Format_ARGB32)
	# path_painter_(&path_image_)
	# frame_count_(0)
	# id_counter_(0)

	# ros::NodeHandle nh_;
	# QTimer* update_timer_;
	# QImage path_image_;
	# QPainter path_painter_;

	# uint64_t frame_count_;

	# ros::WallTime last_turtle_update_;

	# ros::ServiceServer clear_srv_;
	# ros::ServiceServer reset_srv_;
	# ros::ServiceServer spawn_srv_;
	# ros::ServiceServer kill_srv_;

	# typedef std::map<std::string, TurtlePtr> M_Turtle;
	# M_Turtle turtles_;
	# uint32_t id_counter_;

	# QVector<QImage> turtle_images_;

	# float meter_;
	# float width_in_meters_;
	# float height_in_meters_;

	def __init__(self):
		""" Ctor """

		object.__init__(self)

		# Init window

		#   setFixedSize(500, 500);
		#   setWindowTitle("TurtleSim");

		# Init update timer (16 msec)

		#   srand(time(NULL));

		#   update_timer_ = new QTimer(this);
		#   update_timer_->setInterval(16);
		#   update_timer_->start();

		#   connect(update_timer_, SIGNAL(timeout()), this, SLOT(onUpdate()));

		#   nh_.setParam("background_r", DEFAULT_BG_R);
		#   nh_.setParam("background_g", DEFAULT_BG_G);
		#   nh_.setParam("background_b", DEFAULT_BG_B);

		#   QString images_path = (ros::package::getPath("turtlesim") + "/images/").c_str();
		#   /*for (int i = 0; i < turtles.size(); ++i)
		#   {
		#     QImage img;
		#     img.load(images_path + turtles[i]);
		#     turtle_images_.append(img);
		#   }*/

		#   // TODO
		#   QImage img;
		#   img.load(images_path + turtles[0]);
		#   turtle_images_.append(img);

		#   // END TODO

		#   meter_ = turtle_images_[0].height();

		#   clear();

		#   clear_srv_ = nh_.advertiseService("clear", &TurtleFrame::clearCallback, this);
		#   reset_srv_ = nh_.advertiseService("reset", &TurtleFrame::resetCallback, this);
		#   spawn_srv_ = nh_.advertiseService("spawn", &TurtleFrame::spawnCallback, this);
		#   kill_srv_ = nh_.advertiseService("kill", &TurtleFrame::killCallback, this);

		#   ROS_INFO("LET'S GOOOOOO, %s", ros::this_node::getName().c_str()) ;

		#   width_in_meters_ = (width() - 1) / meter_;
		#   height_in_meters_ = (height() - 1) / meter_;
		#   // spawnTurtle("", width_in_meters_ / 2.0, height_in_meters_ / 2.0, 0);

		#   // TODO
		#   float x_pct = (rand() % 101) / 100.0; // * width_in_meters_;
		#   float y_pct = (rand() % 101) / 100.0; // * height_in_meters_;

		#   spawnTurtle("", width_in_meters_ * x_pct, height_in_meters_ * y_pct, 0);

		#   // === VAU ===
		#   // Window is 500 x 500, starting top left at 0,0 and ending bottom right at 500,500

		#   // TOP LEFT: PURPLE
		#   QColor pastelPurple = QColor(150, 125, 210);
		#   vauDrawArea(pastelPurple, QPoint(0, 0), QPoint(250, 250));

		#   // // TOP RIGHT: YELLOW
		#   QColor pastelYellow = QColor(255, 240, 120);
		#   vauDrawArea(pastelYellow, QPoint(250, 0), QPoint(500, 250));

		#   // // BOTTOM LEFT: GREEN
		#   QColor pastelGreen = QColor(100, 180, 100);
		#   vauDrawArea(pastelGreen, QPoint(0, 250), QPoint(250, 500));

		#   // // BOTTOM RIGHT: BLUE
		#   QColor pastelBlue = QColor(100, 180, 250);
		#   vauDrawArea(pastelBlue, QPoint(250, 250), QPoint(500, 500));

		#   // DANGER ZONE MIDDLE: RED
		#   vauDrawArea(Qt::red, QPoint(245, 245), QPoint(255, 255));

		#   // END TODO

		#   // spawn all available turtle types
		#   if(false)
		#   {
		#     for(int index = 0; index < turtles.size(); ++index)
		#     {
		#       QString name = turtles[index];
		#       name = name.split(".").first();
		#       name.replace(QString("-"), QString(""));
		#       spawnTurtle(name.toStdString(), 1.0 + 1.5 * (index % 7), 1.0 + 1.5 * (index / 7), PI / 2.0, index);
		#     }
		#   }


	def drawArea(self):
		""" Draw defined area in defined colour """
		pass

		# void TurtleFrame::vauDrawArea(QColor vauColour, QPoint vauTopLeft, QPoint vauBottomRight)
		# {
		# path_painter_.setPen(QPen(vauColour, 1, Qt::SolidLine, Qt::SquareCap));
		# path_painter_.setBrush(vauColour);

		# // === TL ===       BR x, TL y
		# // TL x, BR y       === BR ===

		# QPoint points[4] = {
		# 	vauTopLeft,
		# 	QPoint(vauBottomRight.x(), vauTopLeft.y()),
		# 	vauBottomRight,
		# 	QPoint(vauTopLeft.x(), vauBottomRight.y())
		# };

		# path_painter_.drawPolygon(points, 4);
		# }
