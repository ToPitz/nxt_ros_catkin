#include "nxt_ultrasonic_display.h"
#include "nxt_ultrasonic_visual.h"

#include <tf/transform_listener.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace nxt_rviz_plugin
{
//NXTUltrasonicDisplay::NXTUltrasonicDisplay( const std::string& name, rviz::VisualizationManager* manager )
//: Display()
//, color_( 0.1f, 1.0f, 0.0f )
//, messages_received_(0)
//, tf_filter_(*manager->getTFClient(), "", 10, update_nh_)
   NXTUltrasonicDisplay::NXTUltrasonicDisplay() :
      scene_node_(NULL),
      ultrasonic_enabled_(false),
      messages_received_(0)
   {
      createProperties();

//  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
//
//  //cone_ = new rviz::Shape(rviz::Shape::Cone, vis_manager_->getSceneManager(), scene_node_);
//  cone_ = new rviz::Shape(rviz::Shape::Cone, manager->getSceneManager(), scene_node_);
//
//  scene_node_->setVisible( false );
//
//  setAlpha( 0.5f );
//  Ogre::Vector3 scale( 0, 0, 0);
//  //rviz::scaleRobotToOgre( scale );
//  cone_->setScale(scale);
//  cone_->setColor(color_.r_, color_.g_, color_.b_, alpha_);
//
//  tf_filter_.connectInput(sub_);
//  tf_filter_.registerCallback(boost::bind(&NXTUltrasonicDisplay::incomingMessage, this, _1));
//  manager->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
   }

   NXTUltrasonicDisplay::~NXTUltrasonicDisplay()
   {
//  unsubscribe();
//  clear();
//  delete cone_;
   }

   // Clear the visuals by deleting their objects.
   void NXTUltrasonicDisplay::reset()
   {
      MFDClass::reset();
      messages_received_ = 0;
      setStatus(rviz::StatusProperty::Warn, "Topic", "No messages received" );
      ultrasonic_visual_->hide();
   }

   void NXTUltrasonicDisplay::onInitialize()
   {
      MFDClass::onInitialize();

      // Make an Ogre::SceneNode to contain all our visuals.
      scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

      ultrasonic_visual_ = new NXTUltrasonicVisual(context_->getSceneManager(), scene_node_);
      if (ultrasonic_enabled_)
        ultrasonic_visual_->show();
      else
        ultrasonic_visual_->hide();

//      // We are keeping a circular buffer of visual pointers. This gets
//      // the next one, or creates and stores it if the buffer is not full
//      boost::shared_ptr<NXTUltrasonicVisual> visual;
//      if( visuals_.full() )
//      {
//         visual = visuals_.front();
//      }
//      else
//      {
//         visual.reset(new NXTUltrasonicVisual( context_->getSceneManager(), scene_node_ ));
//      }
   }
//
//void NXTUltrasonicDisplay::clear()
//{
//
//  messages_received_ = 0;
//  setStatus(rviz::StatusProperty::Warn, "Topic", "No messages received");
//}
//
//void NXTUltrasonicDisplay::setTopic( const std::string& topic )
//{
//  unsubscribe();
//
//  topic_ = topic;
//
//  subscribe();
//
//  propertyChanged(topic_property_);
//
//  //causeRender();
//  context_->queueRender();
//}
//
//void NXTUltrasonicDisplay::setColor( const rviz::Color& color )
//{
//  color_ = color;
//
//  propertyChanged(color_property_);
//
//  processMessage(current_message_);
//  //causeRender();
//  context_->queueRender();
//}
//
//void NXTUltrasonicDisplay::setAlpha( float alpha )
//{
//  alpha_ = alpha;
//
//  propertyChanged(alpha_property_);
//
//  processMessage(current_message_);
//  //causeRender();
//  context_->queueRender();
//}
//
//void NXTUltrasonicDisplay::subscribe()
//{
//  if ( !isEnabled() )
//  {
//    return;
//  }
//
//  sub_.subscribe(update_nh_, topic_, 10);
//}
//
//void NXTUltrasonicDisplay::unsubscribe()
//{
//  sub_.unsubscribe();
//}
//
//void NXTUltrasonicDisplay::onEnable()
//{
//  scene_node_->setVisible( true );
//  subscribe();
//}
//
//void NXTUltrasonicDisplay::onDisable()
//{
//  unsubscribe();
//  clear();
//  scene_node_->setVisible( false );
//}
//
//void NXTUltrasonicDisplay::fixedFrameChanged()
//{
//  clear();
//
//  tf_filter_.setTargetFrame( fixed_frame_.toStdString() );
//}
//
//void NXTUltrasonicDisplay::update(float wall_dt, float ros_dt)
//{
//}

   void NXTUltrasonicDisplay::updateCone()
   {
      ultrasonic_enabled_ = cone_enabled_property_->getBool();
      if (ultrasonic_enabled_)
      {
         ultrasonic_visual_->show();
      }
      else
      {
         ultrasonic_visual_->hide();
      }

      ultrasonic_visual_->setAlpha(alpha_property_->getFloat());
      ultrasonic_visual_->setColor(color_property_->getColor());
   }

   void NXTUltrasonicDisplay::processMessage(const nxt_msgs::Range::ConstPtr& msg)
   {
      if (!msg)
      {
         return;
      }

      ++messages_received_;

      {
         QString messageString;
         messageString = QString::number(messages_received_) + " messages received";
         setStatus(rviz::StatusProperty::Ok, "Topic", messageString);
      }

      Ogre::Vector3 position;
      Ogre::Quaternion orientation;
      geometry_msgs::Pose pose;
      pose.position.z = pose.position.y = 0;
      pose.position.x = msg->range/2;
      pose.orientation.x = 0.707;
      pose.orientation.z = -0.707;
//  if (!vis_manager_->getFrameManager()->transform(msg->header.frame_id,msg->header.stamp,pose, position, orientation, true))
//  {
//    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.toStdString().c_str() );
//  }



      // Here we call the rviz::FrameManager to get the transform from the
      // fixed frame to the frame in the header of this NXTColor message. If
      // it fails, we can't do anything else so we return.
      if( !context_->getFrameManager()->transform( msg->header.frame_id,
                                                   msg->header.stamp,
                                                   pose, position, orientation ))
      {
         ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
                  msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
         return;
      }
//
//      // We are keeping a circular buffer of visual pointers. This gets
//      // the next one, or creates and stores it if the buffer is not full
//      boost::shared_ptr<NXTUltrasonicVisual> visual;
//      if( visuals_.full() )
//      {
//         visual = visuals_.front();
//      }
//      else
//      {
//         visual.reset(new NXTUltrasonicVisual( context_->getSceneManager(), scene_node_ ));
//      }

//  cone_->setPosition(position);
//  cone_->setOrientation(orientation);
//  Ogre::Vector3 scale( sin(msg->spread_angle) * msg->range, sin(msg->spread_angle) * msg->range , msg->range);
//  //rviz::scaleRobotToOgre( scale );
//  cone_->setScale(scale);
//  cone_->setColor(color_.r_, color_.g_, color_.b_, alpha_);

      if (ultrasonic_enabled_)
      {
         // Now set or update the contents of the chosen visual.
         ultrasonic_visual_->setMessage( msg );
         ultrasonic_visual_->setFramePosition( position );
         ultrasonic_visual_->setFrameOrientation( orientation );
         ultrasonic_visual_->show();
      }

//      updateAlpha();
//
//      // And send it to the end of the circular buffer
//      visuals_.push_back(visual);
   }

//void NXTUltrasonicDisplay::incomingMessage(const nxt_msgs::Range::ConstPtr& msg)
//{
//  processMessage(msg);
//}

   void NXTUltrasonicDisplay::createProperties()
   {
      alpha_property_ = new rviz::FloatProperty( "Alpha", 0.5f,
                                                "0 is fully transparent, 1.0 is fully opaque.",
                                                this, SLOT( updateCone() ));
      display_property_ = new rviz::FloatProperty( "Display", 0.003f,
                                                "0 is minimum, 1.0 is maximum.",
                                                this, SLOT( updateCone() ));

      cone_enabled_property_ = new rviz::BoolProperty("Enable ultrasonic", ultrasonic_enabled_,
                                                      "Enable cone of Ultrasonic sensor range",
                                                      this, SLOT(updateCone()));

//  topic_property_ = property_manager_->createProperty<rviz::RosTopicProperty>( "Topic", property_prefix_, boost::bind( &NXTUltrasonicDisplay::getTopic, this ),
//                                                                                boost::bind( &NXTUltrasonicDisplay::setTopic, this, _1 ), parent_category_, this );
//  setPropertyHelpText(topic_property_, "nxt_msgs::Range topic to subscribe to.");
//  rviz::RosTopicPropertyPtr topic_prop = topic_property_.lock();
//  topic_prop->setMessageType(ros::message_traits::datatype<nxt_msgs::Range>());
//  color_property_ = property_manager_->createProperty<rviz::ColorProperty>( "Color", property_prefix_, boost::bind( &NXTUltrasonicDisplay::getColor, this ),
//                                                                      boost::bind( &NXTUltrasonicDisplay::setColor, this, _1 ), parent_category_, this );
//  setPropertyHelpText(color_property_, "Color to draw the range.");
//  alpha_property_ = property_manager_->createProperty<rviz::FloatProperty>( "Alpha", property_prefix_, boost::bind( &NXTUltrasonicDisplay::getAlpha, this ),
//                                                                       boost::bind( &NXTUltrasonicDisplay::setAlpha, this, _1 ), parent_category_, this );
//  setPropertyHelpText(alpha_property_, "Amount of transparency to apply to the range.");
   }

QString NXTUltrasonicDisplay::getDescription() const
{
  return "Displays data from a nxt_msgs::Range message as a cone.";
}
} // namespace nxt_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(nxt_rviz_plugin::NXTUltrasonicDisplay, rviz::Display)
