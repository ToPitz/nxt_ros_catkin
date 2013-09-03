#include "nxt_color_display.h"
#include "nxt_color_visual.h"

#include <tf/transform_listener.h>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

namespace nxt_rviz_plugin
{
//NXTColorDisplay::NXTColorDisplay( const std::string& name, rviz::VisualizationManager* manager )
//  : Display()
//  , messages_received_(0)
//  , tf_filter_(*manager->getTFClient(), "", 10, update_nh_)
   NXTColorDisplay::NXTColorDisplay() :
      scene_node_(NULL),
      color_enabled_(false),
      messages_received_(0)
   {
      createProperties();

      //scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  //cylinder_ = new rviz::Shape(rviz::Shape::Cylinder, vis_manager_->getSceneManager(), scene_node_);
//      cylinder_ = new rviz::Shape(rviz::Shape::Cylinder, manager->getSceneManager(), scene_node_);
//
//      scene_node_->setVisible( false );
//
//      setAlpha( 0.5f );
//      setDisplayLength( 0.003f );
//
//      Ogre::Vector3 scale( 0, 0, 0);
//  //rviz::scaleRobotToOgre( scale );
//      cylinder_->setScale(scale);
//
//      tf_filter_.connectInput(sub_);
//      tf_filter_.registerCallback(boost::bind(&NXTColorDisplay::incomingMessage, this, _1));
//      manager->getFrameManager()->registerFilterForTransformStatusCheck(tf_filter_, this);
   }

   NXTColorDisplay::~NXTColorDisplay()
   {
   }

   // Clear the visuals by deleting their objects.
   void NXTColorDisplay::reset()
   {
      MFDClass::reset();
      messages_received_ = 0;
      setStatus(rviz::StatusProperty::Warn, "Topic", "No messages received" );
      color_visual_->hide();
//      visuals_.clear();
   }

   void NXTColorDisplay::onInitialize()
   {
      MFDClass::onInitialize();

      // Make an Ogre::SceneNode to contain all our visuals.
      scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

      color_visual_ = new NXTColorVisual(context_->getSceneManager(), scene_node_);
      if (color_enabled_)
        color_visual_->show();
      else
        color_visual_->hide();

//      // We are keeping a circular buffer of visual pointers. This gets
//      // the next one, or creates and stores it if the buffer is not full
//      boost::shared_ptr<NXTColorVisual> visual;
//      if( visuals_.full() )
//      {
//         visual = visuals_.front();
//      }
//      else
//      {
//         visual.reset(new NXTColorVisual( context_->getSceneManager(), scene_node_ ));
//      }
   }

//void NXTColorDisplay::clear()
//{
//
//  messages_received_ = 0;
//  setStatus(rviz::StatusProperty::Warn, "Topic", "No messages received");
//}
//
//void NXTColorDisplay::setTopic( const std::string& topic )
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
//void NXTColorDisplay::setAlpha( float alpha )
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
//void NXTColorDisplay::setDisplayLength( float displayLength )
//{
//  displayLength_ = displayLength;
//
//  propertyChanged(display_property_);
//
//  processMessage(current_message_);
//  //causeRender();
//  context_->queueRender();
//}


//void NXTColorDisplay::subscribe()
//{
//  if ( !isEnabled() )
//  {
//    return;
//  }
//
//  sub_.subscribe(update_nh_, topic_, 10);
//}

//void NXTColorDisplay::unsubscribe()
//{
//  sub_.unsubscribe();
//}

//void NXTColorDisplay::onEnable()
//{
//  scene_node_->setVisible( true );
//  subscribe();
//}
//
//void NXTColorDisplay::onDisable()
//{
//  unsubscribe();
//  clear();
//  scene_node_->setVisible( false );
//}

//void NXTColorDisplay::fixedFrameChanged()
//{
//  clear();
//
//  tf_filter_.setTargetFrame( fixed_frame_.toStdString() );
//}

//void NXTColorDisplay::update(float wall_dt, float ros_dt)
//{
//}

      void NXTColorDisplay::updateCylinder()
      {
         color_enabled_ = cylinder_enabled_property_->getBool();
         if (color_enabled_)
         {
            color_visual_->show();
         }
         else
         {
            color_visual_->hide();
         }

         color_visual_->setAlpha(alpha_property_->getFloat());
      }

//   void NXTColorDisplay::updateAlpha()
//   {
//      float alpha = alpha_property_->getFloat();
//
//      for( size_t i = 0; i < visuals_.size(); i++ )
//      {
//         visuals_[ i ]->setAlpha( alpha );
//      }
//   }
//
//   void NXTColorDisplay::updateDisplayLength()
//   {
//      float displayLength_ = display_property_->getFloat();
//
////      for( size_t i = 0; i < visuals_.size(); i++ )
////      {
////         visuals_[ i ]->setDisplay( displayLength );
////      }
//   }

   void NXTColorDisplay::processMessage(const nxt_msgs::Color::ConstPtr& msg)
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
      pose.position.z = -0.0033;
      pose.position.y = 0;
      pose.position.x = 0.0185 + displayLength_/2;
      pose.orientation.x = 0.707;
      pose.orientation.z = -0.707;
  //if (!vis_manager_->getFrameManager()->transform(msg->header.frame_id,msg->header.stamp,pose, position, orientation, true))
  //{
  //  ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), fixed_frame_.toStdString().c_str() );
  //}

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

//      // We are keeping a circular buffer of visual pointers. This gets
//      // the next one, or creates and stores it if the buffer is not full
//      boost::shared_ptr<NXTColorVisual> visual;
//      if( visuals_.full() )
//      {
//         visual = visuals_.front();
//      }
//      else
//      {
//         visual.reset(new NXTColorVisual( context_->getSceneManager(), scene_node_ ));
//      }

//      cylinder_->setPosition(position);
//      cylinder_->setOrientation(orientation);
//      Ogre::Vector3 scale( 0.0155, 0.0155, displayLength_);
//  //rviz::scaleRobotToOgre( scale );
//      cylinder_->setScale(scale);
//      cylinder_->setColor(msg->r, msg->g, msg->b, alpha_);

      if (color_enabled_)
      {
         // Now set or update the contents of the chosen visual.
         color_visual_->setMessage( msg );
         color_visual_->setFramePosition( position );
         color_visual_->setFrameOrientation( orientation );
         color_visual_->show();
      }

//      // Now set or update the contents of the chosen visual.
//      visual->setMessage( msg );
//      visual->setFramePosition( position );
//      visual->setFrameOrientation( orientation );
//
//      updateCylinder();
//
//      // And send it to the end of the circular buffer
//      visuals_.push_back(visual);
   }

//void NXTColorDisplay::incomingMessage(const nxt_msgs::Color::ConstPtr& msg)
//{
//  processMessage( msg );
//}

   void NXTColorDisplay::createProperties()
   {
      alpha_property_ = new rviz::FloatProperty( "Alpha", 0.5f,
                                                "0 is fully transparent, 1.0 is fully opaque.",
                                                this, SLOT( updateCylinder() ));
      display_property_ = new rviz::FloatProperty( "Display", 0.003f,
                                                "0 is minimum, 1.0 is maximum.",
                                                this, SLOT( updateCylinder() ));
      cylinder_enabled_property_ = new rviz::BoolProperty( "Enable color", color_enabled_,
                                                          "Enable Ring of Color", this,
                                                          SLOT(updateCylinder()));

//      topic_property_ = property_manager_->createProperty<rviz::RosTopicProperty>( "Topic", property_prefix_, boost::bind( &NXTColorDisplay::getTopic, this ),
//                                                                                 boost::bind( &NXTColorDisplay::setTopic, this, _1 ), parent_category_, this );
//      setPropertyHelpText(topic_property_, "nxt_msgs::Color topic to subscribe to.");
//      rviz::RosTopicPropertyPtr topic_prop = topic_property_.lock();
//      topic_prop->setMessageType(ros::message_traits::datatype<nxt_msgs::Color>());
//
//
//      alpha_property_ = property_manager_->createProperty<rviz::FloatProperty>("Alpha", property_prefix_,
//                                                                               boost::bind( &NXTColorDisplay::getAlpha, this ),
//                                                                               boost::bind( &NXTColorDisplay::setAlpha, this, _1 ),
//                                                                               parent_category_, this);
//
//      display_property_ = property_manager_->createProperty<rviz::FloatProperty>("Display Length", property_prefix_,
//                                                                                 boost::bind( &NXTColorDisplay::getDisplayLength, this ),
//                                                                                 boost::bind( &NXTColorDisplay::setDisplayLength, this, _1 ),
//                                                                                 parent_category_, this);
//
//      setPropertyHelpText(alpha_property_, "Amount of transparency to apply to the circle.");
   }

   QString NXTColorDisplay::getDescription() const
   {
      return "Displays data from a nxt_msgs::Color message as a cirle of color.";
   }
} // namespace nxt_rviz_plugin

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(nxt_rviz_plugin::NXTColorDisplay, rviz::Display)
