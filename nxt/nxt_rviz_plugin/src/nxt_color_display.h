#ifndef NXT_COLOR_DISPLAY_H
#define NXT_COLOR_DISPLAY_H

//#include "rviz/display.h"
//#include "rviz/helpers/color.h"
//#include "rviz/selection/forwards.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/status_property.h"
#include "rviz/properties/ros_topic_property.h"
#include "rviz/message_filter_display.h"
//#include "rviz/common.h"
#include "rviz/frame_manager.h"
//#include "rviz/validate_floats.h"
#include "rviz/ogre_helpers/shape.h"

#include <nxt_msgs/Color.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include <boost/circular_buffer.hpp>
#include <boost/shared_ptr.hpp>

namespace Ogre
{
   class SceneNode;
}

namespace nxt_rviz_plugin
{
   class NXTColorVisual;

/**
 * \class NXTColorDisplay
 * \brief Displays a nxt_msgs::Color message
 */
//class NXTColorDisplay : public rviz::Display
   class NXTColorDisplay : public rviz::MessageFilterDisplay<nxt_msgs::Color>
   {
      Q_OBJECT

   public:
      //NXTColorDisplay( const std::string& name, rviz::VisualizationManager* manager );
      NXTColorDisplay();
      virtual ~NXTColorDisplay();

  //void setTopic( const std::string& topic );
  //const std::string& getTopic() { return topic_; }

  //void setAlpha( float alpha );
  //float getAlpha() { return alpha_; }

  //void setDisplayLength( float displayLength );
  //float getDisplayLength() { return displayLength_; }

  // Overrides from Display
  //virtual void targetFrameChanged() {}
  //virtual void fixedFrameChanged();
  virtual void createProperties();
  //virtual void update(float wall_dt, float ros_dt);
  //virtual void reset();

  //static const char* getTypeStatic() { return "Color"; }
  //virtual const char* getType() const { return getTypeStatic(); }
  QString getDescription() const;

   protected:
  //void subscribe();
  //void unsubscribe();
  //void clear();
      /** @brief Implement this to process the contents of a message.
      *
      * This is called by incomingMessage(). */
      void processMessage(const nxt_msgs::Color::ConstPtr& msg);

      // overrides from Display
  //virtual void onEnable();
  //virtual void onDisable();
      virtual void onInitialize();
      // A helper to clear this display back to the initial state.
      virtual void reset();

  //std::string topic_;
  //float alpha_;
  float displayLength_;

  //uint32_t messages_received_;

      // A node in the Ogre scene tree to be the parent of all our visuals.
      Ogre::SceneNode* scene_node_;
  //rviz::Shape* cylinder_;      ///< Handles actually drawing the cone

  //message_filters::Subscriber<nxt_msgs::Color> sub_;
  //tf::MessageFilter<nxt_msgs::Color> tf_filter_;
  //nxt_msgs::Color::ConstPtr current_message_;

   private Q_SLOTS:
//      void updateAlpha();
//      void updateDisplayLength();
      void updateCylinder();

   private:
      bool color_enabled_;
//      // Storage for the list of visuals. It is a circular buffer where
//      // data gets popped from the front (oldest) and pushed to the back (newest)
//      boost::circular_buffer<boost::shared_ptr<NXTColorVisual> > visuals_;
      NXTColorVisual* color_visual_;
  //rviz::RosTopicProperty* topic_property_;
      rviz::BoolProperty* cylinder_enabled_property_;
      rviz::FloatProperty* alpha_property_;
      rviz::FloatProperty* display_property_;
      int messages_received_;
   };

} // namespace nxt_rviz_plugin

#endif // NXT_COLOR_DISPLAY_H

