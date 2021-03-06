/*
* Copyright (c) 2012, Willow Garage, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in the
* documentation and/or other materials provided with the distribution.
* * Neither the name of the Willow Garage, Inc. nor the names of its
* contributors may be used to endorse or promote products derived from
* this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef NXT_ULTRASONIC_VISUAL_H
#define NXT_ULTRASONIC_VISUAL_H

#include <nxt_msgs/Range.h>
#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <rviz/ogre_helpers/shape.h>
#include <QColor>

namespace Ogre
{
   class Vector3;
   class Quaternion;
}

namespace rviz
{
   class Shape;
}

namespace nxt_rviz_plugin
{
   // Declare the visual class for this display.
   //
   // Each instance of NXTUltrasonicVisual represents the visualization of a single
   // nxt_msgs::Range message.
   class NXTUltrasonicVisual
   {
   public:
      // Constructor. Creates the visual stuff and puts it into the
      // scene, but in an unconfigured state.
      NXTUltrasonicVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node );

      // Destructor. Removes the visual stuff from the scene.
      virtual ~NXTUltrasonicVisual();

      // Configure the visual to show the data in the message.
      void setMessage( const nxt_msgs::Range::ConstPtr& msg );

      // Set the pose of the coordinate frame the message refers to.
      // These could be done inside setMessage(), but that would require
      // calls to FrameManager and error handling inside setMessage(),
      // which doesn't seem as clean. This way NXTUltrasonicVisual is only
      // responsible for visualization.
      void setFramePosition( const Ogre::Vector3& position );
      void setFrameOrientation( const Ogre::Quaternion& orientation );

      void setScale(float scale);
      void setDisplay( float display );
      // Set the color and alpha of the visual, which are user-editable
      // parameters and therefore don't come from the NXT Range message.
      void setColor(const QColor &color);
      void setAlpha( float alpha );

      void show();
      void hide();

   private:
      void create();
      // The object implementing the actual cylinder shape
      rviz::Shape* cone_;

      // A SceneNode whose pose is set to match the coordinate frame of
      // the NXT Ultrasonic Sensor message header.
      Ogre::SceneNode* frame_node_;

      // The SceneManager, kept here only so the destructor can ask it to
      // destroy the ``frame_node_``.
      Ogre::SceneManager* scene_manager_;

      float scale_;
      float display_;
      float alpha_;
      QColor color_;
   };

} // end namespace nxt_rviz_plugin

#endif // NXT_ULTRASONIC_VISUAL_H
