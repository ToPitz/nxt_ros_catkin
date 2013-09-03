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

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include "rviz/ogre_helpers/shape.h"

#include "nxt_ultrasonic_visual.h"

namespace nxt_rviz_plugin
{

   NXTUltrasonicVisual::NXTUltrasonicVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node ) :
      cone_(NULL),
      scale_(0.05),
      alpha_(0.5),
      color_(0.5, 0.5, 0.5)
   {
      scene_manager_ = scene_manager;

      // Ogre::SceneNode s form a tree, with each node storing the
      // transform (position and orientation) of itself relative to its
      // parent. Ogre does the math of combining those transforms when it
      // is time to render.
      //
      // Here we create a node to store the pose of the Imu's header frame
      // relative to the RViz fixed frame.
      frame_node_ = parent_node->createChildSceneNode();

//      // We create the cylinder object within the frame node so that we can
//      // set its position and direction relative to its header frame.
//      cone_.reset(new rviz::Shape::Cone( scene_manager_, frame_node_ ));
   }

   NXTUltrasonicVisual::~NXTUltrasonicVisual()
   {
      hide();
      // Destroy the frame node since we don't need it anymore.
      scene_manager_->destroySceneNode( frame_node_ );
   }

   void NXTUltrasonicVisual::show()
   {
      if (!cone_)
      {
         cone_ = new rviz::Shape(rviz::Shape::Cone, scene_manager_, frame_node_);
         cone_->setColor(color_.redF(), color_.greenF(), color_.blueF(), alpha_);
      }
   }

   void NXTUltrasonicVisual::hide()
   {
      if (cone_)
      {
         delete cone_;
         cone_ = NULL;
      }
   }

   void NXTUltrasonicVisual::setMessage( const nxt_msgs::Range::ConstPtr& msg )
   {
//      // Scale the cone's dimensions
//      cone_->setPosition(position);
//      cone_->setOrientation(orientation);
      Ogre::Vector3 scale( sin(msg->spread_angle) * msg->range, sin(msg->spread_angle) * msg->range , msg->range);
      cone_->setScale(scale);
      cone_->setColor(color_.redF(), color_.greenF(), color_.blueF(), alpha_);
   }

   // Position and orientation are passed through to the SceneNode.
   void NXTUltrasonicVisual::setFramePosition( const Ogre::Vector3& position )
   {
      frame_node_->setPosition( position );
   }

   void NXTUltrasonicVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
   {
      frame_node_->setOrientation( orientation );
   }

   void NXTUltrasonicVisual::setScale(float scale)
   {
      scale_ = scale;
      if (cone_)
      {
         Ogre::Vector3 scale( 0.0155, 0.0155, display_);
         cone_->setScale(scale);
      }
   }

   void NXTUltrasonicVisual::setDisplay( float display )
   {
      display_ = display;
   }

   void NXTUltrasonicVisual::setAlpha( float alpha )
   {
      alpha_ = alpha;
      if (cone_)
      {
         cone_->setColor(color_.redF(), color_.greenF(), color_.blueF(), alpha_);
      }
   }

   void NXTUltrasonicVisual::setColor(const QColor& color)
   {
      color_ = color;
      if (cone_)
      {
         cone_->setColor(color_.redF(), color_.greenF(), color_.blueF(), alpha_);
      }
   }

} // end namespace nxt_rviz_plugin
