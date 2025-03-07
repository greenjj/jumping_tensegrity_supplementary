/*
 * Copyright © 2012, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All rights reserved.
 * 
 * The NASA Tensegrity Robotics Toolkit (NTRT) v1 platform is licensed
 * under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0.
 * 
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific language
 * governing permissions and limitations under the License.
 */

/**
 * @file LengthControllerYAML.cpp
 * @brief Implementation of LengthControllerYAML.
 * @author Dario Bozinovski  adapted from Drew Sabelhaus
 * $Id$
 */

// This module
#include "LengthControllerdarYAMLdir.h"
// This application
#include "yamlbuilder/TensegrityModel.h"
// This library
#include "core/tgBasicActuator.h"
#include "core/tgSpringCableActuator.h"
#include "core/tgString.h"
#include "core/tgTags.h"

//#include "sensors/tgDataObserver.h"
// The C++ Standard Library
#include <cassert>
#include <stdexcept>
#include <vector>
#include <iostream>
#include "helpers/FileHelpers.h"
#include <stdexcept>

// Constructor assigns variables, does some simple sanity checks.
// Also, initializes the accumulator variable timePassed so that it can
// be incremented in onStep.

LengthControllerYAML::LengthControllerYAML(double startTime,
					   double minLength,
					   double rate,
             double jumpTime,
             double jumpdelay,
             double extra1,
             double extra2,
             double extra3,
					   std::vector<std::string> tagsToControl):
  m_startTime(startTime),
  m_jumpTime(jumpTime),
  m_jumpdelay(jumpdelay),
  m_minLength(minLength),
  m_rate(rate),
  m_tagsToControl(tagsToControl),
  m_timePassed(0.0),
  m_extra1(extra1),
  m_extra2(extra2),
  m_extra3(extra3),
  Ijumped(0)
  
{
  // start time must be greater than or equal to zero
  if( m_startTime < 0.0 ) {
    throw std::invalid_argument("Start time must be greater than or equal to zero.");
  }
  // min length must be between 1 and 0
  else if( m_minLength > 1 ) {
    throw std::invalid_argument("minLength is a percent, must be less than 1. (100%)");
  }
  else if( m_minLength < 0.0) {
    throw std::invalid_argument("minLength is a percent, must be greater than 0.");
  }
  // rate must be greater than zero
  else if( m_rate < 0.0 ) {
    throw std::invalid_argument("Rate cannot be negative.");
  }
  // jump time must be greater than start time
  else if( m_jumpTime < m_startTime &&m_jumpTime==0) {
    throw std::invalid_argument("Jump time must be greater than start Time or 0 for no jump");
  }
  // jump delay must be greater than 0
  

}



/**
 * The initializeActuators method is call in onSetup to put pointers to 
 * specific actuators in the cablesWithTags array, as well as store the initial
 * rest lengths in the initialRL map.
 */
void LengthControllerYAML::initializeActuators(TensegrityModel& subject,
					       std::string tag) {
  //DEBUGGING
  std::cout << "Finding cables with the tag: " << tag << std::endl;
  // Pick out the actuators with the specified tag
  std::vector<tgBasicActuator*> foundActuators = subject.find<tgBasicActuator>(tag);
  std::cout << "The following cables were found and will be controlled: "
	    << std::endl;
  //Iterate through array and output strings to command line
  for (std::size_t i = 0; i < foundActuators.size(); i ++) {	
    std::cout << foundActuators[i]->getTags() << std::endl;
    // Also, add the rest length of the actuator at this time
    // to the list of all initial rest lengths.
    initialRL[foundActuators[i]->getTags()] = foundActuators[i]->getRestLength();
    //DEBUGGING:
    std::cout << "Cable rest length at t=0 is "
	      << initialRL[foundActuators[i]->getTags()] << std::endl;
    //DEBUG minlengthchange
    //std::cout << "target restlength:" <<m_minLength<< std::endl; 
    //pick out the actuator with tag c1
    
      //cout name of actuator is the impostor
            

    
  }
  std::vector<tgBasicActuator*> foundActuatorsc1 = subject.find<tgBasicActuator>("c1");
  std::vector<tgBasicActuator*> foundActuatorsc2 = subject.find<tgBasicActuator>("c2");
  std::vector<tgBasicActuator*> foundActuatorsc3 = subject.find<tgBasicActuator>("c3");
  std::cout << foundActuatorsc1[0]->getTags() << std::endl;
  std::cout << foundActuatorsc2[0]->getTags() << std::endl;
  std::cout << foundActuatorsc3[0]->getTags() << std::endl;
  // Add this list of actuators to the full list. Thanks to:
  // http://stackoverflow.com/questions/201718/concatenating-two-stdvectors
  cablesWithTags.insert( cablesWithTags.end(), foundActuators.begin(),
			 foundActuators.end() );
}

/**
 * For this controller, the onSetup method initializes the actuators,
 * which means just store pointers to them and record their rest lengths.
 * This method calls the helper initializeActuators.
 */
void LengthControllerYAML::onSetup(TensegrityModel& subject)
{
  
  std::cout << "Setting up the LengthControllerYAML controller." << std::endl;
  //	    << "Finding cables with tags: " << m_tagsToControl
  //	    << std::endl;
  cablesWithTags = {};
  // For all the strings in the list, call initializeActuators.
  std::vector<std::string>::iterator it;
  for( it = m_tagsToControl.begin(); it < m_tagsToControl.end(); it++ ) {
    // Call the helper for this tag.
    initializeActuators(subject, *it);
  }
  std::cout << "Finished setting up the controller." << std::endl;   
  
}

void LengthControllerYAML::onStep(TensegrityModel& subject, double dt)
{

  // First, increment the accumulator variable.
  m_timePassed += dt;
  //--------------------------------------------------------------------------------
  //charging
  //--------------------------------------------------------------------------------
  // Then, if it's passed the time to start the controller,
  if( m_timePassed > m_startTime&&!Ijumped) {
    //count number of steps
    
    // For each cable, check if its rest length is past the minimum,
    // otherwise adjust its length according to m_rate and dt.
    for (std::size_t i = 0; i < cablesWithTags.size(); i ++) {	
      double currRestLength = cablesWithTags[i]->getRestLength();
      // Calculate the minimum rest length for this cable.
      // Remember that m_minLength is a percent.
      double extra = 0;
      // printf("cable %s\n",foundActuatorsc1[0]->getTags());
      if(i==0){
        extra=m_extra1;
      }
      else if(i==1){
        extra=m_extra2;
      }
      else if(i==2){
        extra=m_extra3;
      }
      
      //print extra and cable name
      //std::cout<<" "<<cablesWithTags[i]->getTags()<<"extra: "<<extra<<std::endl;
      double minRestLength = initialRL[cablesWithTags[i]->getTags()] * (m_minLength+extra);
      // If the current rest length is still greater than the minimum,
      if( currRestLength > minRestLength ) {
	      // output a progress bar for the controller, to track when control occurs.
	      //std::cout << "." << i;
	      // Then, adjust the rest length of the actuator itself, according to
	      // m_rate and dt.
	      double nextRestLength = currRestLength - m_rate * dt;
        
        
	    //DEBUGGING
	    //std::cout << "Next Rest Length: " << nextRestLength << std::endl;
	    cablesWithTags[i]->setControlInput(nextRestLength,dt);
      }
    }   
  }
  //--------------------------------------------------------------------------------
  //jumping
  //--------------------------------------------------------------------------------
  //if jumptime is reached release tension from actuated cables c1
  //array with 3 zeros element named firsttime
  

  if (m_timePassed > m_jumpTime ) {

    // Code to execute when m_timePassed is greater than m_jumpTime + m_jumpdelay
    Ijumped = 1;
    //for eache cable remove tension if cable 1 wait also for delay
    for (std::size_t i = 0; i < cablesWithTags.size(); i ++) {	
      if(m_jumpdelay>0){
        if(i!=0||m_timePassed > m_jumpTime+m_jumpdelay){
          double currRestLength = cablesWithTags[i]->getRestLength();
          //std::cout<<"prima: "<<initialRL[cablesWithTags[i]->getTags()]<<" "<<cablesWithTags[i]->getRestLength()<<" "<<m_timePassed<<" ";
          cablesWithTags[i]->setControlInput(initialRL[cablesWithTags[i]->getTags()],dt); 
          //print when the first time every cable is released and time of reease
          // if(firstTime[i]==0){
          //   std::cout<<"cable "<<cablesWithTags[i]->getTags()<<" released at time: "<<m_timePassed<<std::endl;
          //   firstTime[i]=1;
          // }

        }
      } 
      else{
        if ((i != 1 && i != 2 )|| m_timePassed > m_jumpTime - m_jumpdelay) {
          double currRestLength = cablesWithTags[i]->getRestLength();
          cablesWithTags[i]->setControlInput(initialRL[cablesWithTags[i]->getTags()], dt);
          // if (firstTime[i] == 0) {
          //   std::cout << "caaaaable " << cablesWithTags[i]->getTags() << " released at time: " << m_timePassed << std::endl;
          //   firstTime[i] = 1;

          // }
        }
        
      }
    }
  }


}
void LengthControllerYAML::reset() {
    m_timePassed = 0.0;
    Ijumped = 0;
    
    // Optionally, reinitialize other necessary components
}
void LengthControllerYAML::nextStep() {
  //insrt values to sweep
  reset();

  //change minLength
  //m_minLength=m_minLength-0.1;


}
 
