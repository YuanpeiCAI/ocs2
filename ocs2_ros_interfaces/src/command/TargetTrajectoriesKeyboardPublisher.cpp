/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_ros_interfaces/command/TargetTrajectoriesKeyboardPublisher.h"

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/Display.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectoriesKeyboardPublisher::TargetTrajectoriesKeyboardPublisher(::ros::NodeHandle& nodeHandle, const std::string& topicPrefix,
                                                                         const scalar_array_t& targetCommandLimits,
                                                                         CommandLineToTargetTrajectories commandLineToTargetTrajectoriesFun)
    : targetCommandLimits_(Eigen::Map<const vector_t>(targetCommandLimits.data(), targetCommandLimits.size())),
      commandLineToTargetTrajectoriesFun_(std::move(commandLineToTargetTrajectoriesFun)) {
  // observation subscriber
  auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
  };
  observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);
  // twist command subscriber
  auto twistCommandCallback = [this](const geometry_msgs::Twist::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestTwistCommandMutex_);
    latestTwistCommand_ = ros_msg_conversions::readTwistCommandMsg(*msg);
  };
  twistCommandSubscriber_ = nodeHandle.subscribe<geometry_msgs::Twist>(topicPrefix + "_twist_command", 1, twistCommandCallback);
  // mpc divergence service 
  mpcDivergeServiceServer_ = nodeHandle.advertiseService(topicPrefix + "_mpc_diverge", &TargetTrajectoriesKeyboardPublisher::mpcDivergeCallback, this);

  // mpc divergence service 
  mrtResetServiceClient_ = nodeHandle.serviceClient<ocs2_msgs::reset_mrt>(topicPrefix + "_mrt_reset");

  // Trajectories publisher
  targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, topicPrefix));

  // initialize command mapping from keyboard command
  mappingCommand_.insert(Position('w', {0.1, 0.0, 0.0, 0.0, 0.0, 0.0}));
  mappingCommand_.insert(Position('s', {-0.1, 0.0, 0.0, 0.0, 0.0, 0.0}));
  mappingCommand_.insert(Position('a', {0.0, 0.1, 0.0, 0.0, 0.0, 0.0}));
  mappingCommand_.insert(Position('d', {0.0, -0.1, 0.0, 0.0, 0.0, 0.0}));
  mappingCommand_.insert(Position('r', {0.0, 0.0, 0.0, 0.1, 0.0, 0.0}));
  mappingCommand_.insert(Position('e', {0.0, 0.0, 0.0, -0.1, 0.0, 0.0}));
  mappingCommand_.insert(Position('p', {0.0, 0.0, 0.0, 0.0, 0.1, 0.0}));
  mappingCommand_.insert(Position('o', {0.0, 0.0, 0.0, 0.0, -0.1, 0.0}));
  mappingCommand_.insert(Position('y', {0.0, 0.0, 0.0, 0.0, 0.0, 0.1}));
  mappingCommand_.insert(Position('t', {0.0, 0.0, 0.0, 0.0, 0.0, -0.1}));

}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesKeyboardPublisher::publishKeyboardCommand(const std::string& commadMsg) {
  while (ros::ok() && ros::master::check()) {
    // get command line
    std::cout << commadMsg << ": ";
    const vector_t commandLineInput = getCommandLine().cwiseMin(targetCommandLimits_).cwiseMax(-targetCommandLimits_);

    // display
    std::cout << "The following command is published: [" << toDelimitedString(commandLineInput) << "]\n\n";

    // get the latest observation
    ::ros::spinOnce();
    SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }

    // get TargetTrajectories
    const auto targetTrajectories = commandLineToTargetTrajectoriesFun_(commandLineInput, observation);

    // publish TargetTrajectories
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
  }  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesKeyboardPublisher::publishKeyboardIncrementalCommand(const std::string& commadMsg) {

  while (ros::ok() && ros::master::check()) {
    bool isMpcDiverged{};
    {
      std::lock_guard<std::mutex> lock(lastestMpcDivergenceMutex_);
      isMpcDiverged = lastestMpcDivergence_;
    }
    if (isMpcDiverged) {
      if (getMrtResetCommand()) {
        publishResetMrtCommand(); 
        {
          std::lock_guard<std::mutex> lock(lastestMpcDivergenceMutex_);
          lastestMpcDivergence_ = false;
        }
      }
      continue;
    }
    // get command line
    std::cout << commadMsg << ": ";

    vector_t incrementalCommand = getIncrementalCommand();

    // get the latest observation
    ::ros::spinOnce();
    SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }

    // get TargetTrajectories
    const auto targetTrajectories = commandLineToTargetTrajectoriesFun_(incrementalCommand, observation);

    // publish TargetTrajectories
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
  }  // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesKeyboardPublisher::publishResetMrtCommand(void) {

  ocs2_msgs::reset_mrt resetMrtSrv{};
  resetMrtSrv.request.reset = static_cast<uint8_t>(true);
  if (mrtResetServiceClient_.call(resetMrtSrv)) {
    printf("MRT reset command has been received!!\n");
    if (static_cast<bool>(resetMrtSrv.response.done)) {
      {
        std::lock_guard<std::mutex> lock(lastestMpcDivergenceMutex_);
        bool lastestMpcDivergence_ = false;
      }
      std::cerr << "\n#####################################################"
                << "\n#####################################################"
                << "\n#################  MRT is reset.  ###################"
                << "\n#####################################################"
                << "\n#####################################################\n";
    } else { std::cerr << "MRT is not reset!!!!!! \n"; }
  } else {std::cerr << "MRT does NOT recievie reset message!!!!!! \n"; }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesKeyboardPublisher::publishTwistCommand(void) {
  // wait for the system to start up
  ros::Duration(10.0).sleep();
  while (ros::ok() && ros::master::check()) {
    bool isMpcDiverged{};
    {
      std::lock_guard<std::mutex> lock(lastestMpcDivergenceMutex_);
      isMpcDiverged = lastestMpcDivergence_;
    }
    if (isMpcDiverged) {
      if (getMrtResetCommand()) {
        publishResetMrtCommand(); 
        {
          std::lock_guard<std::mutex> lock(lastestMpcDivergenceMutex_);
          lastestMpcDivergence_ = false;
        }
      }
      continue;
    }

    // get the latest observation
    ::ros::spinOnce();
    SystemObservation observation;
    {
      std::lock_guard<std::mutex> lock(latestObservationMutex_);
      observation = latestObservation_;
    }

    vector_t twistCommand(6);
    {
      std::lock_guard<std::mutex> lock(latestTwistCommandMutex_);
      twistCommand = latestTwistCommand_;
    }

    // get TargetTrajectories
    const auto targetTrajectories = commandLineToTargetTrajectoriesFun_(twistCommand, observation);

    // publish TargetTrajectories
    targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
  }  // end of while loop
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t TargetTrajectoriesKeyboardPublisher::getCommandLine() {
  // get command line as one long string
  auto shouldTerminate = []() { return !ros::ok() || !ros::master::check(); };
  const std::string line = getCommandLineString(shouldTerminate);

  // a line to words
  const std::vector<std::string> words = stringToWords(line);

  const size_t targetCommandSize = targetCommandLimits_.size();
  vector_t targetCommand = vector_t::Zero(targetCommandSize);
  for (size_t i = 0; i < std::min(words.size(), targetCommandSize); i++) {
    targetCommand(i) = static_cast<scalar_t>(stof(words[i]));
  }

  return targetCommand;
}

vector_t TargetTrajectoriesKeyboardPublisher::getIncrementalCommand( void ) {
    vector_t incrementalCommand{};
    incrementalCommand.setConstant(6, 0.0);
    const char commandLineInput = getIncrementalCommandChar();
    // find the corresponsing velocity command
    MappingIncrementalCommand::left_const_iterator iter = mappingCommand_.left.find(commandLineInput);
    if (iter == mappingCommand_.left.end()) {
      std::cout<<"The command ";
      std::cout<<commandLineInput;
      std::cout << " does not exist!!\n";
      return incrementalCommand;
    }
    // return vector command
    incrementalCommand(0) = iter->second[0];
    incrementalCommand(1) = iter->second[1];
    incrementalCommand(2) = iter->second[2];
    incrementalCommand(3) = iter->second[3];
    incrementalCommand(4) = iter->second[4];
    incrementalCommand(5) = iter->second[5];

    return incrementalCommand;
}

bool TargetTrajectoriesKeyboardPublisher::getMrtResetCommand( void ) {
    std::cout << "Please type \"reset\" to reset MRT node and MPC node: ";
    // get command line as one long string
    auto shouldTerminate = []() { return !ros::ok() || !ros::master::check(); };
    const std::string line = getCommandLineString(shouldTerminate);
    if (line == "reset") {
      return true;
    } else {
      std::cout<<"Please check your input. \n";
      return false;
    }
}

bool TargetTrajectoriesKeyboardPublisher::mpcDivergeCallback(ocs2_msgs::mpc_diverge::Request& req, ocs2_msgs::mpc_diverge::Response& res) {
  if (static_cast<bool>(req.isDiverge)) {
    std::lock_guard<std::mutex> lock(lastestMpcDivergenceMutex_);
    lastestMpcDivergence_ = true;
    res.done = static_cast<uint8_t>(true);
  }
  return true;
}


}  // namespace ocs2
