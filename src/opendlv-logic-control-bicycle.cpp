/*
 * Copyright (C) 2021 Björnborg Nguyen
 * Copyright (C) 2019 Ola Benderius
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <string>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"

int32_t main(int32_t argc, char **argv)
{
  int32_t retCode{1};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if ((0 == commandlineArguments.count("cid")) ||
      (0 == commandlineArguments.count("freq")) ||
      // (0 == commandlineArguments.count("speed-max")) ||
      (0 == commandlineArguments.count("cog-to-front")) ||
      (0 == commandlineArguments.count("cog-to-rear")))
  {
    std::cerr << argv[0] << " controls a vehicle by using an inverted bicycle "
              << "model." << std::endl
              << "Usage:   " << argv[0] << " --cid=<CID> --freq=<Frequency to send> "
              // << "--speed-max=<Maximum speed>
              << "--cog-to-front=<Distance COG to front> "
              << "--cog-to-rear=<Distance COG to rear> "
              << "[--id-input=<Sender stamp, input message (default 0)>] "
              << "[--id-output=<Sender stamp, output messages (default 0)>] "
              << "[--steering-coef=<converting road wheel angle to steering>] "
              << "[--verbose]"
              << std::endl
              << "Example: " << argv[0]
              << " --cid=111 --freq=100 --cog-to-front=1.2 --cog-to-rear=1.3 --verbose" << std::endl;
  }
  else
  {
    uint32_t const senderStampInput{
        (commandlineArguments.count("id-input") != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id-input"])) : 0};
    uint32_t const senderStampOutput{
        (commandlineArguments.count("id-output") != 0) ? static_cast<uint32_t>(std::stoi(commandlineArguments["id-output"])) : 0};
    bool const verbose{commandlineArguments.count("verbose") != 0};

    float const cogToFront{std::stof(commandlineArguments["cog-to-front"])};
    float const cogToRear{std::stof(commandlineArguments["cog-to-rear"])};
    // float const speedMax{std::stof(commandlineArguments["speed-max"])};
    float const vehicleLength = cogToFront + cogToRear;
    float const steeringCoef{
        (commandlineArguments.count("steering-coef") != 0) ? std::stof(commandlineArguments["steering-coef"]) : 1.0f};

    cluon::OD4Session od4{
        static_cast<uint16_t>(std::stoi(commandlineArguments["cid"]))};

    std::mutex timestampLastRequestMutex;
    // Zero timestamp
    cluon::data::TimeStamp timestampLastRequest;

    std::mutex requestMutex;
    float vxRequest{0.0f};
    float yawRateRequest{0.0f};

    auto onGroundMotionRequest{
        [&vxRequest, &yawRateRequest, &senderStampInput,
         &requestMutex, &timestampLastRequest, &timestampLastRequestMutex, &verbose, &od4](cluon::data::Envelope &&envelope)
        {
          if (envelope.senderStamp() == senderStampInput)
          {
            if (cluon::time::deltaInMicroseconds(envelope.sampleTimeStamp(), timestampLastRequest) > 0)
            {
              std::lock_guard<std::mutex> lock(timestampLastRequestMutex);
              timestampLastRequest = envelope.sampleTimeStamp();
            }
            auto msg = cluon::extractMessage<opendlv::proxy::GroundMotionRequest>(
                std::move(envelope));

            {
              std::lock_guard<std::mutex> lock(requestMutex);
              vxRequest = msg.vx();
              yawRateRequest = msg.yawRate();
            }

            // Hack to use opendlv-logic-control-speed
            opendlv::proxy::GroundSpeedRequest gsr;
            gsr.groundSpeed(vxRequest);
            od4.send(gsr);

            if (verbose)
            {
              std::cout << "Got request, vx=" << msg.vx() << " yawRate="
                        << msg.yawRate() << std::endl;
            }
          }
        }};

    od4.dataTrigger(opendlv::proxy::GroundMotionRequest::ID(),
                    onGroundMotionRequest);

    // Hack to use opendlv-logic-control-speed
    std::mutex speedControlMutex;
    float speedControl = 0.0f;
    auto onGroundDecelerationRequest{
        [&speedControl, &speedControlMutex,
         &verbose](cluon::data::Envelope &&envelope)
        {
          auto msg =
              cluon::extractMessage<opendlv::proxy::GroundDecelerationRequest>(
                  std::move(envelope));

          {
            std::lock_guard<std::mutex> lock(speedControlMutex);
            speedControl = msg.groundDeceleration();
          }

          if (verbose)
          {
            std::cout << "Got deceleration request, req="
                      << msg.groundDeceleration() << std::endl;
          }
        }};
    od4.dataTrigger(opendlv::proxy::GroundDecelerationRequest::ID(),
                    onGroundDecelerationRequest);

    auto onPedalPositionRequest{
        [&speedControl, &speedControlMutex,
         &verbose](cluon::data::Envelope &&envelope)
        {
          auto msg =
              cluon::extractMessage<opendlv::proxy::PedalPositionRequest>(
                  std::move(envelope));

          {
            std::lock_guard<std::mutex> lock(speedControlMutex);
            speedControl = msg.position();
          }

          if (verbose)
          {
            std::cout << "Got pedal position request, req="
                      << msg.position() << std::endl;
          }
        }};
    od4.dataTrigger(opendlv::proxy::PedalPositionRequest::ID(),
                    onPedalPositionRequest);

    // Using opendlv-logic-control-speed
    std::mutex groundAccelerationRequestMutex;
    float groundAccelerationRequest = 0.0f;
    auto onGroundAccelerationRequest{
        [&groundAccelerationRequest, &groundAccelerationRequestMutex,
         &timestampLastRequest, &timestampLastRequestMutex,
         &verbose](cluon::data::Envelope &&envelope)
        {
          if (cluon::time::deltaInMicroseconds(envelope.sampleTimeStamp(), timestampLastRequest) > 0)
          {
            std::lock_guard<std::mutex> lock(timestampLastRequestMutex);
            timestampLastRequest = envelope.sampleTimeStamp();
          }
          auto msg =
              cluon::extractMessage<opendlv::proxy::GroundAccelerationRequest>(
                  std::move(envelope));

          {
            std::lock_guard<std::mutex> lock(groundAccelerationRequestMutex);
            groundAccelerationRequest = msg.groundAcceleration();
          }

          if (verbose)
          {
            std::cout << "Got ground acceleration request, req="
                      << msg.groundAcceleration() << std::endl;
          }
        }};
    od4.dataTrigger(opendlv::proxy::GroundAccelerationRequest::ID(),
                    onGroundAccelerationRequest);

    double groundSpeed{0.0};
    std::mutex groundSpeedMutex;
    auto onGroundSpeedReading{
        [&groundSpeed, &groundSpeedMutex](
            cluon::data::Envelope &&envelope)
        {
          std::lock_guard<std::mutex> lock(groundSpeedMutex);
          auto groundSpeedReading = cluon::extractMessage<opendlv::proxy::GroundSpeedReading>(
              std::move(envelope));
          groundSpeed = groundSpeedReading.groundSpeed();
        }};
    od4.dataTrigger(opendlv::proxy::GroundSpeedReading::ID(),
                    onGroundSpeedReading);

    auto atFrequency{
        [&od4, &vehicleLength, &cogToRear, &vxRequest,
         &yawRateRequest, &senderStampOutput, &requestMutex,
         &groundAccelerationRequest, &groundAccelerationRequestMutex,
         &timestampLastRequest, &timestampLastRequestMutex,
         &steeringCoef, &groundSpeed, &groundSpeedMutex, &verbose]() -> bool
        {
          float delta{0};
          {
            std::lock_guard<std::mutex> lock(requestMutex);
            std::lock_guard<std::mutex> lock(groundSpeedMutex);
            if (vxRequest > 0.1 && groundSpeed > 0.1)
            {
              // 2021-08-09 21:41:53 | This is too weak. Roughly 90 deg steering. 360 is possible.
              // 2021-08-12 16:12:48 | This is now scaled linearly with steeringCoef of 16.8 according to roadwheel angle to steering wheel.
              delta = steeringCoef * static_cast<float>(atan2(
                                         vehicleLength * tan(cogToRear * yawRateRequest / vxRequest),
                                         cogToRear));
            }
          }

          cluon::data::TimeStamp ts = cluon::time::now();
          opendlv::proxy::ActuationRequest ar;
          ar.acceleration(-1.5);
          // Only send positive acceleration requests if fresh, 1 second
          if (cluon::time::deltaInMicroseconds(ts, timestampLastRequest) < 1000000)
          {
            std::lock_guard<std::mutex> lock(groundAccelerationRequestMutex);
            ar.acceleration(groundAccelerationRequest);
          }
          // Only send valued acceleration requests if fresh, 1 second

          ar.steering(delta);
          ar.isValid(true);

          od4.send(ar, ts, senderStampOutput);

          if (verbose)
          {
            std::cout << "Timediff=" << cluon::time::deltaInMicroseconds(ts, timestampLastRequest) << std::endl;
            std::cout << "Sending steering wheel angle=" << ar.steering() << ", acceleration=" << ar.acceleration() << std::endl;
          }

          return true;
        }};

    od4.timeTrigger(std::stoi(commandlineArguments["freq"]), atFrequency);
    opendlv::proxy::ActuationRequest ar;
    ar.acceleration(-1.5);
    ar.steering(0.0);
    ar.isValid(true);
    od4.send(ar, cluon::time::now(), senderStampOutput);
    retCode = 0;
  }
  return retCode;
}
