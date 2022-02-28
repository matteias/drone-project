
"use strict";

let RemoveCrazyflie = require('./RemoveCrazyflie.js')
let StartTrajectory = require('./StartTrajectory.js')
let UpdateParams = require('./UpdateParams.js')
let Takeoff = require('./Takeoff.js')
let Stop = require('./Stop.js')
let SetGroupMask = require('./SetGroupMask.js')
let UploadTrajectory = require('./UploadTrajectory.js')
let sendPacket = require('./sendPacket.js')
let Land = require('./Land.js')
let GoTo = require('./GoTo.js')
let AddCrazyflie = require('./AddCrazyflie.js')

module.exports = {
  RemoveCrazyflie: RemoveCrazyflie,
  StartTrajectory: StartTrajectory,
  UpdateParams: UpdateParams,
  Takeoff: Takeoff,
  Stop: Stop,
  SetGroupMask: SetGroupMask,
  UploadTrajectory: UploadTrajectory,
  sendPacket: sendPacket,
  Land: Land,
  GoTo: GoTo,
  AddCrazyflie: AddCrazyflie,
};
