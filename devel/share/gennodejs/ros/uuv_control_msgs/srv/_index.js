
"use strict";

let SwitchToAutomatic = require('./SwitchToAutomatic.js')
let AddWaypoint = require('./AddWaypoint.js')
let InitHelicalTrajectory = require('./InitHelicalTrajectory.js')
let SetPIDParams = require('./SetPIDParams.js')
let InitRectTrajectory = require('./InitRectTrajectory.js')
let InitWaypointSet = require('./InitWaypointSet.js')
let GoTo = require('./GoTo.js')
let SetSMControllerParams = require('./SetSMControllerParams.js')
let GetPIDParams = require('./GetPIDParams.js')
let Hold = require('./Hold.js')
let InitCircularTrajectory = require('./InitCircularTrajectory.js')
let InitWaypointsFromFile = require('./InitWaypointsFromFile.js')
let SwitchToManual = require('./SwitchToManual.js')
let GoToIncremental = require('./GoToIncremental.js')
let IsRunningTrajectory = require('./IsRunningTrajectory.js')
let SetMBSMControllerParams = require('./SetMBSMControllerParams.js')
let GetWaypoints = require('./GetWaypoints.js')
let ClearWaypoints = require('./ClearWaypoints.js')
let GetSMControllerParams = require('./GetSMControllerParams.js')
let GetMBSMControllerParams = require('./GetMBSMControllerParams.js')
let StartTrajectory = require('./StartTrajectory.js')
let ResetController = require('./ResetController.js')

module.exports = {
  SwitchToAutomatic: SwitchToAutomatic,
  AddWaypoint: AddWaypoint,
  InitHelicalTrajectory: InitHelicalTrajectory,
  SetPIDParams: SetPIDParams,
  InitRectTrajectory: InitRectTrajectory,
  InitWaypointSet: InitWaypointSet,
  GoTo: GoTo,
  SetSMControllerParams: SetSMControllerParams,
  GetPIDParams: GetPIDParams,
  Hold: Hold,
  InitCircularTrajectory: InitCircularTrajectory,
  InitWaypointsFromFile: InitWaypointsFromFile,
  SwitchToManual: SwitchToManual,
  GoToIncremental: GoToIncremental,
  IsRunningTrajectory: IsRunningTrajectory,
  SetMBSMControllerParams: SetMBSMControllerParams,
  GetWaypoints: GetWaypoints,
  ClearWaypoints: ClearWaypoints,
  GetSMControllerParams: GetSMControllerParams,
  GetMBSMControllerParams: GetMBSMControllerParams,
  StartTrajectory: StartTrajectory,
  ResetController: ResetController,
};
