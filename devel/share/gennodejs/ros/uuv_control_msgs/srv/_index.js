
"use strict";

let SwitchToManual = require('./SwitchToManual.js')
let InitHelicalTrajectory = require('./InitHelicalTrajectory.js')
let GoToIncremental = require('./GoToIncremental.js')
let GetPIDParams = require('./GetPIDParams.js')
let InitWaypointSet = require('./InitWaypointSet.js')
let SetPIDParams = require('./SetPIDParams.js')
let InitWaypointsFromFile = require('./InitWaypointsFromFile.js')
let GetWaypoints = require('./GetWaypoints.js')
let InitRectTrajectory = require('./InitRectTrajectory.js')
let SetMBSMControllerParams = require('./SetMBSMControllerParams.js')
let SwitchToAutomatic = require('./SwitchToAutomatic.js')
let ResetController = require('./ResetController.js')
let AddWaypoint = require('./AddWaypoint.js')
let SetSMControllerParams = require('./SetSMControllerParams.js')
let IsRunningTrajectory = require('./IsRunningTrajectory.js')
let GetMBSMControllerParams = require('./GetMBSMControllerParams.js')
let GetSMControllerParams = require('./GetSMControllerParams.js')
let InitCircularTrajectory = require('./InitCircularTrajectory.js')
let Hold = require('./Hold.js')
let ClearWaypoints = require('./ClearWaypoints.js')
let StartTrajectory = require('./StartTrajectory.js')
let GoTo = require('./GoTo.js')

module.exports = {
  SwitchToManual: SwitchToManual,
  InitHelicalTrajectory: InitHelicalTrajectory,
  GoToIncremental: GoToIncremental,
  GetPIDParams: GetPIDParams,
  InitWaypointSet: InitWaypointSet,
  SetPIDParams: SetPIDParams,
  InitWaypointsFromFile: InitWaypointsFromFile,
  GetWaypoints: GetWaypoints,
  InitRectTrajectory: InitRectTrajectory,
  SetMBSMControllerParams: SetMBSMControllerParams,
  SwitchToAutomatic: SwitchToAutomatic,
  ResetController: ResetController,
  AddWaypoint: AddWaypoint,
  SetSMControllerParams: SetSMControllerParams,
  IsRunningTrajectory: IsRunningTrajectory,
  GetMBSMControllerParams: GetMBSMControllerParams,
  GetSMControllerParams: GetSMControllerParams,
  InitCircularTrajectory: InitCircularTrajectory,
  Hold: Hold,
  ClearWaypoints: ClearWaypoints,
  StartTrajectory: StartTrajectory,
  GoTo: GoTo,
};
