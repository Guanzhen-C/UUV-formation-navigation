
"use strict";

let SetCurrentVelocity = require('./SetCurrentVelocity.js')
let CreatePassiveScalarTurbulentPlume = require('./CreatePassiveScalarTurbulentPlume.js')
let DeletePlume = require('./DeletePlume.js')
let GetPlumeConfiguration = require('./GetPlumeConfiguration.js')
let StorePlumeState = require('./StorePlumeState.js')
let GetPlumeSourcePosition = require('./GetPlumeSourcePosition.js')
let SetPlumeSourcePosition = require('./SetPlumeSourcePosition.js')
let GetNumParticles = require('./GetNumParticles.js')
let SetCurrentModel = require('./SetCurrentModel.js')
let SetPlumeLimits = require('./SetPlumeLimits.js')
let CreateSpheroidPlume = require('./CreateSpheroidPlume.js')
let GetCurrentModel = require('./GetCurrentModel.js')
let SetPlumeConfiguration = require('./SetPlumeConfiguration.js')
let LoadPlumeParticles = require('./LoadPlumeParticles.js')
let SetCurrentDirection = require('./SetCurrentDirection.js')

module.exports = {
  SetCurrentVelocity: SetCurrentVelocity,
  CreatePassiveScalarTurbulentPlume: CreatePassiveScalarTurbulentPlume,
  DeletePlume: DeletePlume,
  GetPlumeConfiguration: GetPlumeConfiguration,
  StorePlumeState: StorePlumeState,
  GetPlumeSourcePosition: GetPlumeSourcePosition,
  SetPlumeSourcePosition: SetPlumeSourcePosition,
  GetNumParticles: GetNumParticles,
  SetCurrentModel: SetCurrentModel,
  SetPlumeLimits: SetPlumeLimits,
  CreateSpheroidPlume: CreateSpheroidPlume,
  GetCurrentModel: GetCurrentModel,
  SetPlumeConfiguration: SetPlumeConfiguration,
  LoadPlumeParticles: LoadPlumeParticles,
  SetCurrentDirection: SetCurrentDirection,
};
