
"use strict";

let SetCurrentVelocity = require('./SetCurrentVelocity.js')
let TransformToSphericalCoord = require('./TransformToSphericalCoord.js')
let TransformFromSphericalCoord = require('./TransformFromSphericalCoord.js')
let SetOriginSphericalCoord = require('./SetOriginSphericalCoord.js')
let GetOriginSphericalCoord = require('./GetOriginSphericalCoord.js')
let SetCurrentModel = require('./SetCurrentModel.js')
let GetCurrentModel = require('./GetCurrentModel.js')
let SetCurrentDirection = require('./SetCurrentDirection.js')

module.exports = {
  SetCurrentVelocity: SetCurrentVelocity,
  TransformToSphericalCoord: TransformToSphericalCoord,
  TransformFromSphericalCoord: TransformFromSphericalCoord,
  SetOriginSphericalCoord: SetOriginSphericalCoord,
  GetOriginSphericalCoord: GetOriginSphericalCoord,
  SetCurrentModel: SetCurrentModel,
  GetCurrentModel: GetCurrentModel,
  SetCurrentDirection: SetCurrentDirection,
};
